"""In-process fake controller board implementing ``spec/protocol.md`` v1.

It is faithful to the *spec*, not to the C firmware. Where the spec leaves a choice open,
the choice is listed in ``README.md`` ("Spec interpretations") and marked ``INTERP`` here.

Two ways to drive it:

* deterministic: ``board.receive(bytes)``, ``board.step(dt_s)``, ``board.take_output()``;
* real time: ``MockTransport(board)`` runs a 1 kHz-ish thread and looks like a serial port
  (``write`` / ``read`` / ``close``) so :class:`armctl.client.ArmClient` can use it.

Model: per servo, a quintic commanded trajectory (replanned from the current commanded
position/velocity/acceleration on every SET_TARGETS) tracked by a first-order servo lag.
Servo state is kept in encoder ticks so CONFIG offset/direction behave like the real thing.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

from . import protocol as P
from .kinematics import ArmModel, default_model

N = P.N_JOINTS
CONTROL_DT = 0.001  # 1 kHz control loop


def quintic_coeffs(p0, v0, a0, pf, T):
    """Coefficients (6, n) of the quintic from (p0, v0, a0) to (pf, 0, 0) in time T."""
    p0, v0, a0, pf = (np.asarray(x, dtype=float) for x in (p0, v0, a0, pf))
    h = pf - p0
    c0, c1, c2 = p0, v0, a0 / 2
    c3 = (20 * h - 12 * v0 * T - 3 * a0 * T**2) / (2 * T**3)
    c4 = (-30 * h + 16 * v0 * T + 3 * a0 * T**2) / (2 * T**4)
    c5 = (12 * h - 6 * v0 * T - a0 * T**2) / (2 * T**5)
    return np.stack([c0, c1, c2, c3, c4, c5])


def quintic_eval(c, t):
    """Position, velocity, acceleration of quintic coeffs ``c`` at time ``t``."""
    p = c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))))
    v = c[1] + t * (2 * c[2] + t * (3 * c[3] + t * (4 * c[4] + t * 5 * c[5])))
    a = 2 * c[2] + t * (6 * c[3] + t * (12 * c[4] + t * 20 * c[5]))
    return p, v, a


@dataclass
class JointConfig:
    offset_ticks: int = 0
    direction: int = 1
    limit_min_mrad: int = -3000
    limit_max_mrad: int = 3000
    vmax_mrad_s: int = 3000
    current_limit_ma: int = 1000


@dataclass
class MockStats:
    rx_frames: int = 0
    nacks_sent: int = 0
    acks_sent: int = 0
    states_sent: int = 0
    acks_dropped: int = 0


class MockBoard:
    FW_VERSION = 0x0100

    def __init__(
        self,
        model: ArmModel | None = None,
        *,
        initial_q=None,
        servo_tau_s: float = 0.015,
        mount_offset_ticks=None,
        mount_direction=None,
    ):
        self.model = model or default_model()
        self.lock = threading.RLock()
        self.k = 2 * math.pi / self.model.ticks_per_rev  # rad per tick
        self.center = self.model.center_tick
        lo, hi = self.model.lower6, self.model.upper6
        self.jcfg = [
            JointConfig(
                limit_min_mrad=P.rad_to_mrad(lo[i]),
                limit_max_mrad=P.rad_to_mrad(hi[i]),
                vmax_mrad_s=P.rad_to_mrad(self.model.vmax[i]),
            )
            for i in range(N)
        ]
        self.watchdog_hold_ms = 100
        self.watchdog_off_ms = 500
        self.stream_period_us = 5000
        self.servo_tau = servo_tau_s

        # Hidden "physical" servo mounting: true joint angle = mdir * (ticks - center - moff) * k.
        self.mount_offset = np.asarray(mount_offset_ticks if mount_offset_ticks is not None else [0] * N, float)
        self.mount_dir = np.asarray(mount_direction if mount_direction is not None else [1] * N, float)
        q0 = np.zeros(N) if initial_q is None else np.asarray(initial_q, dtype=float)
        self.ticks = self.center + self.mount_offset + q0 / (self.mount_dir * self.k)
        self.vel_rad_s = np.zeros(N)

        self.t_us = 0.0
        self.mode = P.Mode.IDLE
        self.faults = P.Fault.NONE
        self.torque = False
        self.hw_estop_open = False
        self.bus_mv = 12000
        self.temp_c = np.full(N, 30)
        self.last_host_seq = 0
        self.last_rx_us = 0.0
        self.tx_seq = 0
        self.next_state_us = 0.0

        # commanded trajectory in joint space (rad)
        self.traj_c = None
        self.traj_t0 = 0.0
        self.traj_T = 0.0
        self.cmd_p = self.measured_q()
        self.cmd_v = np.zeros(N)
        self.cmd_a = np.zeros(N)

        self.decoder = P.FrameDecoder()
        self.out = bytearray()
        self.stats = MockStats()
        self.drop_acks = 0  # test hook: silently drop the next N ACK frames
        self.received: deque[P.Frame] = deque(maxlen=10000)  # recent valid host frames (tests)

    # ------------------------------------------------------------------ joint mapping
    def _ticks_to_q(self, ticks):
        d = np.array([c.direction for c in self.jcfg], float)
        off = np.array([c.offset_ticks for c in self.jcfg], float)
        # INTERP: joint_rad = direction * (ticks - center_tick - offset_ticks) * 2pi/ticks_per_rev
        return d * (ticks - self.center - off) * self.k

    def _q_to_ticks(self, q):
        d = np.array([c.direction for c in self.jcfg], float)
        off = np.array([c.offset_ticks for c in self.jcfg], float)
        return np.asarray(q) / (d * self.k) + self.center + off

    def measured_q(self) -> np.ndarray:
        """Joint angles as the board reports them (through CONFIG offset/direction)."""
        return self._ticks_to_q(self.ticks)

    def true_q(self) -> np.ndarray:
        """Physical joint angles (through the hidden mount offset/direction)."""
        return self.mount_dir * (self.ticks - self.center - self.mount_offset) * self.k

    def backdrive(self, q_true) -> None:
        """Test hook: a human moves the (limp) arm to physical angles ``q_true``."""
        with self.lock:
            if self.torque:
                return
            self.ticks = self.center + self.mount_offset + np.asarray(q_true, float) / (self.mount_dir * self.k)

    # ------------------------------------------------------------------ fault hooks
    def inject_fault(self, bits: P.Fault) -> None:
        with self.lock:
            self._latch(P.Fault(bits))

    def set_hw_estop(self, open_: bool) -> None:
        with self.lock:
            self.hw_estop_open = open_
            if open_:
                self._latch(P.Fault.ESTOP_HW)

    def set_bus_voltage(self, mv: int) -> None:
        with self.lock:
            self.bus_mv = mv
            self._update_undervolt()

    def _update_undervolt(self) -> None:
        # INTERP: BUS_UNDERVOLT is not marked latched -> it tracks the voltage and does not
        # by itself force FAULT.
        if self.bus_mv < 10500:
            self.faults |= P.Fault.BUS_UNDERVOLT
        else:
            self.faults &= ~P.Fault.BUS_UNDERVOLT

    def _latch(self, bits: P.Fault) -> None:
        self.faults |= bits
        if self.faults & P.LATCHED_FAULTS:
            self._torque_off()
            self.mode = P.Mode.FAULT

    def _torque_off(self) -> None:
        self.torque = False
        self.traj_c = None
        self.cmd_v[:] = 0
        self.cmd_a[:] = 0

    # ------------------------------------------------------------------ I/O
    def receive(self, data: bytes) -> None:
        with self.lock:
            for fr in self.decoder.feed(data):
                self._handle(fr)

    def take_output(self) -> bytes:
        with self.lock:
            out = bytes(self.out)
            self.out.clear()
            return out

    def _send(self, msg: P.Message) -> None:
        if isinstance(msg, P.Ack) and self.drop_acks > 0:
            self.drop_acks -= 1
            self.stats.acks_dropped += 1
            return
        self.out += P.encode(msg, self.tx_seq)
        self.tx_seq = (self.tx_seq + 1) & 0xFF
        if isinstance(msg, P.Ack):
            self.stats.acks_sent += 1
        elif isinstance(msg, P.Nack):
            self.stats.nacks_sent += 1
        elif isinstance(msg, P.State):
            self.stats.states_sent += 1

    def _nack(self, fr: P.Frame, err: P.NackError) -> None:
        self._send(P.Nack(fr.seq, fr.type, err))

    def _ack(self, fr: P.Frame) -> None:
        self._send(P.Ack(fr.seq, fr.type))

    # ------------------------------------------------------------------ frame handling
    def _handle(self, fr: P.Frame) -> None:
        self.stats.rx_frames += 1
        self.received.append(fr)
        # INTERP: any CRC-valid frame counts as "valid" for the watchdog and last_host_seq,
        # even if it is then NACKed.
        self.last_rx_us = self.t_us
        self.last_host_seq = fr.seq
        if self.mode == P.Mode.HOLD:
            self.mode = P.Mode.ACTIVE  # INTERP: host is back; keep holding the frozen setpoint

        cls = P.MESSAGE_CLASSES.get(fr.type)
        if cls is None or fr.type not in P.HOST_TO_BOARD:
            return self._nack(fr, P.NackError.UNKNOWN_TYPE)
        if len(fr.payload) != cls.SIZE:
            return self._nack(fr, P.NackError.BAD_LENGTH)
        msg = cls.unpack(fr.payload)
        handler = {
            P.MsgType.PING: self._on_ping,
            P.MsgType.SET_TARGETS: self._on_set_targets,
            P.MsgType.CONFIG: self._on_config,
            P.MsgType.ESTOP: self._on_estop,
            P.MsgType.CLEAR_FAULT: self._on_clear_fault,
            P.MsgType.SET_STREAM: self._on_set_stream,
        }[P.MsgType(fr.type)]
        handler(fr, msg)

    def _on_ping(self, fr, msg) -> None:
        self._send(P.Pong(self.FW_VERSION, int(self.t_us // 1000) & 0xFFFFFFFF, P.PROTO_VER))

    def _on_set_targets(self, fr, msg: P.SetTargets) -> None:
        if msg.flags & ~P.FLAG_TORQUE_ENABLE:
            return self._nack(fr, P.NackError.BAD_VALUE)
        if not 1 <= msg.move_ms <= 5000:
            return self._nack(fr, P.NackError.BAD_VALUE)
        if not msg.torque_enable:
            self._torque_off()
            self.mode = P.Mode.FAULT if self.faults & P.LATCHED_FAULTS else P.Mode.IDLE
            return self._ack(fr)
        if self.faults & P.LATCHED_FAULTS:
            return self._nack(fr, P.NackError.REJECTED_FAULTED)
        if not self.torque:
            # INTERP: torque-on from limp starts the commanded state at the measured position.
            self.cmd_p = self.measured_q()
            self.cmd_v[:] = 0
            self.cmd_a[:] = 0
            self.torque = True
        self.mode = P.Mode.ACTIVE
        self._plan(np.array(msg.pos_mrad, float) / 1000.0, msg.move_ms / 1000.0)
        self._ack(fr)

    def _plan(self, target: np.ndarray, T: float) -> None:
        lo = np.array([c.limit_min_mrad for c in self.jcfg]) / 1000.0
        hi = np.array([c.limit_max_mrad for c in self.jcfg]) / 1000.0
        vmax = np.array([c.vmax_mrad_s for c in self.jcfg]) / 1000.0
        clamped = np.clip(target, lo, hi)
        clamp = bool(np.any(clamped != target))
        # Stretch the (common) duration until every joint's peak speed is within vmax.
        # INTERP: with a non-zero initial velocity/acceleration (mid-move replan) a longer
        # quintic does not always lower the peak (and |v0| itself may exceed vmax), so the
        # search is over a bounded geometric grid of durations; the start speed is allowed,
        # and if nothing satisfies vmax the duration with the lowest peak ratio is used.
        ts = np.linspace(0, 1, 65)
        allowed = np.maximum(vmax, np.abs(self.cmd_v) * (1 + 1e-6))
        best = None
        for k in range(48):
            Tk = T * (1.15**k)
            if Tk > 30.0:
                break
            c = quintic_coeffs(self.cmd_p, self.cmd_v, self.cmd_a, clamped, Tk)
            _, v, _ = quintic_eval(c[:, None, :], (ts * Tk)[:, None])
            ratio = float(np.max(np.max(np.abs(v), axis=0) / allowed))
            if best is None or ratio < best[0]:
                best = (ratio, Tk, c)
            if ratio <= 1.0 + 1e-9:
                break
        ratio, Tk, c = best
        if Tk != T:
            clamp = True
        T = Tk
        if clamp:
            self.faults |= P.Fault.LIMIT_CLAMP
        else:
            self.faults &= ~P.Fault.LIMIT_CLAMP
        self.traj_c, self.traj_t0, self.traj_T = c, self.t_us, T

    def _on_config(self, fr, msg: P.Config) -> None:
        try:
            key = P.ConfigKey(msg.key)
        except ValueError:
            return self._nack(fr, P.NackError.UNKNOWN_CONFIG_KEY)
        v = msg.value
        if key in P.GLOBAL_CONFIG_KEYS:
            if msg.joint != P.GLOBAL_JOINT:
                return self._nack(fr, P.NackError.BAD_VALUE)
            if key == P.ConfigKey.WATCHDOG_HOLD_MS:
                if not 1 <= v < self.watchdog_off_ms:
                    return self._nack(fr, P.NackError.BAD_VALUE)
                self.watchdog_hold_ms = v
            else:
                if not self.watchdog_hold_ms < v <= 60000:
                    return self._nack(fr, P.NackError.BAD_VALUE)
                self.watchdog_off_ms = v
            return self._ack(fr)
        if not 0 <= msg.joint < N:
            return self._nack(fr, P.NackError.BAD_VALUE)
        jc = self.jcfg[msg.joint]
        tpr = self.model.ticks_per_rev
        ok = {
            P.ConfigKey.OFFSET_TICKS: -tpr < v < tpr,
            P.ConfigKey.DIRECTION: v in (1, -1),
            P.ConfigKey.LIMIT_MIN_MRAD: -32768 <= v < jc.limit_max_mrad,
            P.ConfigKey.LIMIT_MAX_MRAD: jc.limit_min_mrad < v <= 32767,
            P.ConfigKey.VMAX_MRAD_S: 0 < v <= 32767,
            P.ConfigKey.CURRENT_LIMIT_MA: 0 < v <= 32767,
        }[key]
        if not ok:
            return self._nack(fr, P.NackError.BAD_VALUE)
        attr = {
            P.ConfigKey.OFFSET_TICKS: "offset_ticks",
            P.ConfigKey.DIRECTION: "direction",
            P.ConfigKey.LIMIT_MIN_MRAD: "limit_min_mrad",
            P.ConfigKey.LIMIT_MAX_MRAD: "limit_max_mrad",
            P.ConfigKey.VMAX_MRAD_S: "vmax_mrad_s",
            P.ConfigKey.CURRENT_LIMIT_MA: "current_limit_ma",
        }[key]
        setattr(jc, attr, v)
        if key in (P.ConfigKey.OFFSET_TICKS, P.ConfigKey.DIRECTION) and not self.torque:
            self.cmd_p = self.measured_q()
        self._ack(fr)

    def _on_estop(self, fr, msg) -> None:
        self._latch(P.Fault.ESTOP_SOFT)
        self._ack(fr)

    def _on_clear_fault(self, fr, msg: P.ClearFault) -> None:
        mask = P.Fault(msg.mask & 0xFF)
        if self.hw_estop_open:
            mask &= ~P.Fault.ESTOP_HW  # INTERP: cannot clear while the loop is open; still ACK
        mask &= ~P.Fault.BUS_UNDERVOLT  # tracks the voltage
        self.faults &= ~mask
        # INTERP: mode stays FAULT until SET_TARGETS with torque enable (two-step clear).
        self._ack(fr)

    def _on_set_stream(self, fr, msg: P.SetStream) -> None:
        if msg.period_us != 0 and msg.period_us < 1000:
            return self._nack(fr, P.NackError.BAD_VALUE)
        self.stream_period_us = msg.period_us
        self.next_state_us = self.t_us
        self._ack(fr)

    # ------------------------------------------------------------------ simulation
    def step(self, dt: float) -> None:
        """Advance the board by ``dt`` seconds in 1 ms control cycles."""
        with self.lock:
            n = max(1, int(round(dt / CONTROL_DT)))
            for _ in range(n):
                self._cycle()

    def _cycle(self) -> None:
        self.t_us += CONTROL_DT * 1e6
        self._update_undervolt()
        # watchdog (INTERP: only armed while torque is on)
        silent_ms = (self.t_us - self.last_rx_us) / 1000.0
        if self.torque:
            if silent_ms > self.watchdog_off_ms:
                self._latch(P.Fault.WATCHDOG)
            elif self.mode == P.Mode.ACTIVE and silent_ms > self.watchdog_hold_ms:
                self.mode = P.Mode.HOLD
                self.traj_c = None
                self.cmd_v[:] = 0
                self.cmd_a[:] = 0
        # commanded trajectory
        if self.torque and self.traj_c is not None:
            t = min((self.t_us - self.traj_t0) / 1e6, self.traj_T)
            p, v, a = quintic_eval(self.traj_c, t)
            self.cmd_p, self.cmd_v, self.cmd_a = p, v, a
            if t >= self.traj_T:
                self.traj_c = None
                self.cmd_v[:] = 0
                self.cmd_a[:] = 0
        # servo response
        prev = self.ticks.copy()
        if self.torque:
            goal = self._q_to_ticks(self.cmd_p)
            alpha = 1.0 - math.exp(-CONTROL_DT / self.servo_tau)
            self.ticks = self.ticks + (goal - self.ticks) * alpha
        else:
            self.cmd_p = self.measured_q()
        d = np.array([c.direction for c in self.jcfg], float)
        self.vel_rad_s = d * (self.ticks - prev) * self.k / CONTROL_DT
        # streaming
        if self.stream_period_us and self.t_us >= self.next_state_us:
            self._send(self.make_state())
            self.next_state_us = max(self.next_state_us + self.stream_period_us, self.t_us)

    def make_state(self) -> P.State:
        q = self.measured_q()
        if self.torque:
            err_mrad = np.abs(self._ticks_to_q(self._q_to_ticks(self.cmd_p)) - q) * 1000
            cur = np.minimum(150 + 2.0 * err_mrad, 2700)
        else:
            cur = np.zeros(N)
        cmd_mrad = [P.rad_to_mrad(x) for x in self.cmd_p]
        joints = [
            P.JointState(
                pos_mrad=P.rad_to_mrad(q[i]),
                vel_mrad_s=P.rad_to_mrad(self.vel_rad_s[i]),
                current_ma=int(cur[i]),
                temp_c=int(self.temp_c[i]),
                status=1 if self.torque else 0,  # INTERP: joint_status bits are not specified
            )
            for i in range(N)
        ]
        return P.State(
            t_us=int(self.t_us) & 0xFFFFFFFF,
            fault_flags=int(self.faults),
            mode=int(self.mode),
            last_host_seq=self.last_host_seq,
            joints=joints,
            bus_mv=self.bus_mv,
            bus_ma=int(min(cur.sum(), 32767)),
            loop_jitter_max_us=0,
            loop_overruns=0,
            cmd_pos_crc=P.commanded_pos_crc(cmd_mrad),
        )


class MockTransport:
    """Serial-port-like byte stream to a :class:`MockBoard` stepped by a real-time thread."""

    def __init__(self, board: MockBoard | None = None, *, read_timeout: float = 0.01, start: bool = True):
        self.board = board or MockBoard()
        self.read_timeout = read_timeout
        self._rx = bytearray()
        self._cv = threading.Condition()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="mockboard", daemon=True)
        self.closed = False
        self.drop_host_frames = 0  # test hook: drop next N writes entirely
        if start:
            self._thread.start()

    def _run(self) -> None:
        last = time.monotonic()
        while not self._stop.is_set():
            time.sleep(0.001)
            now = time.monotonic()
            n = int((now - last) / CONTROL_DT)
            if n <= 0:
                continue
            n = min(n, 200)  # do not spiral if the host was suspended
            last += n * CONTROL_DT if n < 200 else (now - last)
            self.board.step(n * CONTROL_DT)
            self._pull()

    # serial-like API
    def write(self, data: bytes) -> int:
        if self.closed:
            raise OSError("transport closed")
        if self.drop_host_frames > 0:
            self.drop_host_frames -= 1
            return len(data)
        self.board.receive(bytes(data))
        self._pull()  # PONG/ACK go out without waiting for the next tick
        return len(data)

    def _pull(self) -> None:
        with self._cv:  # take + append atomically so frames keep their order
            out = self.board.take_output()
            if out:
                self._rx += out
                self._cv.notify_all()

    def read(self, size: int = 4096) -> bytes:
        with self._cv:
            if not self._rx and not self.closed:
                self._cv.wait(self.read_timeout)
            data = bytes(self._rx[:size])
            del self._rx[:size]
            return data

    def close(self) -> None:
        self.closed = True
        self._stop.set()
        if self._thread.is_alive() and threading.current_thread() is not self._thread:
            self._thread.join(timeout=1.0)
        with self._cv:
            self._cv.notify_all()
