"""Host client for the arm controller board.

``ArmClient`` owns a byte transport (pyserial port, :class:`~armctl.mockboard.MockTransport`
or anything with ``write(bytes)``, ``read(n) -> bytes`` that returns ``b""`` on timeout and
``close()``), runs a reader thread that keeps the latest STATE, matches ACK/NACK to requests
by SEQ, retries on timeout, and sends a PING heartbeat so the board watchdog stays fed while
the host is idle.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Protocol

import numpy as np

from . import protocol as P
from .safety import SafetyReport, SafetyShim

log = logging.getLogger(__name__)


class Transport(Protocol):
    def write(self, data: bytes) -> int | None: ...
    def read(self, size: int) -> bytes: ...
    def close(self) -> None: ...


class SerialTransport:
    """pyserial wrapper: ``read`` returns whatever is available (or waits up to timeout)."""

    def __init__(self, port: str, baudrate: int = 1_000_000, timeout: float = 0.01):
        import serial  # imported lazily so the package works without pyserial for tests

        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout, write_timeout=0.5)

    def write(self, data: bytes) -> int:
        return self.ser.write(data)

    def read(self, size: int = 4096) -> bytes:
        n = self.ser.in_waiting
        return self.ser.read(min(size, n) if n else 1)

    def close(self) -> None:
        self.ser.close()


class ArmError(Exception):
    pass


class NackError(ArmError):
    def __init__(self, nack: P.Nack):
        try:
            name = P.NackError(nack.error).name
        except ValueError:
            name = f"error {nack.error}"
        try:
            tname = P.MsgType(nack.type).name
        except ValueError:
            tname = f"0x{nack.type:02X}"
        super().__init__(f"board NACKed {tname} seq={nack.seq}: {name}")
        self.nack = nack


class ArmTimeout(ArmError, TimeoutError):
    pass


class SafetyRejected(ArmError):
    def __init__(self, report: SafetyReport):
        super().__init__(str(report))
        self.report = report


class _Pending:
    __slots__ = ("type", "event", "result")

    def __init__(self, msg_type: int):
        self.type = msg_type
        self.event = threading.Event()
        self.result: P.Ack | P.Nack | None = None


@dataclass
class ClientStats:
    tx_frames: int = 0
    rx_frames: int = 0
    states: int = 0
    acks: int = 0
    nacks: int = 0
    unmatched: int = 0
    retries: int = 0
    timeouts: int = 0
    decode_errors: int = 0
    heartbeats: int = 0


class ArmClient:
    def __init__(
        self,
        transport: Transport,
        *,
        safety: SafetyShim | None = None,
        heartbeat_period: float | None = 0.04,
        heartbeat_max_idle: float | None = None,
        ack_timeout: float = 0.1,
        retries: int = 2,
        on_nack: Callable[[P.Nack], None] | None = None,
        start: bool = True,
    ):
        self.transport = transport
        self.safety = safety
        self.heartbeat_period = heartbeat_period
        # If set, the heartbeat stops once the application has issued no command for this
        # long, so a hung application loop still lets the board watchdog trip.
        self.heartbeat_max_idle = heartbeat_max_idle
        self._last_user_cmd = time.monotonic()
        self.ack_timeout = ack_timeout
        self.retries = retries
        self.on_nack = on_nack
        self.stats = ClientStats()
        self.decoder = P.FrameDecoder()

        self._seq = 0
        self._tx_lock = threading.Lock()
        self._pending: dict[int, _Pending] = {}
        self._pend_lock = threading.Lock()
        self._pong_waiters: list[tuple[threading.Event, list]] = []
        self._state_cv = threading.Condition()
        self._state: P.State | None = None
        self._state_time = 0.0
        self._state_count = 0
        self._last_tx = 0.0
        # host's belief that the board has torque on (for seeding the safety shim)
        self._torque_on = False
        self._torque_seq = 0
        self._stop = threading.Event()
        self._hb_enabled = threading.Event()
        if heartbeat_period:
            self._hb_enabled.set()
        self._reader = threading.Thread(target=self._read_loop, name="armctl-reader", daemon=True)
        self._hb = threading.Thread(target=self._hb_loop, name="armctl-heartbeat", daemon=True)
        if start:
            self.start()

    # ------------------------------------------------------------------ constructors
    @classmethod
    def open_serial(cls, port: str, baudrate: int = 1_000_000, **kw) -> "ArmClient":
        return cls(SerialTransport(port, baudrate), **kw)

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        if not self._reader.is_alive():
            self._reader.start()
        if self.heartbeat_period and not self._hb.is_alive():
            self._hb.start()

    def close(self) -> None:
        self._stop.set()
        for t in (self._hb, self._reader):
            if t.is_alive() and t is not threading.current_thread():
                t.join(timeout=1.0)
        try:
            self.transport.close()
        except Exception:  # pragma: no cover
            log.exception("transport close failed")

    def __enter__(self) -> "ArmClient":
        return self

    def __exit__(self, *exc) -> None:
        try:
            self.torque_off()
        except Exception as e:  # never mask the original exception
            log.warning("torque_off on exit failed: %s", e)
        finally:
            self.close()

    def pause_heartbeat(self) -> None:
        """Stop feeding the watchdog (for tests / deliberate host-silence)."""
        self._hb_enabled.clear()

    def resume_heartbeat(self) -> None:
        self._hb_enabled.set()

    # ------------------------------------------------------------------ low level tx
    def _next_seq(self) -> int:
        s = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return s

    def send(self, msg: P.Message, pending: _Pending | None = None) -> int:
        """Encode and write one frame; returns its SEQ. Thread-safe."""
        with self._tx_lock:
            seq = self._next_seq()
            if pending is not None:
                with self._pend_lock:
                    self._pending[seq] = pending
            self.transport.write(P.encode(msg, seq))
            self._last_tx = time.monotonic()
            self.stats.tx_frames += 1
            return seq

    def request(self, msg: P.Message, *, timeout: float | None = None, retries: int | None = None) -> P.Ack:
        """Send ``msg`` and wait for its ACK. Retries with a fresh SEQ on timeout (an ACK
        for any earlier attempt also counts). Raises :class:`NackError` / :class:`ArmTimeout`."""
        timeout = self.ack_timeout if timeout is None else timeout
        retries = self.retries if retries is None else retries
        self._last_user_cmd = time.monotonic()
        pend = _Pending(msg.TYPE)
        seqs = []
        try:
            for attempt in range(retries + 1):
                if attempt:
                    self.stats.retries += 1
                seqs.append(self.send(msg, pend))
                if pend.event.wait(timeout):
                    if isinstance(pend.result, P.Nack):
                        raise NackError(pend.result)
                    return pend.result  # type: ignore[return-value]
            self.stats.timeouts += 1
            raise ArmTimeout(f"no ACK for {P.MsgType(msg.TYPE).name} after {retries + 1} attempts")
        finally:
            with self._pend_lock:
                for s in seqs:
                    if self._pending.get(s) is pend:
                        del self._pending[s]

    def send_nowait(self, msg: P.Message) -> int:
        """Fire-and-forget with ACK tracking: a later NACK goes to ``on_nack`` / stats."""
        self._last_user_cmd = time.monotonic()
        return self.send(msg, _Pending(msg.TYPE))

    # ------------------------------------------------------------------ rx
    def _read_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data = self.transport.read(4096)
            except Exception as e:
                if self._stop.is_set():
                    return
                log.error("transport read failed: %s", e)
                time.sleep(0.05)
                continue
            if not data:
                continue
            for fr in self.decoder.feed(data):
                self._on_frame(fr)

    def _on_frame(self, fr: P.Frame) -> None:
        self.stats.rx_frames += 1
        try:
            msg = fr.decode()
        except P.ProtocolError as e:
            self.stats.decode_errors += 1
            log.debug("dropping frame: %s", e)
            return
        if isinstance(msg, P.State):
            self.stats.states += 1
            if (
                self._torque_on
                and msg.mode in (P.Mode.IDLE, P.Mode.FAULT)
                and ((msg.last_host_seq - self._torque_seq) & 0xFF) < 128
            ):
                self._torque_on = False  # board dropped torque after our enable (fault, ...)
            with self._state_cv:
                self._state = msg
                self._state_time = time.monotonic()
                self._state_count += 1
                self._state_cv.notify_all()
        elif isinstance(msg, (P.Ack, P.Nack)):
            seq = msg.acked_seq if isinstance(msg, P.Ack) else msg.seq
            typ = msg.acked_type if isinstance(msg, P.Ack) else msg.type
            with self._pend_lock:
                pend = self._pending.pop(seq, None)
                if pend is not None and pend.type != typ:
                    self._pending[seq] = pend  # SEQ reused by something else; not ours
                    pend = None
            if isinstance(msg, P.Ack):
                self.stats.acks += 1
            else:
                self.stats.nacks += 1
                if self.on_nack:
                    self.on_nack(msg)
            if pend is None:
                self.stats.unmatched += 1
                return
            pend.result = msg
            pend.event.set()
        elif isinstance(msg, P.Pong):
            # PONG carries no SEQ echo: it answers the oldest outstanding ping.
            with self._pend_lock:
                if self._pong_waiters:
                    ev, box = self._pong_waiters.pop(0)
                    box.append(msg)
                    ev.set()

    def _hb_loop(self) -> None:
        period = self.heartbeat_period or 0.04
        while not self._stop.wait(period / 2):
            if not self._hb_enabled.is_set():
                continue
            if self.heartbeat_max_idle is not None and (
                time.monotonic() - self._last_user_cmd > self.heartbeat_max_idle
            ):
                continue
            if time.monotonic() - self._last_tx >= period:
                try:
                    self.send(P.Ping())
                    self.stats.heartbeats += 1
                except Exception as e:  # pragma: no cover
                    log.warning("heartbeat failed: %s", e)

    # ------------------------------------------------------------------ state access
    @property
    def state(self) -> P.State | None:
        return self._state

    def state_age(self) -> float:
        return time.monotonic() - self._state_time if self._state else float("inf")

    def wait_for_state(
        self, predicate: Callable[[P.State], bool] | None = None, timeout: float = 1.0, fresh: bool = True
    ) -> P.State:
        """Block until a STATE (newer than now if ``fresh``) satisfies ``predicate``."""
        deadline = time.monotonic() + timeout
        with self._state_cv:
            start = self._state_count
            while True:
                st = self._state
                if st is not None and (not fresh or self._state_count > start) and (
                    predicate is None or predicate(st)
                ):
                    return st
                left = deadline - time.monotonic()
                if left <= 0:
                    raise ArmTimeout("no matching STATE before timeout")
                self._state_cv.wait(left)

    def positions(self, fresh: bool = False, timeout: float = 0.5) -> np.ndarray:
        st = self.wait_for_state(timeout=timeout, fresh=fresh) if (fresh or self._state is None) else self._state
        return np.array(st.positions_rad)

    # ------------------------------------------------------------------ commands
    def ping(self, timeout: float | None = None, retries: int | None = None) -> P.Pong:
        timeout = self.ack_timeout if timeout is None else timeout
        retries = self.retries if retries is None else retries
        for attempt in range(retries + 1):
            ev, box = threading.Event(), []
            with self._pend_lock:
                self._pong_waiters.append((ev, box))
            self.send(P.Ping())
            if ev.wait(timeout):
                return box[0]
            with self._pend_lock:
                if (ev, box) in self._pong_waiters:
                    self._pong_waiters.remove((ev, box))
            self.stats.retries += attempt < retries
        self.stats.timeouts += 1
        raise ArmTimeout("no PONG")

    def set_targets(
        self,
        q,
        move_ms: int = 100,
        *,
        torque: bool = True,
        wait: bool = True,
        apply_safety: bool = True,
    ) -> SafetyReport | None:
        """Command 6 joint targets (rad; gripper last) reached in ``move_ms`` (1-5000).

        The safety shim (if configured) runs first; a rejected command raises
        :class:`SafetyRejected` and nothing is sent. Returns the shim's report.
        """
        if not 1 <= int(move_ms) <= 5000:
            raise ValueError("move_ms must be 1..5000")
        q = np.asarray(q, dtype=float)
        if q.shape != (P.N_JOINTS,):
            raise ValueError("need 6 joint targets")
        report = None
        if torque and apply_safety and self.safety is not None:
            if self.safety.last is None or not self._torque_on:
                # starting from limp: velocity-limit relative to where the arm actually is
                self.safety.reset(self.positions())
            report = self.safety.filter(q, move_ms / 1000.0, now=time.monotonic())
            if report.rejected:
                raise SafetyRejected(report)
            if report.modified:
                log.info("safety: %s", report)
            q = report.q_out
        msg = P.SetTargets(
            move_ms=int(move_ms),
            pos_mrad=tuple(P.rad_to_mrad(x) for x in q),
            flags=P.FLAG_TORQUE_ENABLE if torque else 0,
        )
        if wait:
            seq = self.request(msg).acked_seq
        else:
            seq = self.send_nowait(msg)
        if torque and not self._torque_on:
            self._torque_on, self._torque_seq = True, seq
        elif not torque:
            self._torque_on = False
        return report

    def torque_off(self) -> None:
        q = self.positions() if self._state else np.zeros(P.N_JOINTS)
        self.request(P.SetTargets(1, tuple(P.rad_to_mrad(x) for x in q), 0))
        if self.safety:
            self.safety.reset()

    def enable(self, move_ms: int = 500) -> None:
        """Torque on, holding the current measured pose."""
        # Right after boot the firmware NACKs torque-enable (REJECTED_FAULTED, no fault bits
        # set) until every servo has been read once (~6 ms). Found by the firmware cross-test;
        # retry briefly in that case only.
        deadline = time.monotonic() + 0.1
        while True:
            q = self.positions(fresh=True)
            if self.safety:
                self.safety.reset(q)
            try:
                self.set_targets(q, move_ms)
                return
            except NackError as e:
                st = self._state
                booting = (
                    e.nack.error == P.NackError.REJECTED_FAULTED
                    and st is not None
                    and st.fault_flags == 0
                    and st.mode == P.Mode.IDLE
                )
                if not booting or time.monotonic() > deadline:
                    raise

    def estop(self) -> None:
        """Latch the soft e-stop (torque off). Retries harder than normal commands."""
        self._torque_on = False
        self.request(P.EStop(), retries=max(self.retries, 5))

    def clear_faults(self, mask: int | None = None) -> None:
        """Clear latched faults (default: every bit currently set). Leaving FAULT still
        needs a torque-enabled SET_TARGETS afterwards (e.g. :meth:`enable`)."""
        if mask is None:
            st = self._state
            mask = int(st.fault_flags) if st else 0xFFFF
        self.request(P.ClearFault(mask & 0xFFFF))

    def configure(self, joint: int, key: P.ConfigKey | int, value: int) -> None:
        self.request(P.Config(joint, int(key), int(value)))

    def set_stream(self, period_us: int) -> None:
        self.request(P.SetStream(period_us))
