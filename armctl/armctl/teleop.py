"""Leader -> follower teleoperation loop.

The leader arm is a second board (same protocol) with torque off; its STATE stream gives
the operator's joint angles. Each tick: read the latest leader state, map it through the
leader->follower calibration, pass it through the safety shim, and send it to the follower
as a fire-and-forget SET_TARGETS (ACKs/NACKs are tracked asynchronously).

``TeleopLoop.step()`` is one deterministic iteration, so tests can drive it directly.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

import numpy as np

from . import protocol as P
from .calibration import LeaderMap
from .client import ArmClient, SafetyRejected
from .safety import SafetyShim

log = logging.getLogger(__name__)


@dataclass
class TeleopStats:
    steps: int = 0
    sent: int = 0
    stale_leader: int = 0
    safety_modified: int = 0
    safety_rejected: int = 0
    last_reasons: list[str] = field(default_factory=list)
    overruns: int = 0


class FollowerFaulted(RuntimeError):
    pass


class TeleopLoop:
    def __init__(
        self,
        leader: ArmClient,
        follower: ArmClient,
        mapping: LeaderMap | None = None,
        safety: SafetyShim | None = None,
        *,
        rate_hz: float = 100.0,
        move_ms: int | None = None,
        max_leader_age: float = 0.1,
    ):
        self.leader = leader
        self.follower = follower
        self.mapping = mapping or LeaderMap()
        self.safety = safety if safety is not None else SafetyShim()
        self.rate_hz = rate_hz
        # move duration a bit longer than the period so consecutive moves blend smoothly
        self.move_ms = move_ms or max(1, int(round(2000.0 / rate_hz)))
        self.max_leader_age = max_leader_age
        self.stats = TeleopStats()
        self._started = False

    def start(self) -> None:
        """Enable follower torque at its current pose and seed the safety shim."""
        st = self.follower.wait_for_state(timeout=1.0)
        if st.faults & P.LATCHED_FAULTS:
            raise FollowerFaulted(f"follower has latched faults {P.Fault(st.fault_flags)!r}")
        q = np.array(st.positions_rad)
        self.safety.reset(q)
        self.follower.set_targets(q, 200, apply_safety=False)
        self._started = True

    def step(self) -> np.ndarray | None:
        """One iteration. Returns the command sent, or None if nothing was sent."""
        if not self._started:
            self.start()
        self.stats.steps += 1
        fst = self.follower.state
        if fst is not None and fst.mode == P.Mode.FAULT:
            raise FollowerFaulted(f"follower in FAULT: {P.Fault(fst.fault_flags)!r}")
        lst = self.leader.state
        if lst is None or self.leader.state_age() > self.max_leader_age:
            self.stats.stale_leader += 1
            return None  # follower keeps its last target; heartbeat keeps it out of HOLD
        q_cmd = self.mapping(lst.positions_rad)
        rep = self.safety.filter(q_cmd, self.move_ms / 1000.0)
        if rep.rejected:
            self.stats.safety_rejected += 1
            self.stats.last_reasons = rep.reasons
            return None
        if rep.modified:
            self.stats.safety_modified += 1
            self.stats.last_reasons = rep.reasons
        # the shim already ran here; don't run the client's copy again
        self.follower.set_targets(rep.q_out, self.move_ms, wait=False, apply_safety=False)
        self.stats.sent += 1
        return rep.q_out

    def run(self, duration: float | None = None, stop=None) -> TeleopStats:
        """Run at ``rate_hz`` until ``duration`` elapses or ``stop`` (an Event) is set."""
        period = 1.0 / self.rate_hz
        t0 = time.monotonic()
        next_t = t0
        while (duration is None or time.monotonic() - t0 < duration) and not (stop and stop.is_set()):
            try:
                self.step()
            except SafetyRejected:  # pragma: no cover - step() handles rejections itself
                pass
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                self.stats.overruns += 1
                next_t = time.monotonic()
        return self.stats
