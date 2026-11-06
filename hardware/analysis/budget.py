"""Torque, counterbalance, current and servo-bus timing budget.

All functions are pure (inputs -> numbers) so they can be unit-tested against hand calculations.
`python -m analysis.report` assembles them into analysis/budget_report.md.

Conventions: SI units (m, kg, N, s) unless a name says otherwise. Pitch angles follow
spec/kinematics.yaml: rotation about +y, positive = DOWN. Torques are magnitudes the servo must
supply to hold the pose statically.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.81


# ============================================================================ arm model
@dataclass
class ArmModel:
    """Planar (vertical-plane) model of the pitch chain.

    link_len: shoulder->elbow, elbow->wrist pitch, wrist pitch->TCP (m)
    masses:   upper_arm, forearm, wrist, gripper (kg)
    coms:     (x, z) of each mass in its own link frame (m); gripper COM given in the wrist frame
    """
    link_len: tuple[float, float, float]
    masses: dict[str, float]
    coms: dict[str, tuple[float, float]]
    payload: float = 0.0

    def points(self, q2, q3, q4):
        """World-plane (x, z) of each mass and of the TCP, relative to the shoulder axis."""
        L1, L2, Lt = self.link_len
        out = {}
        # joint positions
        e = np.array([L1 * math.cos(q2), -L1 * math.sin(q2)])
        w = e + np.array([L2 * math.cos(q2 + q3), -L2 * math.sin(q2 + q3)])
        origin = {"upper_arm": np.zeros(2), "forearm": e, "wrist": w, "gripper": w}
        phi = {"upper_arm": q2, "forearm": q2 + q3, "wrist": q2 + q3 + q4, "gripper": q2 + q3 + q4}
        for k, (cx, cz) in self.coms.items():
            f = phi[k]
            out[k] = origin[k] + np.array([cx * math.cos(f) + cz * math.sin(f), -cx * math.sin(f) + cz * math.cos(f)])
        f = phi["wrist"]
        out["payload"] = w + np.array([Lt * math.cos(f), -Lt * math.sin(f)])
        out["_elbow"], out["_wrist"] = e, w
        return out


PITCH_JOINTS = ("shoulder_pitch", "elbow_pitch", "wrist_pitch")
DOWNSTREAM = {"shoulder_pitch": ("upper_arm", "forearm", "wrist", "gripper", "payload"),
              "elbow_pitch": ("forearm", "wrist", "gripper", "payload"),
              "wrist_pitch": ("wrist", "gripper", "payload")}


def gravity_torques(m: ArmModel, q2=0.0, q3=0.0, q4=0.0) -> dict[str, float]:
    """Signed gravity torque about each pitch axis (N*m, + = pulls the arm DOWN)."""
    pts = m.points(q2, q3, q4)
    axis = {"shoulder_pitch": np.zeros(2), "elbow_pitch": pts["_elbow"], "wrist_pitch": pts["_wrist"]}
    mass = dict(m.masses, payload=m.payload)
    return {j: sum(mass[k] * G * (pts[k][0] - axis[j][0]) for k in DOWNSTREAM[j]) for j in PITCH_JOINTS}


# ============================================================================ counterbalance
def spring_torque(K: float, q2) -> float:
    """Zero-free-length spring between A (height a above the shoulder axis, on the yaw-fixed bracket)
    and B (distance b along the upper arm): torque = k*a*b*sin(angle AOB) = K*cos(q2), lifting."""
    return K * np.cos(q2)


def minimax_balance(G_tau: np.ndarray, cos_q2: np.ndarray) -> tuple[float, float]:
    """K minimising max |G_tau - K cos_q2| over the sample set (exact for a 1-D linear minimax:
    scan the breakpoints). Returns (K, worst residual)."""
    # residual r_i(K) = G_i - K c_i is linear in K; the max of |.| is convex -> ternary search
    lo, hi = 0.0, float(np.max(np.abs(G_tau) / np.maximum(np.abs(cos_q2), 1e-3))) + 1.0
    f = lambda K: float(np.max(np.abs(G_tau - K * cos_q2)))
    for _ in range(200):
        a, b = lo + (hi - lo) / 3, hi - (hi - lo) / 3
        if f(a) <= f(b):
            hi = b
        else:
            lo = a
    K = 0.5 * (lo + hi)
    return K, f(K)


@dataclass
class SpringDesign:
    K: float          # required k*a*b (N*m)
    a: float          # m
    b: float          # m
    k: float = field(init=False)           # N/m
    ab_min: float = field(init=False)      # m
    ab_max: float = field(init=False)

    def bounds(self, q2_range):
        q = np.linspace(*q2_range, 721)
        ang = math.pi / 2 + q                 # angle AOB
        d = np.sqrt(self.a ** 2 + self.b ** 2 - 2 * self.a * self.b * np.cos(ang))
        self.k = self.K / (self.a * self.b)
        self.ab_min, self.ab_max = float(d.min()), float(d.max())
        # true minimum also includes the vertical-up pose if inside the range
        if q2_range[0] <= -math.pi / 2 <= q2_range[1]:
            self.ab_min = abs(self.a - self.b)
        return self

    def realisation(self, preload_fraction=0.8) -> dict:
        """Extension spring + cable over a pulley at A: F = Fi + k*x with the cable length set so that
        x = |AB| - Fi/k, which makes F = k*|AB| exactly (equivalent zero free length) as long as the
        spring never goes slack, i.e. Fi/k <= |AB|_min."""
        c = preload_fraction * self.ab_min
        Fi = self.k * c
        return dict(k_N_per_mm=self.k / 1000, Fi_N=Fi, ab_min_mm=self.ab_min * 1000, ab_max_mm=self.ab_max * 1000,
                    extension_max_mm=(self.ab_max - c) * 1000, extension_min_mm=(self.ab_min - c) * 1000,
                    F_max_N=self.k * self.ab_max)


# ============================================================================ servo
@dataclass(frozen=True)
class ServoRating:
    stall_torque: float      # N*m
    stall_current: float     # A
    max_cont_current: float  # A
    derate: float = 0.40     # continuous torque = derate * stall
    idle_current: float = 0.05  # A per servo, torque on, holding nothing (ASSUMED, measure)

    @property
    def cont_torque(self) -> float:
        return self.derate * self.stall_torque

    def current_for(self, tau: float) -> float:
        """Linear torque-constant model I = idle + (I_stall - idle) * tau / tau_stall."""
        return self.idle_current + (self.stall_current - self.idle_current) * min(abs(tau) / self.stall_torque, 1.0)


# ============================================================================ bus timing
@dataclass(frozen=True)
class BusTiming:
    baud: int = 1_000_000
    bits_per_byte: int = 10          # 8N1
    n_servos: int = 6
    write_bytes_per_servo: int = 2   # goal position only
    read_bytes: int = 15             # 0x38..0x46: pos, speed, load, voltage, temp, ..., current
    servo_latency_us: float = 100.0  # ASSUMED worst-case servo processing + return delay; measure on HW
    turnaround_us: float = 2.0       # DE switch / buffer enable per direction change
    cpu_overhead_us: float = 30.0    # ISR/DMA scheduling slack per cycle (assumed)
    cycle_us: float = 1000.0

    @property
    def byte_us(self) -> float:
        return self.bits_per_byte * 1e6 / self.baud

    def sync_write_len(self) -> int:
        # FF FF FE LEN 83 ADDR DLEN [ID d0..dn-1]*N CHK
        return 8 + self.n_servos * (1 + self.write_bytes_per_servo)

    def read_req_len(self) -> int:
        return 8          # FF FF ID LEN 02 ADDR NBYTES CHK

    def status_len(self) -> int:
        return 6 + self.read_bytes   # FF FF ID LEN ERR data CHK

    def budget(self) -> dict:
        items = {
            f"SYNC_WRITE ({self.sync_write_len()} B)": self.sync_write_len() * self.byte_us,
            f"READ request ({self.read_req_len()} B)": self.read_req_len() * self.byte_us,
            "turnaround tx->rx": self.turnaround_us,
            "servo latency (assumed)": self.servo_latency_us,
            f"status reply ({self.status_len()} B)": self.status_len() * self.byte_us,
            "turnaround rx->tx": self.turnaround_us,
            "CPU/DMA slack (assumed)": self.cpu_overhead_us,
        }
        total = sum(items.values())
        return dict(items=items, total_us=total, margin_us=self.cycle_us - total,
                    utilisation=total / self.cycle_us)
