"""URDF of the arm's kinematic chain, generated from spec/kinematics.yaml.

    python -m armctl.urdf > vla_arm.urdf

Links: base_link -> shoulder_link -> upper_arm -> forearm -> wrist -> gripper -> tcp (fixed).
Joint origins, axes, limits and velocity limits are copied from the spec; link masses from
``masses_kg`` (inertia as a small solid sphere; good enough for viewers and planners, not for
dynamics). No visual/collision geometry: armctl's collision model lives in
``armctl/data/collision_model.yaml``. The gripper's own servo is not a URDF joint (its jaws are
a linkage); it is commanded separately.

Used by tests/test_crosscheck.py to compare :func:`armctl.kinematics.fk` / ``jacobian`` with
Pinocchio and ikpy.
"""

from __future__ import annotations

import sys

from .kinematics import ArmModel, default_model

LINKS = ("base_link", "shoulder_link", "upper_arm", "forearm", "wrist", "gripper")


def _inertial(mass: float | None) -> str:
    if not mass:
        return ""
    i = 0.4 * mass * 0.02**2  # solid sphere, r = 2 cm
    return (
        f'    <inertial><origin xyz="0 0 0"/><mass value="{mass:g}"/>'
        f'<inertia ixx="{i:.3e}" iyy="{i:.3e}" izz="{i:.3e}" ixy="0" ixz="0" iyz="0"/></inertial>\n'
    )


def to_urdf(model: ArmModel | None = None, name: str = "vla_arm") -> str:
    model = model or default_model()
    masses = (model.raw or {}).get("masses_kg", {}) if hasattr(model, "raw") else {}
    out = [f'<?xml version="1.0"?>\n<robot name="{name}">\n']
    for ln in LINKS:
        out.append(f'  <link name="{ln}">\n{_inertial(masses.get(ln))}  </link>\n')
    out.append('  <link name="tcp"/>\n')
    for k, j in enumerate(model.joints):
        o = " ".join(f"{v:.9g}" for v in j.origin)
        a = " ".join(f"{v:g}" for v in j.axis)
        out.append(
            f'  <joint name="{j.name}" type="revolute">\n'
            f'    <parent link="{LINKS[k]}"/><child link="{LINKS[k + 1]}"/>\n'
            f'    <origin xyz="{o}" rpy="0 0 0"/><axis xyz="{a}"/>\n'
            f'    <limit lower="{j.limits[0]:g}" upper="{j.limits[1]:g}" velocity="{j.vmax:g}" effort="2.9"/>\n'
            "  </joint>\n"
        )
    t = " ".join(f"{v:.9g}" for v in model.tcp)
    out.append(
        '  <joint name="tcp_fixed" type="fixed">\n'
        f'    <parent link="{LINKS[-1]}"/><child link="tcp"/><origin xyz="{t}" rpy="0 0 0"/>\n'
        "  </joint>\n</robot>\n"
    )
    return "".join(out)


if __name__ == "__main__":  # pragma: no cover
    sys.stdout.write(to_urdf())
