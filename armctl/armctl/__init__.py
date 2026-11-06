"""armctl: host driver for the VLA arm controller board (protocol v1)."""

from . import protocol
from .client import ArmClient, ArmTimeout, NackError, SafetyRejected, SerialTransport
from .kinematics import fk, ik, jacobian, load_model, solve_ik, solve_ik_xyz
from .safety import SafetyConfig, SafetyShim

__all__ = [
    "protocol", "ArmClient", "ArmTimeout", "NackError", "SafetyRejected", "SerialTransport",
    "fk", "ik", "jacobian", "load_model", "solve_ik", "solve_ik_xyz", "SafetyConfig", "SafetyShim",
]
__version__ = "0.1.0"
