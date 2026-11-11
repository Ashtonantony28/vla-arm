"""Open an :class:`~armctl.client.ArmClient` from a LeRobot config ``port`` string."""

from __future__ import annotations

import threading

import numpy as np

from ..client import ArmClient
from ..mockboard import MockBoard, MockTransport

# a comfortable, collision-free pose (TCP ~0.29 m out, 0.30 m up); mock boards start here
MOCK_HOME = (0.0, -1.2, 1.4, 0.6, 0.0, 0.5)

_mocks: dict[str, MockBoard] = {}
_lock = threading.Lock()


def mock_board(name: str, initial_q=None) -> MockBoard:
    """The shared mock board ``name`` (port ``"mock:<name>"``), created on first use."""
    with _lock:
        b = _mocks.get(name)
        if b is None:
            b = _mocks[name] = MockBoard(initial_q=np.asarray(initial_q if initial_q is not None else MOCK_HOME))
        return b


def forget_mock_board(name: str | None = None) -> None:
    """Drop one (or every) shared mock board, so the next open starts fresh."""
    with _lock:
        if name is None:
            _mocks.clear()
        else:
            _mocks.pop(name, None)


def open_client(port: str, baudrate: int = 1_000_000, initial_q=None, **kw) -> ArmClient:
    if port == "mock":
        board = MockBoard(initial_q=np.asarray(initial_q if initial_q is not None else MOCK_HOME))
        return ArmClient(MockTransport(board), **kw)
    if port.startswith("mock:"):
        return ArmClient(MockTransport(mock_board(port[5:], initial_q)), **kw)
    return ArmClient.open_serial(port, baudrate, **kw)
