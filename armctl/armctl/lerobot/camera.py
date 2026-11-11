"""A synthetic camera implementing LeRobot's ``Camera`` interface (no hardware, no OpenCV)."""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
from lerobot.cameras.camera import Camera
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .config import ArmctlFakeCameraConfig


class ArmctlFakeCamera(Camera):
    """Frames are a gradient that scrolls with time plus a per-frame counter stripe, so
    consecutive frames differ and frame drops are visible in a recording."""

    def __init__(self, config: ArmctlFakeCameraConfig):
        super().__init__(config)
        self.config = config
        self._connected = False
        self._count = 0
        self._last_consumed = -1
        self._lock = threading.Lock()
        h, w = int(config.height), int(config.width)
        rng = np.random.default_rng(config.seed)
        self._base = rng.integers(0, 64, size=(h, w, 3), dtype=np.uint8)
        self._yy, self._xx = np.mgrid[0:h, 0:w]

    @property
    def is_connected(self) -> bool:
        return self._connected

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        return []

    def connect(self, warmup: bool = True) -> None:
        if self._connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
        self._connected = True
        self._t0 = time.monotonic()
        if warmup:
            self.read()

    def _frame(self) -> np.ndarray:
        if not self._connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        with self._lock:
            self._count += 1
            k = self._count
        shift = int((time.monotonic() - self._t0) * 60)
        img = self._base.copy()
        img[..., 0] += ((self._xx + shift) % 192).astype(np.uint8)
        img[..., 1] += ((self._yy * 2) % 192).astype(np.uint8)
        img[: max(1, img.shape[0] // 16), :, 2] = k % 256
        return img

    def read(self) -> np.ndarray:
        return self._frame()

    def async_read(self, timeout_ms: float = 200) -> np.ndarray:
        return self._frame()

    def read_latest(self, max_age_ms: int = 500) -> np.ndarray:
        return self._frame()

    def disconnect(self) -> None:
        if not self._connected:
            raise DeviceNotConnectedError(f"{self} is not connected")
        self._connected = False

    def __str__(self) -> str:
        return f"ArmctlFakeCamera({self.config.width}x{self.config.height})"
