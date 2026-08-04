"""
Multi-camera simulation for OpenArm data collection.

Real rig: 4 cameras -- wrist_left, wrist_right (Arducam), ceiling (Arducam),
and a ZED stereo head. They run at *different* frame rates and are not hardware
triggered, so frames arrive asynchronously. This module simulates each camera
as an independent thread producing timestamped frames into a ring buffer.

Frame payloads here are tiny synthetic images (numpy arrays) so the storage and
sync logic is exercised without real capture hardware. Swapping in real capture
means replacing Camera.grab() with cv2.VideoCapture / pyzed reads -- the sync
layer is unchanged.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    camera: str
    timestamp: float          # capture time, seconds (time.time())
    seq: int
    image: np.ndarray         # HxWxC uint8 (synthetic)


# name -> (fps, resolution, channels)
CAMERA_SPECS = {
    "wrist_left":  (60, (48, 64), 3),
    "wrist_right": (60, (48, 64), 3),
    "ceiling":     (30, (48, 64), 3),
    "zed_stereo":  (15, (48, 128), 3),  # side-by-side stereo -> wider
}


class Camera:
    def __init__(self, name: str):
        self.name = name
        self.fps, self.res, self.ch = CAMERA_SPECS[name]
        self._seq = 0
        self._latest: Frame | None = None
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def grab(self) -> Frame:
        """Produce one synthetic frame. Replace with real capture on hardware."""
        h, w = self.res
        # deterministic-ish moving gradient, keeps files small but non-trivial.
        # Compute in int32 then cast to avoid uint8 overflow.
        base = (self._seq * 7) % 256
        img = np.full((h, w, self.ch), base, dtype=np.int32)
        img[:, :, 0] = (img[:, :, 0] + np.arange(w)) % 256
        img = img.astype(np.uint8)
        frame = Frame(camera=self.name, timestamp=time.time(), seq=self._seq, image=img)
        self._seq += 1
        return frame

    def _loop(self) -> None:
        period = 1.0 / self.fps
        while not self._stop.is_set():
            f = self.grab()
            with self._lock:
                self._latest = f
            time.sleep(period)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def latest(self) -> Frame | None:
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)


class CameraRig:
    """Manages all 4 cameras and produces synchronized bundles."""

    def __init__(self):
        self.cameras = {name: Camera(name) for name in CAMERA_SPECS}

    def start(self) -> None:
        for cam in self.cameras.values():
            cam.start()

    def stop(self) -> None:
        for cam in self.cameras.values():
            cam.stop()

    def snapshot(self) -> dict[str, Frame | None]:
        """Latest frame from every camera (nearest-neighbor in time)."""
        return {name: cam.latest() for name, cam in self.cameras.items()}
