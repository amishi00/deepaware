"""
Episode recorder: fuses CAN joint telemetry with multi-camera frames and
writes structured episodes to disk.

SYNCHRONIZATION STRATEGY
------------------------
The joint state stream (CAN, ~500 Hz) is the fastest and most latency-critical
signal, so it drives the master clock. We sample at a fixed recording rate
(default 30 Hz). At each tick we:

  1. Read the latest joint snapshot for both arms (left=can0, right=can1).
  2. For each camera, take the most recent frame whose timestamp <= tick time
     (nearest-past / zero-order hold). Cameras run at different fps (60/30/15),
     so slower cameras simply repeat their last frame -- explicit and lossless
     to reason about, versus interpolating pixels which invents data.
  3. Record every frame's *own* capture timestamp alongside the tick timestamp,
     so downstream consumers can compute per-camera age / staleness and, if they
     prefer, do their own interpolation. We never fabricate a timestamp.

This "align to master clock, keep true source timestamps" approach is the same
pattern used by rosbag/MCAP and by LeRobot-style datasets: one canonical
timeline, plus the raw sensor timestamps for auditability.

STORAGE FORMAT: HDF5 (see docs/DESIGN.md for the full rationale)
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from pathlib import Path

import h5py
import numpy as np

from .cameras import CAMERA_SPECS, CameraRig
from .can_reader import CanReader


class EpisodeRecorder:
    def __init__(
        self,
        data_dir: str | Path,
        left_channel: str = "vcan0",
        right_channel: str = "vcan1",
        rate_hz: float = 30.0,
        fd: bool = True,
        use_cameras: bool = True,
    ):
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.rate_hz = rate_hz
        self.left = CanReader(left_channel, fd=fd)
        self.right = CanReader(right_channel, fd=fd)
        self.rig = CameraRig() if use_cameras else None
        self._recording = False
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._current_id: str | None = None
        self._started = False

    # ---- lifecycle -----------------------------------------------------
    def start_backends(self) -> None:
        """Start CAN readers + cameras (streaming, not yet recording).

        Cameras are started independently of CAN so the dashboard still shows
        camera previews even when no vcan/hardware bus is present.
        """
        if self._started:
            return
        if self.rig:
            self.rig.start()
        # CAN readers may fail if the bus isn't up; surface but don't block cams.
        self.can_error: str | None = None
        for reader in (self.left, self.right):
            try:
                reader.start()
            except Exception as exc:
                self.can_error = str(exc)
        self._started = True

    def stop_backends(self) -> None:
        self.left.stop()
        self.right.stop()
        if self.rig:
            self.rig.stop()
        self._started = False

    @property
    def is_recording(self) -> bool:
        return self._recording

    # ---- recording -----------------------------------------------------
    def start_recording(self) -> str:
        if self._recording:
            raise RuntimeError("already recording")
        self.start_backends()
        self._current_id = time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:6]
        self._stop.clear()
        self._recording = True
        self._thread = threading.Thread(target=self._record_loop, daemon=True)
        self._thread.start()
        return self._current_id

    def stop_recording(self) -> str | None:
        if not self._recording:
            return None
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._recording = False
        eid = self._current_id
        self._current_id = None
        return eid

    def _record_loop(self) -> None:
        assert self._current_id is not None
        path = self.data_dir / f"{self._current_id}.h5"
        period = 1.0 / self.rate_hz
        cam_names = list(CAMERA_SPECS.keys())

        ticks_t = []
        left_rows = []   # each row: 7 joints x 3 (pos,vel,tau)
        right_rows = []
        cam_buffers = {c: [] for c in cam_names}
        cam_ts = {c: [] for c in cam_names}

        t0 = time.time()
        while not self._stop.is_set():
            tick = time.time()
            ticks_t.append(tick - t0)

            left_rows.append(self._joint_matrix(self.left))
            right_rows.append(self._joint_matrix(self.right))

            if self.rig:
                snap = self.rig.snapshot()
                for c in cam_names:
                    fr = snap.get(c)
                    if fr is None:
                        h, w = CAMERA_SPECS[c][1]
                        cam_buffers[c].append(np.zeros((h, w, 3), dtype=np.uint8))
                        cam_ts[c].append(-1.0)
                    else:
                        cam_buffers[c].append(fr.image)
                        cam_ts[c].append(fr.timestamp - t0)

            time.sleep(max(0.0, period - (time.time() - tick)))

        self._write_hdf5(path, ticks_t, left_rows, right_rows, cam_buffers, cam_ts)

    @staticmethod
    def _joint_matrix(reader: CanReader) -> np.ndarray:
        snap = reader.get_snapshot()
        rows = []
        for j in range(7):
            js = snap.joints.get(j)
            if js is None:
                rows.append([0.0, 0.0, 0.0])
            else:
                rows.append([js.position, js.velocity, js.torque])
        return np.array(rows, dtype=np.float32)

    def _write_hdf5(self, path, ticks_t, left_rows, right_rows, cam_buffers, cam_ts):
        n = len(ticks_t)
        with h5py.File(path, "w") as f:
            f.attrs["episode_id"] = self._current_id
            f.attrs["created"] = time.time()
            f.attrs["rate_hz"] = self.rate_hz
            f.attrs["num_ticks"] = n
            f.attrs["schema"] = "openarm-pipeline/v1"

            f.create_dataset("t", data=np.array(ticks_t, dtype=np.float64))

            js = f.create_group("joint_states")
            js.create_dataset("left", data=np.array(left_rows, dtype=np.float32),
                              compression="gzip")
            js.create_dataset("right", data=np.array(right_rows, dtype=np.float32),
                              compression="gzip")
            js.attrs["layout"] = "n_ticks x 7 joints x [pos_rad, vel_rad_s, torque_Nm]"

            if cam_buffers:
                cams = f.create_group("cameras")
                for name, frames in cam_buffers.items():
                    if not frames:
                        continue
                    arr = np.stack(frames)
                    g = cams.create_group(name)
                    g.create_dataset("frames", data=arr, compression="gzip")
                    g.create_dataset("timestamps",
                                     data=np.array(cam_ts[name], dtype=np.float64))
                    g.attrs["fps"] = CAMERA_SPECS[name][0]

        # sidecar metadata for fast listing without opening the h5
        meta = {
            "episode_id": self._current_id,
            "created": time.time(),
            "num_ticks": n,
            "duration_s": ticks_t[-1] if ticks_t else 0.0,
            "rate_hz": self.rate_hz,
            "cameras": list(cam_buffers.keys()),
            "size_bytes": path.stat().st_size,
        }
        (self.data_dir / f"{self._current_id}.json").write_text(json.dumps(meta, indent=2))

    # ---- live view for dashboard --------------------------------------
    def live_state(self) -> dict:
        return {
            "recording": self._recording,
            "episode_id": self._current_id,
            "left": self.left.get_snapshot().as_dict(),
            "right": self.right.get_snapshot().as_dict(),
        }
