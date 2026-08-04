"""
CAN FD reader for OpenArm joint telemetry.

Listens on a SocketCAN interface, decodes frames into JointState objects, and
maintains a live snapshot of the most recent value per joint. Also tracks a
rolling counter per joint to detect dropped frames -- basic real-time health
awareness you'd want on real hardware.

Works identically against real can0/can1 or the mock vcan0/vcan1.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

import can

from .can_protocol import JOINTS_PER_ARM, JointState, decode


@dataclass
class ArmSnapshot:
    """Latest joint states for one arm plus health counters."""
    channel: str
    joints: dict[int, JointState] = field(default_factory=dict)
    last_rx_monotonic: float = 0.0
    frames_received: int = 0
    frames_dropped: int = 0
    _last_counter: dict[int, int] = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {
            "channel": self.channel,
            "timestamp": time.time(),
            "frames_received": self.frames_received,
            "frames_dropped": self.frames_dropped,
            "joints": [
                {
                    "index": js.joint_index,
                    "position": round(js.position, 5),
                    "velocity": round(js.velocity, 5),
                    "torque": round(js.torque, 5),
                }
                for js in sorted(self.joints.values(), key=lambda x: x.joint_index)
            ],
        }


class CanReader:
    """Background thread that keeps an up-to-date ArmSnapshot for one channel."""

    def __init__(self, channel: str, fd: bool = True):
        self.channel = channel
        self.fd = fd
        self.snapshot = ArmSnapshot(channel=channel)
        self._bus: can.BusABC | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

    def start(self) -> None:
        self._bus = can.Bus(channel=self.channel, interface="socketcan", fd=self.fd)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        assert self._bus is not None
        while not self._stop.is_set():
            msg = self._bus.recv(timeout=0.5)
            if msg is None:
                continue
            try:
                state = decode(msg.arbitration_id, bytes(msg.data))
            except ValueError:
                continue
            with self._lock:
                snap = self.snapshot
                snap.frames_received += 1
                snap.last_rx_monotonic = time.monotonic()
                # Drop detection via rolling counter.
                prev = snap._last_counter.get(state.joint_index)
                if prev is not None:
                    expected = (prev + 1) & 0xFF
                    if state.counter != expected:
                        gap = (state.counter - expected) & 0xFF
                        snap.frames_dropped += gap
                snap._last_counter[state.joint_index] = state.counter
                snap.joints[state.joint_index] = state

    def get_snapshot(self) -> ArmSnapshot:
        with self._lock:
            # shallow copy sufficient for read-only export
            return self.snapshot

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._bus:
            self._bus.shutdown()


def demo(channel: str = "vcan0", seconds: float = 3.0) -> None:
    reader = CanReader(channel)
    reader.start()
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            time.sleep(0.5)
            snap = reader.get_snapshot()
            if snap.joints:
                j0 = snap.joints.get(0)
                print(f"[{channel}] rx={snap.frames_received} "
                      f"drop={snap.frames_dropped} "
                      f"j0 pos={j0.position:+.3f} vel={j0.velocity:+.3f} "
                      f"tau={j0.torque:+.3f}")
    finally:
        reader.stop()


if __name__ == "__main__":
    import sys
    ch = sys.argv[1] if len(sys.argv) > 1 else "vcan0"
    demo(ch)
