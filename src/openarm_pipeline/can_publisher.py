"""
Mock OpenArm CAN publisher.

Emulates the arm firmware: streams synthetic joint feedback frames onto a
SocketCAN interface (vcan0 / vcan1) at a fixed rate. Because we drive a *real*
SocketCAN bus, the reader (can_reader.py) is exercised through the exact same
code path it would use against real hardware -- only the frame source differs.

Usage:
    python -m openarm_pipeline.can_publisher --channel vcan0 --rate 500
"""

from __future__ import annotations

import argparse
import math
import time

import can

from .can_protocol import JOINTS_PER_ARM, JointState, encode


def make_motion(t: float, joint: int) -> JointState:
    """Deterministic, smooth synthetic trajectory per joint."""
    phase = joint * math.pi / JOINTS_PER_ARM
    freq = 0.2 + 0.05 * joint
    pos = 1.5 * math.sin(2 * math.pi * freq * t + phase)
    vel = 1.5 * 2 * math.pi * freq * math.cos(2 * math.pi * freq * t + phase)
    # Torque loosely proportional to acceleration (inertia-like) plus gravity term.
    acc = -1.5 * (2 * math.pi * freq) ** 2 * math.sin(2 * math.pi * freq * t + phase)
    torque = 0.3 * acc + 0.8 * math.cos(pos)
    return JointState(joint_index=joint, position=pos, velocity=vel, torque=torque)


def run(channel: str, rate_hz: float, fd: bool) -> None:
    bus = can.Bus(channel=channel, interface="socketcan", fd=fd)
    period = 1.0 / rate_hz
    counter = 0
    t0 = time.time()
    print(f"[publisher] streaming {JOINTS_PER_ARM} joints on {channel} "
          f"at {rate_hz:.0f} Hz (fd={fd}). Ctrl-C to stop.")
    try:
        while True:
            now = time.time() - t0
            for j in range(JOINTS_PER_ARM):
                state = make_motion(now, j)
                state.counter = counter & 0xFF
                msg = can.Message(
                    arbitration_id=state.can_id(),
                    data=encode(state),
                    is_extended_id=False,
                    is_fd=fd,
                )
                bus.send(msg)
            counter += 1
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[publisher] stopped.")
    finally:
        bus.shutdown()


def main() -> None:
    ap = argparse.ArgumentParser(description="Mock OpenArm CAN publisher")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--rate", type=float, default=500.0, help="Hz per full joint sweep")
    ap.add_argument("--no-fd", action="store_true", help="use classic CAN 2.0 frames")
    args = ap.parse_args()
    run(args.channel, args.rate, fd=not args.no_fd)


if __name__ == "__main__":
    main()
