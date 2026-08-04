"""
OpenArm CAN FD protocol codec (simplified / mock).

The real OpenArm uses DM-series motors (Damiao) whose feedback frames pack
position, velocity, and torque into a compact payload. The exact bit layout is
firmware-specific; here we implement a clean, self-consistent codec that mimics
the *shape* of that data so the rest of the pipeline (reader, sync, storage,
dashboard) is exercised realistically.

Frame layout per joint (CAN FD, 8 data bytes used):
    byte 0-1 : position  (uint16, maps to [-12.5, 12.5] rad)
    byte 2-3 : velocity  (uint16, maps to [-45.0, 45.0] rad/s)
    byte 4-5 : torque    (uint16, maps to [-18.0, 18.0] Nm)
    byte 6   : motor id / error flags
    byte 7   : rolling counter (for drop detection)

CAN IDs: base 0x100 + joint_index, so joint 0 -> 0x100, joint 1 -> 0x101, ...
Each arm (can0, can1) carries 7 joints -> IDs 0x100..0x106.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

BASE_ID = 0x100
JOINTS_PER_ARM = 7

# Physical ranges used for uint16 <-> float encoding.
P_MIN, P_MAX = -12.5, 12.5       # rad
V_MIN, V_MAX = -45.0, 45.0       # rad/s
T_MIN, T_MAX = -18.0, 18.0       # Nm

_U16 = 0xFFFF


def _to_u16(value: float, lo: float, hi: float) -> int:
    value = max(lo, min(hi, value))
    return int((value - lo) / (hi - lo) * _U16)


def _from_u16(raw: int, lo: float, hi: float) -> float:
    return raw / _U16 * (hi - lo) + lo


@dataclass
class JointState:
    joint_index: int
    position: float   # rad
    velocity: float   # rad/s
    torque: float     # Nm
    counter: int = 0
    error: int = 0

    def can_id(self) -> int:
        return BASE_ID + self.joint_index


def encode(state: JointState) -> bytes:
    """Pack a JointState into an 8-byte CAN FD payload."""
    p = _to_u16(state.position, P_MIN, P_MAX)
    v = _to_u16(state.velocity, V_MIN, V_MAX)
    t = _to_u16(state.torque, T_MIN, T_MAX)
    return struct.pack(
        ">HHHBB",
        p, v, t,
        state.joint_index & 0xFF,
        state.counter & 0xFF,
    )


def decode(can_id: int, data: bytes) -> JointState:
    """Decode an 8-byte payload back into a JointState."""
    if len(data) < 8:
        raise ValueError(f"expected >=8 bytes, got {len(data)}")
    p, v, t, jid, counter = struct.unpack(">HHHBB", data[:8])
    return JointState(
        joint_index=can_id - BASE_ID,
        position=_from_u16(p, P_MIN, P_MAX),
        velocity=_from_u16(v, V_MIN, V_MAX),
        torque=_from_u16(t, T_MIN, T_MAX),
        counter=counter,
        error=(jid >> 7) & 0x1,
    )
