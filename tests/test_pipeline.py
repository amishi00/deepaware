"""
Minimal test suite. Run: PYTHONPATH=src python -m pytest tests/ -q
(or without pytest: PYTHONPATH=src python tests/test_pipeline.py)
"""
import math
import tempfile
from pathlib import Path

import numpy as np

from openarm_pipeline.can_protocol import JointState, encode, decode, BASE_ID


def test_codec_roundtrip():
    for j in range(7):
        s = JointState(j, 1.234 * math.sin(j), -2.5 * j, 0.7 * j, counter=j)
        d = decode(s.can_id(), encode(s))
        assert d.joint_index == j
        assert abs(d.position - s.position) < 1e-3
        assert abs(d.velocity - s.velocity) < 1e-2
        assert abs(d.torque - s.torque) < 1e-2


def test_can_ids():
    assert JointState(0, 0, 0, 0).can_id() == BASE_ID
    assert JointState(6, 0, 0, 0).can_id() == BASE_ID + 6


def test_codec_clamps_out_of_range():
    # values beyond physical range clamp rather than wrap
    s = JointState(0, 999.0, -999.0, 999.0)
    d = decode(s.can_id(), encode(s))
    assert d.position <= 12.5 + 1e-3
    assert d.velocity >= -45.0 - 1e-3


def test_hdf5_roundtrip():
    import h5py
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "ep.h5"
        n = 10
        left = np.random.randn(n, 7, 3).astype("float32")
        with h5py.File(p, "w") as f:
            f.attrs["episode_id"] = "t"
            f.create_dataset("t", data=np.linspace(0, 0.3, n))
            f.create_dataset("joint_states/left", data=left, compression="gzip")
        with h5py.File(p, "r") as f:
            assert f["joint_states/left"].shape == (n, 7, 3)
            assert np.allclose(f["joint_states/left"][:], left)


if __name__ == "__main__":
    test_codec_roundtrip()
    test_can_ids()
    test_codec_clamps_out_of_range()
    test_hdf5_roundtrip()
    print("all tests passed")
