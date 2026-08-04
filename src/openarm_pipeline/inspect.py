"""
Inspect a recorded episode: print schema, shapes, and per-camera timing so you
can verify sync alignment. Doubles as a usage example for consumers.

    python -m openarm_pipeline.inspect data/episodes/<id>.h5
"""

from __future__ import annotations

import sys

import h5py
import numpy as np


def inspect(path: str) -> None:
    with h5py.File(path, "r") as f:
        print(f"episode      : {f.attrs.get('episode_id')}")
        print(f"schema       : {f.attrs.get('schema')}")
        print(f"rate_hz      : {f.attrs.get('rate_hz')}")
        n = int(f.attrs.get("num_ticks", 0))
        print(f"ticks        : {n}")
        t = f["t"][:]
        if len(t) > 1:
            dt = np.diff(t)
            print(f"tick dt      : mean {dt.mean()*1000:.2f} ms  "
                  f"jitter±{dt.std()*1000:.2f} ms")

        for side in ("left", "right"):
            arr = f[f"joint_states/{side}"][:]
            print(f"{side:5} joints : shape {arr.shape}  "
                  f"pos[0] {np.round(arr[0,:,0],3)}")

        if "cameras" in f:
            print("cameras:")
            for cam in f["cameras"]:
                frames = f[f"cameras/{cam}/frames"]
                ts = f[f"cameras/{cam}/timestamps"][:]
                valid = ts[ts >= 0]
                if len(valid) > 1:
                    fps = 1.0 / np.diff(valid).mean()
                else:
                    fps = float("nan")
                print(f"  {cam:12} frames {frames.shape}  "
                      f"effective ~{fps:.1f} fps")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: python -m openarm_pipeline.inspect <episode.h5>")
        sys.exit(1)
    inspect(sys.argv[1])
