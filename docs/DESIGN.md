# Design Notes

This document explains the architecture, the synchronization strategy, and the
key trade-offs. It's the "why" companion to the README's "how."

## Architecture

```
  vcan0 (left) ──┐                          ┌── GET /api/live        ┐
                 ├─► CanReader ──┐           ├── GET /api/live/frame  │
  vcan1 (right) ─┘   (thread)    │           ├── POST /record/start  ├─ dashboard
                                 ├► Recorder ├── POST /record/stop    │  (poll 5 Hz)
  4 cameras ──────► CameraRig ───┘  (thread) ├── GET /api/episodes    │
   (threads, async fps)                      └── GET /.../download    ┘
                                       │
                                       ▼
                              data/episodes/*.h5  (+ *.json sidecar)
```

Every source runs in its own thread and maintains a *latest-value* snapshot.
The recorder samples those snapshots on a fixed master clock. The API is a thin
read/-control layer over the recorder. The dashboard polls the API.

## Synchronization strategy (Task 3)

**The joint stream is the master clock.** Joint telemetry is the fastest
(~500 Hz) and most latency-sensitive signal, so recording ticks are driven at a
fixed rate (default 30 Hz) and everything else is aligned to those ticks.

**Cameras use zero-order hold (nearest-past frame).** The four cameras run at
different, unsynchronized rates (60/60/30/15 fps). At each tick we take, per
camera, the most recent frame available. Slower cameras repeat their last frame
rather than blocking or interpolating. Rationale:

- **No fabricated pixels.** Interpolating between frames invents image data that
  never existed; for robot-learning datasets that's worse than an honest repeat.
- **Every frame keeps its own capture timestamp.** We store each camera's true
  capture time alongside the tick time, so a downstream consumer can compute
  frame *age* at each tick and do its own resampling if it wants. We never
  overwrite a real timestamp with the tick time.

This "one canonical timeline + preserved source timestamps" is the same pattern
used by rosbag/MCAP and LeRobot-style datasets.

**Dealing with different frame rates.** Because we sample at the tick rate, the
stored frame count per camera equals the tick count — the 15 fps ZED simply has
more repeated frames than the 60 fps wrist cams. The timestamp arrays make the
true underlying rate recoverable (the inspector prints "effective fps" from
timestamp deltas). An alternative would be to store each camera at its native
rate in separate time bases and align lazily at read time; that saves space but
pushes sync complexity onto every consumer. For a teleoperation dataset where
downstream training wants aligned (state, image) tuples, aligning once at write
time is the better default.

## Storage format: HDF5 (Task 4)

Chosen over the alternatives:

| Format | Why not (here) |
|--------|----------------|
| **HDF5** ✅ | Single self-describing file; named groups/datasets map cleanly to arms/joints/cameras; built-in gzip; random slice access without loading the whole episode; universal reader support (h5py, MATLAB, Julia). |
| MCAP | Excellent for *streaming* ROS-style logs and playback, but the pipeline here is episodic and array-shaped; HDF5's ndarray slicing fits robot-learning consumers (PyTorch/JAX) more directly. |
| zarr | Great for cloud/chunked parallel access to huge arrays; overkill for single-episode files on local disk and adds a directory-of-chunks rather than one portable file. |
| Parquet | Columnar tabular data is a poor fit for per-tick image tensors. |
| custom | No reason to reinvent chunking, compression, and typing that HDF5 already provides well. |

**Layout**

```
/                          attrs: episode_id, rate_hz, num_ticks, schema
├── t                      (N,)          tick timestamps, seconds
├── joint_states/
│   ├── left               (N, 7, 3)     [pos_rad, vel_rad_s, torque_Nm]
│   └── right              (N, 7, 3)
└── cameras/
    ├── wrist_left/frames      (N, H, W, 3) uint8, gzip
    ├── wrist_left/timestamps  (N,)         true capture times
    └── ... (wrist_right, ceiling, zed_stereo)
```

A small JSON sidecar per episode holds listing metadata (duration, size, camera
list) so `GET /api/episodes` never has to open the HDF5 files.

## Real-time awareness

- **Drop detection.** Each frame carries a rolling counter; the reader compares
  against the expected next value per joint and counts gaps. Surfaced live on
  the dashboard as `drop N`.
- **Non-blocking recorder.** The record loop compensates for work time so the
  tick period stays close to target (measured jitter ±0.1 ms at 30 Hz).
- **Backend isolation.** Cameras and CAN start independently — if the CAN bus
  isn't up, camera preview still works and the API reports the CAN error rather
  than failing to boot.

## Trade-offs & simplifying assumptions

- **CAN codec is a plausible mock, not the exact OpenArm/DM-motor bit layout.**
  The real DM-series feedback packing is firmware-specific; the codec here is
  self-consistent (encode/decode round-trip tested) and mimics the data shape.
  Swapping in the real layout touches only `can_protocol.py`.
- **Frames are stored in-memory during an episode, flushed on stop.** Simple and
  fast for short demos; a long-running capture should stream to disk
  incrementally (chunked HDF5 appends) — noted in "Next steps."
- **Synthetic camera images** are tiny gradients so files stay small. Real
  capture replaces `Camera.grab()` only.
- **PNG preview instead of MJPEG stream.** The dashboard re-fetches a PNG at
  5 Hz — trivial and dependency-light. A real deployment would use an MJPEG or
  WebRTC stream for smooth high-fps preview.

## Next steps (with more time / hardware)

1. Replace the mock codec with the real DM-motor CAN-FD parsing against
   `openarm-can` and validate against `candump`.
2. Incremental/chunked HDF5 writes for long episodes; optional per-camera video
   encoding (H.264) instead of raw frames to cut size ~10–50×.
3. Hardware-timestamp the cameras (or a shared trigger) to tighten sync beyond
   software zero-order hold.
4. MJPEG/WebRTC live preview; per-joint plots and drop-rate graphs on the
   dashboard.
5. Dataset export adapter (e.g. to LeRobot/RLDS) for direct training use.
