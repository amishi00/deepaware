# OpenArm 2.0 — Data Collection Pipeline

A data collection platform for the OpenArm 2.0 robotic arm: reads joint
telemetry over CAN FD, synchronizes it with four cameras, stores episodes as
HDF5, and exposes a REST API + live monitoring dashboard.

Built for the DeepAware AI / Robotics Center of Silicon Valley take-home.

> **Hardware honesty:** I developed without an OpenArm or a CAN-FD adapter. I
> used Linux **virtual CAN (`vcan0`/`vcan1`)** as a real SocketCAN bus and a
> mock firmware publisher to drive it, so the entire pipeline runs through the
> same code path it would use on hardware — only the frame *source* is
> simulated. Everything below actually runs.

## Tasks completed

| # | Task | Status | Where |
|---|------|--------|-------|
| 1 | CAN interface setup | ✅ (via vcan; see note) | `scripts/setup_vcan.sh`, below |
| 2 | CAN data reading (pos/vel/torque) | ✅ | `can_reader.py`, `can_protocol.py` |
| 3 | Multi-camera synchronization | ✅ | `cameras.py`, `recorder.py`, `docs/DESIGN.md` |
| 4 | Data storage backend + REST API | ✅ HDF5 + FastAPI | `recorder.py`, `api.py` |
| 5 | Monitoring dashboard | ✅ | `frontend/index.html` |

Design decisions, sync strategy, and storage-format rationale are in
[`docs/DESIGN.md`](docs/DESIGN.md).

## Quick start

```bash
# 1. install
pip install -r requirements.txt

# 2. bring up virtual CAN (stands in for can0/can1)
bash scripts/setup_vcan.sh

# 3. run everything: mock publishers on both buses + API + dashboard
bash scripts/run_stack.sh

# 4. open the dashboard
#    http://127.0.0.1:8000
```

Then click **Start recording**, move through a few seconds, **Stop**, and the
episode appears in the sidebar with a download link.

## Task 1 — CAN setup and the `can_configure` caveat

On real hardware the setup is:

```bash
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
openarm-can-cli -i can0 can_configure
openarm-can-cli -i can0 set_zero --arm      # repeat for can1
```

Without hardware I used virtual CAN:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
sudo ip link add dev vcan1 type vcan && sudo ip link set up vcan1
```

**`openarm-can-cli can_configure` fails on vcan with `Operation not supported`** —
and that's expected. `can_configure` applies CAN-FD *bitrate/timing* parameters,
but a virtual bus has no physical layer to time. vcan still transports frames,
which is all the pipeline needs. The hardware `can_configure`/`set_zero` commands
are the ones shown above.

Verify the interfaces are up:

```bash
ip -br link show type vcan     # both vcan0, vcan1 show state UNKNOWN/UP
candump vcan0                  # in one terminal
cansend vcan0 100#DEADBEEF     # in another — frame appears in candump
```

## Task 2 — CAN data reading

`can_protocol.py` defines the frame codec: each joint publishes an 8-byte CAN FD
payload packing position, velocity, and torque (uint16-scaled to physical
ranges) plus a rolling counter for drop detection. CAN IDs are `0x100 + joint`.

`can_reader.py` runs a background listener per bus, decodes frames into
`JointState`s, keeps a live per-joint snapshot, and counts dropped frames via the
counter. It's identical against real `can0/can1` or mock `vcan0/vcan1`.

```bash
# watch decoded joint state live (after starting a publisher)
PYTHONPATH=src python -m openarm_pipeline.can_reader vcan0
```

## Task 3 — Multi-camera sync

Four simulated cameras (wrist L/R 60fps, ceiling 30fps, ZED stereo 15fps) each
run in their own thread. The recorder samples on a fixed 30 Hz master clock
driven by the joint stream, taking each camera's most-recent frame (zero-order
hold) and storing that frame's **true capture timestamp** alongside the tick
time. Full rationale — why zero-order hold over interpolation, how different fps
are handled — in [`docs/DESIGN.md`](docs/DESIGN.md#synchronization-strategy-task-3).

## Task 4 — Storage + REST API

Episodes are stored as **HDF5** (chosen over MCAP/zarr/custom — see DESIGN.md),
one self-describing file per episode with a JSON sidecar for fast listing.

```
GET  /api/health                    service + backend status
GET  /api/live                      live joint snapshot (both arms)
GET  /api/live/frame/{camera}       latest camera preview (PNG)
POST /api/record/start              begin an episode
POST /api/record/stop               stop + flush to HDF5
GET  /api/episodes                  list episodes
GET  /api/episodes/{id}             episode metadata
GET  /api/episodes/{id}/download    download the .h5
```

Inspect a recorded episode:

```bash
PYTHONPATH=src python -m openarm_pipeline.inspect data/episodes/<id>.h5
```

## Task 5 — Dashboard

`http://127.0.0.1:8000` — live per-joint position/velocity/torque for both arms,
frame counters and drop rate, previews from all four cameras, episode list with
downloads, and a Start/Stop recording button with an elapsed timer. Single HTML
file, no build step.

## Tests

```bash
PYTHONPATH=src python tests/test_pipeline.py     # codec + storage round-trip
```

## Project layout

```
src/openarm_pipeline/
  can_protocol.py   frame encode/decode + JointState
  can_publisher.py  mock firmware: streams joints onto vcan
  can_reader.py     background CAN listener + drop detection
  cameras.py        4-camera simulation (async fps)
  recorder.py       master-clock sync + HDF5 writer
  api.py            FastAPI REST API + dashboard host
  inspect.py        episode inspector CLI
frontend/index.html monitoring dashboard
scripts/            setup_vcan.sh, run_stack.sh
tests/              test_pipeline.py
docs/DESIGN.md      architecture, sync, storage rationale, trade-offs
```

## What I'd do next

Replace the mock codec with real DM-motor CAN-FD parsing, stream frames to disk
incrementally for long episodes, add hardware camera timestamps, and swap the
PNG preview for an MJPEG/WebRTC stream. Full list in DESIGN.md.
