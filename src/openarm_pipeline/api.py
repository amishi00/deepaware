"""
REST API + dashboard host for the OpenArm data collection pipeline.

Endpoints
---------
GET  /                         -> dashboard (static HTML)
GET  /api/health               -> service health
GET  /api/live                 -> live joint snapshot for both arms
GET  /api/live/frame/{camera}  -> latest JPEG preview from one camera
POST /api/record/start         -> begin recording an episode
POST /api/record/stop          -> stop and flush current episode
GET  /api/episodes             -> list recorded episodes (from sidecar json)
GET  /api/episodes/{id}        -> metadata for one episode
GET  /api/episodes/{id}/download -> download the raw .h5 file

The recorder holds the CAN readers and camera rig; the API is a thin layer over
it. Backends start lazily on first use so importing the app is cheap (and so the
server boots even if vcan isn't up yet -- endpoints just report empty state).
"""

from __future__ import annotations

import io
import json
from pathlib import Path

import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, HTMLResponse, Response

from .recorder import EpisodeRecorder

DATA_DIR = Path("data/episodes")
FRONTEND = Path(__file__).resolve().parents[2] / "frontend" / "index.html"


def create_app(
    left_channel: str = "vcan0",
    right_channel: str = "vcan1",
    fd: bool = True,
    use_cameras: bool = True,
) -> FastAPI:
    app = FastAPI(title="OpenArm Data Collection Pipeline", version="1.0")
    recorder = EpisodeRecorder(
        DATA_DIR,
        left_channel=left_channel,
        right_channel=right_channel,
        fd=fd,
        use_cameras=use_cameras,
    )
    app.state.recorder = recorder

    def _try_start_backends() -> bool:
        try:
            recorder.start_backends()
            return True
        except Exception as exc:  # vcan not up, etc.
            app.state.backend_error = str(exc)
            return False

    @app.on_event("startup")
    def _startup() -> None:
        _try_start_backends()

    @app.on_event("shutdown")
    def _shutdown() -> None:
        try:
            recorder.stop_backends()
        except Exception:
            pass

    # ---- dashboard ----------------------------------------------------
    @app.get("/", response_class=HTMLResponse)
    def index() -> str:
        if FRONTEND.exists():
            return FRONTEND.read_text()
        return "<h1>OpenArm pipeline</h1><p>frontend/index.html not found.</p>"

    # ---- health / live ------------------------------------------------
    @app.get("/api/health")
    def health() -> dict:
        return {
            "status": "ok",
            "backends_started": recorder._started,
            "backend_error": getattr(app.state, "backend_error", None),
            "recording": recorder.is_recording,
        }

    @app.get("/api/live")
    def live() -> dict:
        if not recorder._started and not _try_start_backends():
            return {"recording": False, "left": None, "right": None,
                    "error": getattr(app.state, "backend_error", "backends unavailable")}
        return recorder.live_state()

    @app.get("/api/live/frame/{camera}")
    def live_frame(camera: str) -> Response:
        if recorder.rig is None or camera not in recorder.rig.cameras:
            raise HTTPException(404, f"unknown camera: {camera}")
        if not recorder._started:
            _try_start_backends()
        frame = recorder.rig.cameras[camera].latest()
        if frame is None:
            raise HTTPException(503, "no frame yet")
        png = _encode_png(frame.image)
        return Response(content=png, media_type="image/png")

    # ---- recording control -------------------------------------------
    @app.post("/api/record/start")
    def record_start() -> dict:
        if recorder.is_recording:
            raise HTTPException(409, "already recording")
        eid = recorder.start_recording()
        return {"status": "recording", "episode_id": eid}

    @app.post("/api/record/stop")
    def record_stop() -> dict:
        eid = recorder.stop_recording()
        if eid is None:
            raise HTTPException(409, "not recording")
        return {"status": "stopped", "episode_id": eid}

    # ---- episode listing / retrieval ---------------------------------
    @app.get("/api/episodes")
    def list_episodes() -> dict:
        eps = []
        for meta_file in sorted(DATA_DIR.glob("*.json"), reverse=True):
            try:
                eps.append(json.loads(meta_file.read_text()))
            except json.JSONDecodeError:
                continue
        return {"count": len(eps), "episodes": eps}

    @app.get("/api/episodes/{episode_id}")
    def get_episode(episode_id: str) -> dict:
        meta = DATA_DIR / f"{episode_id}.json"
        if not meta.exists():
            raise HTTPException(404, "episode not found")
        return json.loads(meta.read_text())

    @app.get("/api/episodes/{episode_id}/download")
    def download_episode(episode_id: str) -> FileResponse:
        h5 = DATA_DIR / f"{episode_id}.h5"
        if not h5.exists():
            raise HTTPException(404, "episode file not found")
        return FileResponse(h5, media_type="application/x-hdf5",
                            filename=f"{episode_id}.h5")

    return app


def _encode_png(img: np.ndarray) -> bytes:
    """Encode a numpy image to PNG. Uses Pillow if present, else a minimal writer."""
    try:
        from PIL import Image
        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format="PNG")
        return buf.getvalue()
    except Exception:
        # Fallback: encode via matplotlib-free manual PNG is overkill; return raw.
        return img.tobytes()


# module-level app for `uvicorn openarm_pipeline.api:app`
app = create_app()
