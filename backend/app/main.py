"""ASGI entry point.  Run with:  .venv/bin/uvicorn backend.app.main:app --port 8000"""
from __future__ import annotations

import asyncio
import logging
import os
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from contextlib import asynccontextmanager

from backend.app.api.platform import Platform
from backend.app.api.routes import build_router
from backend.app.config import ADAPTER, DATA_DIR, DEFAULT_ROBOT, DEFAULT_SCENARIO, ROOT
from backend.conf.security.auth import Auth
from backend.app.session import SessionManager

log = logging.getLogger("tron")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
Path(DATA_DIR).mkdir(parents=True, exist_ok=True)

platform = Platform(":memory:")          # replaced by the session's SQLite file on start
sessions = SessionManager(platform)
auth = Auth()


@asynccontextmanager
async def lifespan(app: FastAPI):
    platform.loop = asyncio.get_running_loop()
    adapter = "external" if ADAPTER == "external" else "mujoco"
    try:
        info = await asyncio.to_thread(sessions.start, DEFAULT_ROBOT, DEFAULT_SCENARIO, adapter)
        log.info("session %s started (robot=%s scenario=%s adapter=%s)", info["id"], info["robot"], info["scenario"], adapter)
    except Exception as e:
        log.exception("could not start default session: %s", e)
    if auth.enabled:
        log.info("API tokens enabled for %d principal(s)", len(auth.tokens))
    yield
    sessions.stop()
    if platform.store:
        platform.store.close()


app = FastAPI(title="TRON - Cyber-Physical Digital Twin", lifespan=lifespan)
app.include_router(build_router(platform, sessions, auth))

FRONTEND = ROOT / "frontend"


class _NoCacheStatic(StaticFiles):
    """Serve the UI without caching.

    This is a development UI edited in place; a browser holding an old ES module or stylesheet makes
    changes look like they never landed. Revalidating every request costs nothing on localhost."""

    def file_response(self, *args, **kwargs):
        resp = super().file_response(*args, **kwargs)
        resp.headers["Cache-Control"] = "no-store, must-revalidate"
        return resp


app.mount("/static", _NoCacheStatic(directory=str(FRONTEND)), name="static")


@app.get("/healthz")
def healthz():
    ok = platform.store is not None and (platform.adapter is not None or ADAPTER == "external")
    return JSONResponse({"ok": ok, "session": sessions.info() and sessions.info()["id"], "adapter": getattr(platform.adapter, "name", None)}, status_code=200 if ok else 503)


@app.get("/")
def index():
    """Serve the shell with a build stamp on its assets.

    ES modules and stylesheets cache hard; without a changing query a browser keeps serving an old
    module graph after the files on disk change. The stamp is the newest mtime under frontend/, so it
    moves exactly when the UI does and stays stable otherwise."""
    html = (FRONTEND / "index.html").read_text()
    stamp = int(max(f.stat().st_mtime for f in FRONTEND.rglob("*") if f.is_file()))
    html = html.replace("/static/app.css", f"/static/app.css?v={stamp}")
    html = html.replace("/static/app.js", f"/static/app.js?v={stamp}")
    return HTMLResponse(html, headers={"Cache-Control": "no-cache"})
