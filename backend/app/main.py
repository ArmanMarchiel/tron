"""ASGI entry point.  Run with:  .venv/bin/uvicorn backend.app.main:app --port 8000"""
from __future__ import annotations

import asyncio
import logging
import os
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import FileResponse, JSONResponse
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
app.mount("/static", StaticFiles(directory=str(FRONTEND)), name="static")


@app.get("/healthz")
def healthz():
    ok = platform.store is not None and (platform.adapter is not None or ADAPTER == "external")
    return JSONResponse({"ok": ok, "session": sessions.info() and sessions.info()["id"], "adapter": getattr(platform.adapter, "name", None)}, status_code=200 if ok else 503)


@app.get("/")
def index():
    return FileResponse(str(FRONTEND / "index.html"))
