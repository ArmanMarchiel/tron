"""REST + WebSocket + MJPEG API."""
from __future__ import annotations

import asyncio
import csv
import io
import json
import time
from pathlib import Path

from fastapi import APIRouter, Depends, Header, HTTPException, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import Response, StreamingResponse
from pydantic import BaseModel

from backend.app.config import ADAPTER, DATA_DIR, DEFAULT_ROBOT, DEFAULT_SCENARIO, ROBOT_ID, SAFETY, SIM
from backend.pipeline.events.event_model import Event, EventType
from backend.pipeline.incidents.timeline import markers, state_at, track
from backend.conf.registry import list_robots
from backend.conf.scenarios.loader import list_scenarios, load_scenario, save_scenario
from backend.pipeline.twin.robot_model import robot_summary
from backend.app.api.platform import Platform


class IngestBody(BaseModel):
    events: list[Event]


class SessionBody(BaseModel):
    robot: str = DEFAULT_ROBOT
    scenario: str = DEFAULT_SCENARIO
    adapter: str = "mujoco"


class ReplayBody(BaseModel):
    path: str
    speed: float = 1.0          # 0 = as fast as possible
    ground_truth: bool = True   # ingest /tron/ground_truth if present


class CameraBody(BaseModel):
    d_azimuth: float = 0.0
    d_elevation: float = 0.0
    zoom: float = 1.0
    reset: bool = False


class RecordBody(BaseModel):
    since: float | None = None
    until: float | None = None
    name: str | None = None


def build_router(p: Platform, sessions, auth) -> APIRouter:
    r = APIRouter(prefix="/api")
    require = auth.dependency()

    # ---------------------------------------------------------------- ingest (external adapters)
    @r.post("/ingest", dependencies=[Depends(require)])
    def ingest(body: IngestBody):
        p.ingest(body.events)
        return {"accepted": len(body.events)}

    # ---------------------------------------------------------------- session / registry / scenarios
    @r.get("/registry/robots")
    def registry_robots():
        return list_robots()

    @r.get("/session")
    def session():
        return sessions.info()

    @r.post("/session", dependencies=[Depends(require)])
    async def start_session(body: SessionBody, request: Request):
        try:
            info = await asyncio.to_thread(sessions.start, body.robot, body.scenario, body.adapter)
        except KeyError as e:
            raise HTTPException(404, str(e))
        except Exception as e:
            raise HTTPException(400, f"session start failed: {e}")
        p.record_audit(auth.actor(request), "session.start", body.model_dump())
        return info

    @r.get("/scenarios")
    def scenarios():
        cur = sessions.info()
        return [dict(s, active=(cur is not None and s["id"] == cur["scenario"])) for s in list_scenarios()]

    @r.get("/scenarios/{scenario_id}")
    def scenario(scenario_id: str):
        try:
            return load_scenario(scenario_id).model_dump()
        except KeyError:
            raise HTTPException(404, "unknown scenario")

    @r.put("/scenarios/{scenario_id}", dependencies=[Depends(require)])
    def put_scenario(scenario_id: str, body: dict, request: Request):
        try:
            sc = save_scenario(scenario_id, body)
        except Exception as e:
            raise HTTPException(400, f"invalid scenario: {e}")
        p.record_audit(auth.actor(request), "scenario.save", {"id": scenario_id})
        return sc.model_dump()

    # ---------------------------------------------------------------- robot / twin
    @r.get("/robots")
    def robots():
        return [robot_summary(p.state.twin)]

    @r.get("/robots/{robot_id}/twin")
    def twin(robot_id: str):
        if robot_id != p.state.robot_id:
            raise HTTPException(404, "unknown robot")
        return p.state.twin

    @r.get("/robots/{robot_id}/summary")
    def summary(robot_id: str):
        return robot_summary(p.state.twin)

    @r.get("/robots/{robot_id}/timeline/at")
    def twin_at(robot_id: str, ts: float):
        s = state_at(p.store, robot_id, ts)
        if s is None:
            raise HTTPException(404, "no snapshot at or before that time")
        return s

    @r.get("/robots/{robot_id}/timeline/track")
    def twin_track(robot_id: str, since: float | None = None, until: float | None = None, points: int = 600):
        until = until or time.time()
        since = since if since is not None else until - 120
        return {"since": since, "until": until, "points": track(p.store, robot_id, since, until, points), "markers": markers(p.store, since, until)}

    @r.get("/robots/{robot_id}/timeline/range")
    def twin_range(robot_id: str):
        lo, hi = p.store.snapshot_range(robot_id)
        return {"since": lo, "until": hi}

    # ---------------------------------------------------------------- events
    @r.get("/events")
    def events(since: float | None = None, until: float | None = None, types: str | None = None,
               sources: str | None = None, limit: int = Query(200, le=5000), after_seq: int | None = None, order: str = "desc"):
        tl = [EventType(t) for t in types.split(",")] if types else None
        sl = sources.split(",") if sources else None
        return [e.model_dump() for e in p.store.query(since=since, until=until, types=tl, sources=sl, limit=limit, after_seq=after_seq, ascending=(order == "asc"))]

    @r.get("/events/{event_id}")
    def event(event_id: str):
        e = p.store.get(event_id)
        if not e:
            raise HTTPException(404)
        return e.model_dump()

    @r.get("/events/types/all")
    def event_types():
        return [str(t) for t in EventType]

    @r.get("/export/events.csv")
    def export_events(since: float | None = None, until: float | None = None, limit: int = Query(50000, le=500000)):
        buf = io.StringIO()
        w = csv.writer(buf)
        w.writerow(["seq", "event_id", "timestamp", "event_type", "source", "entity_id", "confidence", "payload"])
        for e in p.store.query(since=since, until=until, limit=limit):
            w.writerow([e.seq, e.event_id, e.timestamp, str(e.event_type), e.source, e.entity_id, e.confidence, json.dumps(e.payload)])
        return Response(buf.getvalue(), media_type="text/csv", headers={"Content-Disposition": "attachment; filename=events.csv"})

    # ---------------------------------------------------------------- risk
    @r.get("/risk")
    def risk():
        return {"robot_id": ROBOT_ID, **p.state.twin["risk"], "safety_limits": p.risk.safety, "security_policy": p.security}

    # ---------------------------------------------------------------- incidents
    @r.get("/incidents")
    def incidents(limit: int = 100):
        return p.incidents.list(limit)

    @r.get("/incidents/{incident_id}")
    def incident(incident_id: str, include_operator: bool = False):
        inc = p.incidents.get(incident_id, include_operator=include_operator)
        if not inc:
            raise HTTPException(404)
        return inc

    @r.get("/incidents/{incident_id}/bundle.zip")
    async def incident_bundle(incident_id: str):
        from backend.io.collectors.replay_adapter import build_bundle
        inc = p.incidents.get(incident_id, include_operator=True)
        if not inc:
            raise HTTPException(404)
        data = await asyncio.to_thread(build_bundle, p, inc)
        return Response(data, media_type="application/zip", headers={"Content-Disposition": f"attachment; filename={incident_id}.zip"})

    # ---------------------------------------------------------------- faults
    @r.get("/faults")
    def faults():
        return p.faults.list()

    @r.post("/faults/{fault_id}/trigger", dependencies=[Depends(require)])
    async def trigger(fault_id: str, request: Request, duration_s: float | None = None):
        try:
            res = await p.faults.inject(fault_id, duration_s)
        except KeyError:
            raise HTTPException(404, f"unknown fault {fault_id}")
        p.record_audit(auth.actor(request), "fault.inject", {"id": fault_id})
        return res

    @r.post("/faults/clear", dependencies=[Depends(require)])
    async def clear(request: Request):
        p.record_audit(auth.actor(request), "fault.clear", {})
        return {"cleared": await p.faults.clear()}

    @r.get("/faults/active")
    def active_fault():
        return p.faults.active

    # backwards-compatible aliases
    @r.get("/scenarios/faults/list")
    def faults_alias():
        return p.faults.list()

    # ---------------------------------------------------------------- replay / record
    @r.post("/replay", dependencies=[Depends(require)])
    async def replay(body: ReplayBody, request: Request):
        from backend.io.collectors.replay_adapter import ReplayAdapter
        path = Path(body.path)
        if not path.exists():
            raise HTTPException(404, f"no such file {path}")
        if p.adapter is not None and getattr(p.adapter, "name", "") == "mujoco":
            raise HTTPException(409, "stop the simulator first: start a session with adapter=external")
        ad = ReplayAdapter(p.ingest, path, speed=body.speed, ground_truth=body.ground_truth, robot_id=ROBOT_ID)
        p.adapter = ad
        ad.start()
        p.record_audit(auth.actor(request), "replay.start", body.model_dump())
        return ad.status()

    @r.get("/replay/status")
    def replay_status():
        ad = p.adapter
        return ad.status() if ad is not None and getattr(ad, "name", "") == "replay" else {"state": "idle"}

    @r.post("/record", dependencies=[Depends(require)])
    async def record(body: RecordBody, request: Request):
        from backend.io.collectors.replay_adapter import record_mcap
        until = body.until or time.time()
        since = body.since or (until - 300)
        out = Path(DATA_DIR) / "recordings" / (body.name or f"tron_{int(until)}.mcap")
        out.parent.mkdir(parents=True, exist_ok=True)
        n = await asyncio.to_thread(record_mcap, p.store, out, since, until, p.state.twin["identity"])
        p.record_audit(auth.actor(request), "record", {"path": str(out), "events": n})
        return {"path": str(out), "events": n, "since": since, "until": until}

    @r.get("/recordings")
    def recordings():
        d = Path(DATA_DIR) / "recordings"
        return [{"path": str(f), "size": f.stat().st_size, "mtime": f.stat().st_mtime} for f in sorted(d.glob("*.mcap"))] if d.exists() else []

    # ---------------------------------------------------------------- simulator view
    def _camera(view: str | None) -> str:
        cams = SIM["cameras"]
        if view in cams:
            return cams[view]
        if view in cams.values():
            return view
        return SIM["camera"]

    @r.get("/sim/stream")
    async def sim_stream(view: str | None = None):
        if p.renderer is None:
            raise HTTPException(503, "no in-process simulator")
        cam = _camera(view)
        rd = p.renderer

        async def gen():
            rd.acquire(cam)
            last = -1
            try:
                while True:
                    fid = rd.frame_id.get(cam, 0)
                    frame = rd.latest.get(cam)
                    if frame is not None and fid != last:
                        last = fid
                        yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"
                    if rd._stop:
                        break
                    await asyncio.sleep(1.0 / SIM["render_hz"])
            finally:
                rd.release(cam)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame", headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

    @r.get("/sim/frame")
    async def sim_frame(ts: float | None = None, view: str | None = None):
        if p.renderer is None:
            raise HTTPException(503, "no in-process simulator")
        cam = _camera(view)
        qpos, mocap = p.adapter.live_state()
        if ts is not None:
            snap = p.store.snapshot_at(ROBOT_ID, ts)
            if snap is None:
                raise HTTPException(404, "no snapshot at or before that time")
            for i, v in enumerate(snap["physical"]["position"][: p.adapter.n]):
                qpos[p.adapter.qadr[i]] = v
            humans = snap["environment"].get("humans") or []
            if humans and mocap is not None and "z" in humans[0]:
                pos, quat = mocap
                pos = pos.copy(); pos[0] = [humans[0]["x"], humans[0]["y"], humans[0]["z"]]
                mocap = (pos, quat)
        elif p.renderer.latest.get(cam) is not None:
            return Response(p.renderer.latest[cam], media_type="image/jpeg", headers={"Cache-Control": "no-store"})
        jpeg = await asyncio.wrap_future(p.renderer.render_pose(qpos, mocap, cam))
        return Response(jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @r.post("/sim/camera")
    def sim_camera(body: CameraBody):
        if p.renderer is None:
            raise HTTPException(503, "no in-process simulator")
        return p.renderer.orbit_update(body.d_azimuth, body.d_elevation, body.zoom, body.reset)

    @r.get("/sim/views")
    def sim_views():
        return [{"id": k, "camera": v, "label": "Environment" if k == "environment" else "Robot camera"} for k, v in SIM["cameras"].items()]

    @r.get("/sim/waypoints")
    def sim_waypoints():
        cur = sessions.info()
        return cur.get("targets", {}) if cur else {}

    # ---------------------------------------------------------------- meta
    @r.get("/status")
    def status():
        return {"robot_id": ROBOT_ID, "adapter": (getattr(p.adapter, "name", None) or "external"), "session": sessions.info(),
                "adapters_seen": p.state.twin["identity"]["adapters"], "events": p.store.count(), "active_fault": p.faults.active,
                "renderer": p.renderer is not None, "server_time": time.time()}

    @r.get("/audit")
    def audit(limit: int = 200):
        return p.audit[-limit:]

    @r.get("/security/baseline")
    def baseline_get():
        from backend.conf.security.baseline import current_baseline
        return current_baseline(p)

    @r.post("/security/baseline/capture", dependencies=[Depends(require)])
    def baseline_capture(request: Request, since: float | None = None):
        from backend.conf.security.baseline import capture_baseline
        res = capture_baseline(p, since)
        p.record_audit(auth.actor(request), "baseline.capture", {"nodes": len(res.get("expected_nodes", []))})
        return res

    # ---------------------------------------------------------------- websocket live stream
    @r.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        q: asyncio.Queue = asyncio.Queue(maxsize=500)
        p.ws_clients.append(q)
        try:
            await sock.send_text(json.dumps({"type": "twin", "data": p.state.twin, "fault": p.faults.active, "session": sessions.info()}))
            last_twin = 0.0
            while True:
                try:
                    msg = await asyncio.wait_for(q.get(), timeout=0.25)
                    await sock.send_text(msg)
                except asyncio.TimeoutError:
                    pass
                now = time.time()
                if now - last_twin >= 0.2:
                    last_twin = now
                    await sock.send_text(json.dumps({"type": "twin", "data": p.state.twin, "fault": p.faults.active, "session": sessions.info()}))
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            if q in p.ws_clients:
                p.ws_clients.remove(q)

    return r
