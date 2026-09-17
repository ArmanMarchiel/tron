"""Twin timeline: reconstruct the complete state of the robot at any historical instant."""
from __future__ import annotations

from backend.pipeline.events.event_model import EventType
from backend.pipeline.events.event_store import EventStore


def state_at(store: EventStore, entity_id: str, ts: float, event_window_s: float = 1.0) -> dict | None:
    snap = store.snapshot_at(entity_id, ts)
    if snap is None:
        return None
    events = store.query(since=ts - event_window_s, until=ts, limit=200)
    return {
        "ts": ts, "snapshot_ts": snap.get("ts"), "twin": snap,
        "events": [e.model_dump() for e in events],
        "commands": [e.model_dump() for e in events if e.event_type == EventType.CommandReceived][-5:],
    }


def track(store: EventStore, entity_id: str, since: float, until: float, max_points: int = 600) -> list[dict]:
    """Downsampled series for the timeline strip."""
    snaps = store.snapshots(entity_id, since, until, limit=20000)
    if len(snaps) > max_points:
        step = len(snaps) / max_points
        snaps = [snaps[int(i * step)] for i in range(max_points)]
    out = []
    for s in snaps:
        vel = max((abs(v) for v in s["physical"]["velocity"]), default=0.0)
        vexp = max((abs(v) for v in s["expected"]["velocity"]), default=0.0)
        out.append({
            "ts": s["ts"], "vel": round(vel, 3), "vel_exp": round(vexp, 3),
            "q": s["physical"]["position"], "q_exp": s["expected"]["position"], "q_rep": s["software_belief"]["position"],
            "qd": s["physical"]["velocity"], "errs": s["divergence"]["joint_errors"],
            "err": s["divergence"]["joint_max_abs"], "ee_m": s["divergence"]["ee_m"], "enc": s["divergence"]["encoder_max_abs"],
            "ee_speed": s["physical"]["ee_speed"], "div_level": s["divergence"]["level"],
            "risk": s["risk"]["level"], "rules": [r["rule"] for r in s["risk"]["active_rules"]], "incidents": s["incidents"]["active"],
        })
    return out


def markers(store: EventStore, since: float, until: float) -> list[dict]:
    evs = store.query(since=since, until=until, types=[
        EventType.AnomalyDetected, EventType.SafetyThresholdExceeded, EventType.CollisionDetected,
        EventType.IncidentCreated, EventType.IncidentClosed, EventType.TrajectoryGoalReceived,
        EventType.TrajectoryGoalReached, EventType.FaultInjected, EventType.FaultCleared,
        EventType.StateDivergenceObserved, EventType.ROSNodeStarted, EventType.ROSNodeStopped,
        EventType.TaskStepStarted], limit=2000)
    return [{"ts": e.timestamp, "type": str(e.event_type), "id": e.event_id,
             "label": e.payload.get("rule") or e.payload.get("incident_id") or e.payload.get("node")
             or e.payload.get("scenario") or e.payload.get("step") or e.payload.get("goal_name") or e.payload.get("level") or "",
             "cleared": bool(e.payload.get("cleared")), "first": e.payload.get("first")} for e in evs]
