"""Groups anomaly events into incidents and manages their lifecycle.

An incident is opened by the first anomaly for an entity when none is open (or the open
one has been quiet for longer than the grouping window).  Subsequent anomalies attach.
The incident closes after ``close_after_s`` seconds without anomaly activity and with no
rule still active.  Reconstruction is computed on read (and frozen at close).
"""
from __future__ import annotations

import uuid

from backend.app.config import INCIDENTS
from backend.pipeline.events.event_model import Event, EventType, Source, make_event
from backend.pipeline.events.event_store import EventStore
from backend.pipeline.risk.safety_rules import SEVERITY_ORDER
from backend.pipeline.incidents.reconstruction import reconstruct

ANOMALY_TYPES = {EventType.AnomalyDetected, EventType.SafetyThresholdExceeded, EventType.CollisionDetected}

TITLES = {
    "JOINT_VELOCITY_LIMIT_EXCEEDED": "Unexpected robot motion (joint velocity)",
    "JOINT_LIMIT_EXCEEDED": "Joint limit exceeded",
    "TORQUE_LIMIT_EXCEEDED": "Joint torque limit exceeded",
    "UNEXPECTED_COMMAND_SOURCE": "Unauthorised trajectory source",
    "TASK_STEP_TIMEOUT": "Task step overran its window",
    "TASK_ORDER_VIOLATION": "Task executed out of sequence",
    "INTERLOCK_VIOLATION": "Machine interlock violated",
    "MACHINE_STATE_MISMATCH": "Robot acted on a wrong machine state",
    "UNEXPECTED_MACHINE_COMMAND_SOURCE": "Unauthorised machine command source",
    "REDUCED_SPEED_ZONE_VIOLATION": "Speed limit exceeded in collaborative zone",
    "SSM_VIOLATION": "Speed-and-separation violation",
    "PROTECTIVE_STOP_NOT_ISSUED": "Protective stop not issued",
    "UNEXPECTED_LOAD": "Unexpected external load",
    "SENSOR_STATE_DIVERGENCE": "Sensor / physical state disagreement",
    "TRAJECTORY_DIVERGENCE": "Navigation trajectory deviation",
    "COLLISION_EVENT": "Collision",
    "WORKSPACE_VIOLATION": "Restricted zone entered",
    "PROXIMITY_RISK": "Human proximity risk",
    "UNEXPECTED_ROS_NODE": "Unexpected ROS node",
    "UNEXPECTED_MESSAGE_RATE": "Unexpected message rate",
    "UNEXPECTED_CONNECTION": "Unexpected network participant",
    "UNEXPECTED_CONTROLLER_COMMAND": "Command outside controller limits",
    "UNEXPECTED_TOPIC_PUBLISHER": "Unexpected topic publisher",
    "UNEXPECTED_TOPIC_SUBSCRIBER": "Unexpected topic subscriber",
}


class IncidentEngine:
    def __init__(self, store: EventStore, get_twin, active_rules):
        self.store = store
        self.get_twin = get_twin          # callable -> twin dict
        self.active_rules = active_rules  # callable -> list[str]
        self._open: dict[str, dict] = {}  # entity_id -> incident
        self._counter = self._load_counter()

    def _load_counter(self) -> int:
        incs = self.store.list_incidents(limit=1)
        if incs:
            try:
                return int(incs[0]["incident_id"].split("-")[-1])
            except ValueError:
                return 0
        return 0

    # ------------------------------------------------------------------ event hook
    def handle(self, ev: Event) -> None:
        if ev.event_type in ANOMALY_TYPES:
            if ev.payload.get("informational"):
                return
            if ev.payload.get("cleared"):
                self._touch_activity(ev)
            else:
                self._attach(ev)
        elif ev.event_type == EventType.RobotStateObserved:
            self._maybe_close(ev.entity_id, ev.timestamp)

    def _touch_activity(self, ev: Event) -> None:
        inc = self._open.get(ev.entity_id)
        if inc:
            inc["timeline_event_ids"].append(ev.event_id)
            inc["updated_at"] = ev.timestamp
            self.store.upsert_incident(inc)

    def _attach(self, ev: Event) -> None:
        pl = ev.payload
        rule = pl.get("rule", "COLLISION_EVENT" if ev.event_type == EventType.CollisionDetected else "ANOMALY")
        sev = pl.get("severity", "HIGH")
        inc = self._open.get(ev.entity_id)
        if inc and ev.timestamp - inc["last_anomaly_at"] > INCIDENTS["group_window_s"]:
            self._close(inc, ev.timestamp)
            inc = None
        if inc is None:
            self._counter += 1
            inc = {
                "incident_id": f"INC-{self._counter:04d}",
                "entity_id": ev.entity_id,
                "created_at": ev.timestamp, "updated_at": ev.timestamp, "closed_at": None,
                "last_anomaly_at": ev.timestamp,
                "status": "open", "severity": sev,
                "title": TITLES.get(rule, rule.replace("_", " ").title()),
                "primary_rule": rule, "rules": [rule],
                "anomaly_event_ids": [ev.event_id], "timeline_event_ids": [ev.event_id],
                "reconstruction": None,
            }
            self._open[ev.entity_id] = inc
            self.store.upsert_incident(inc)
            self.store.append(make_event(EventType.IncidentCreated, Source.PLATFORM, ev.entity_id, {
                "incident_id": inc["incident_id"], "title": inc["title"], "severity": sev, "rule": rule,
                "trigger_event_id": ev.event_id}, 1.0, ev.timestamp))
            return
        inc["last_anomaly_at"] = ev.timestamp
        inc["updated_at"] = ev.timestamp
        inc["anomaly_event_ids"].append(ev.event_id)
        inc["timeline_event_ids"].append(ev.event_id)
        changed = False
        if rule not in inc["rules"]:
            inc["rules"].append(rule)
            changed = True
        if SEVERITY_ORDER[sev] > SEVERITY_ORDER[inc["severity"]]:
            inc["severity"] = sev
            inc["primary_rule"] = rule
            inc["title"] = TITLES.get(rule, rule.replace("_", " ").title())
            changed = True
        self.store.upsert_incident(inc)
        if changed:
            self.store.append(make_event(EventType.IncidentUpdated, Source.PLATFORM, ev.entity_id, {
                "incident_id": inc["incident_id"], "title": inc["title"], "severity": inc["severity"],
                "rules": inc["rules"], "trigger_event_id": ev.event_id}, 1.0, ev.timestamp))

    def _maybe_close(self, entity_id: str, ts: float) -> None:
        inc = self._open.get(entity_id)
        if not inc:
            return
        if ts - inc["last_anomaly_at"] >= INCIDENTS["close_after_s"] and not self.active_rules():
            self._close(inc, ts)

    def _close(self, inc: dict, ts: float) -> None:
        inc["status"] = "closed"
        inc["closed_at"] = ts
        inc["updated_at"] = ts
        inc["reconstruction"] = reconstruct(self.store, inc, include_operator=True)
        self.store.upsert_incident(inc)
        self._open.pop(inc["entity_id"], None)
        self.store.append(make_event(EventType.IncidentClosed, Source.PLATFORM, inc["entity_id"], {
            "incident_id": inc["incident_id"], "title": inc["title"], "severity": inc["severity"]}, 1.0, ts))

    # ------------------------------------------------------------------ queries
    def get(self, incident_id: str, include_operator: bool = False) -> dict | None:
        inc = self.store.get_incident(incident_id)
        if not inc:
            return None
        inc = dict(inc)
        inc["reconstruction"] = reconstruct(self.store, inc, include_operator=include_operator)
        return inc

    def list(self, limit: int = 100) -> list[dict]:
        out = []
        for inc in self.store.list_incidents(limit):
            inc = dict(inc)
            inc.pop("reconstruction", None)
            out.append(inc)
        return out
