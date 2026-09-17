"""Wires the core together: store -> state engine -> risk -> incidents, plus live fan-out.

``configure()`` (re)builds the engines for a session: a fresh SQLite file per session, robot identity and
scenario environment injected into the twin, per-robot safety overrides.  The event pipeline itself is
unchanged between sessions and adapters.
"""
from __future__ import annotations

import asyncio
import json
import time
from collections.abc import Iterable
from pathlib import Path

from backend.app.config import DATA_DIR, DB_PATH, ROBOT_ID, SAFETY, SECURITY
from backend.pipeline.events.event_model import Event, EventType, Source, make_event
from backend.pipeline.events.event_store import EventStore
from backend.pipeline.faults.manager import FaultManager
from backend.pipeline.incidents.incident_engine import IncidentEngine
from backend.pipeline.risk.anomaly_engine import AnomalyEngine
from backend.pipeline.twin.state_engine import StateEngine

LIVE_TYPES = {EventType.AnomalyDetected, EventType.SafetyThresholdExceeded, EventType.CollisionDetected,
              EventType.IncidentCreated, EventType.IncidentUpdated, EventType.IncidentClosed,
              EventType.FaultInjected, EventType.FaultCleared, EventType.StateDivergenceObserved,
              EventType.TrajectoryGoalReceived, EventType.TrajectoryGoalReached,
              EventType.TaskStepStarted, EventType.TaskStepCompleted, EventType.TaskStepFailed,
              EventType.MachineCommandReceived, EventType.InterlockViolated,
              EventType.ROSNodeStarted, EventType.ROSNodeStopped, EventType.ReplayStarted, EventType.ReplayFinished,
              EventType.SessionStarted}


class Platform:
    def __init__(self, db_path: str = DB_PATH, identity: dict | None = None, environment: dict | None = None,
                 safety: dict | None = None, security: dict | None = None, faults: dict | None = None):
        self.ws_clients: list[asyncio.Queue] = []
        self.loop: asyncio.AbstractEventLoop | None = None
        self.renderer = None
        self.adapter = None
        self.session: dict | None = None
        self.audit: list[dict] = []
        self.store: EventStore | None = None
        self.configure(db_path, identity, environment, safety, security, faults)

    def configure(self, db_path: str, identity: dict | None = None, environment: dict | None = None,
                  safety: dict | None = None, security: dict | None = None, faults: dict | None = None) -> None:
        if self.store is not None:
            try:
                self.store.close()
            except Exception:
                pass
        self.db_path = db_path
        self.store = EventStore(db_path)
        self.security = security or SECURITY
        self.state = StateEngine(self.store, ROBOT_ID, identity, environment, safety, self.security)
        self.risk = AnomalyEngine(self.store, safety=safety, security=self.security)
        self.incidents = IncidentEngine(self.store, lambda: self.state.twin, self.risk.active_rules)
        self.faults = FaultManager(self.store, ROBOT_ID, faults)
        self.state.risk_hooks = [self.risk.evaluate]
        self.store.subscribe(self.state.handle)
        self.store.subscribe(self.incidents.handle)
        self.store.subscribe(self._fanout)

    def ingest(self, events: Iterable[Event]) -> None:
        for e in events:
            self.store.append(e)

    def record_audit(self, actor: str, action: str, detail: dict | None = None) -> None:
        entry = {"ts": time.time(), "actor": actor, "action": action, "detail": detail or {}}
        self.audit.append(entry)
        self.audit = self.audit[-500:]

    def _fanout(self, e: Event) -> None:
        if e.event_type not in LIVE_TYPES or not self.ws_clients:
            return
        msg = json.dumps({"type": "event", "data": e.model_dump()})

        def _push():
            for q in list(self.ws_clients):
                try:
                    q.put_nowait(msg)
                except asyncio.QueueFull:
                    pass
        if self.loop is not None and not self.loop.is_closed():
            try:
                self.loop.call_soon_threadsafe(_push)
            except RuntimeError:
                pass
