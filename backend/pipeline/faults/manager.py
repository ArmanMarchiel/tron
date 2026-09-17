"""Fault manager: records operator-triggered scenarios and exposes them to adapters.

Faults come from the active scenario file (``faults:``). The platform never modifies the robot; it
records the operator's request as a ``FaultInjected`` event and hands the fault to the in-process
adapter (or exposes it at ``GET /api/faults/active`` for out-of-process adapters).
"""
from __future__ import annotations

import asyncio
import time

from backend.pipeline.events.event_model import EventType, Source, make_event
from backend.pipeline.events.event_store import EventStore


class FaultManager:
    def __init__(self, store: EventStore, robot_id: str, faults: dict | None = None):
        self.store = store
        self.robot_id = robot_id
        self.scenarios: dict[str, dict] = {}
        self.set_faults(faults or {})
        self.active: dict | None = None
        self.adapters: list = []
        self._clear_task: asyncio.Task | None = None

    def set_faults(self, faults: dict) -> None:
        self.scenarios = {fid: {"id": fid, **(f if isinstance(f, dict) else f.model_dump())} for fid, f in faults.items()}

    def register_adapter(self, adapter) -> None:
        self.adapters.append(adapter)

    def list(self) -> list[dict]:
        return [dict(s, active=(self.active is not None and self.active["id"] == s["id"])) for s in self.scenarios.values()]

    async def inject(self, fault_id: str, duration_s: float | None = None) -> dict:
        sc = self.scenarios.get(fault_id.upper())
        if not sc:
            raise KeyError(fault_id)
        if self.active:
            await self.clear()
        dur = duration_s if duration_s is not None else sc["duration_s"]
        self.active = {**sc, "started_at": time.time(), "ends_at": time.time() + dur, "duration_s": dur}
        self.store.append(make_event(EventType.FaultInjected, Source.OPERATOR, self.robot_id, {
            "scenario": sc["id"], "name": sc["name"], "description": sc["description"], "params": sc["params"],
            "duration_s": dur, "expected_detection": sc["expected_detection"], "step": sc.get("step")}))
        for a in self.adapters:
            a.apply_fault(self.active)
        self._clear_task = asyncio.create_task(self._auto_clear(dur))
        return self.active

    async def _auto_clear(self, dur: float) -> None:
        await asyncio.sleep(dur)
        await self.clear()

    async def clear(self) -> dict | None:
        if not self.active:
            return None
        sc, self.active = self.active, None
        if self._clear_task and not self._clear_task.done() and self._clear_task is not asyncio.current_task():
            self._clear_task.cancel()
        for a in self.adapters:
            a.clear_fault()
        self.store.append(make_event(EventType.FaultCleared, Source.OPERATOR, self.robot_id, {"scenario": sc["id"], "name": sc["name"]}))
        return sc
