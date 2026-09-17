"""Runs safety + security rules against the twin and turns rule transitions into events.

* inactive -> active : emit AnomalyDetected / SafetyThresholdExceeded (with evidence)
* active   -> active : re-emit at most every ``repeat_s`` seconds (keeps evidence flowing to incidents)
* active   -> clear  : emit AnomalyDetected with ``cleared: true``
Also maintains ``twin["risk"]`` (level, score, active rules).
"""
from __future__ import annotations

from backend.app.config import SAFETY, SECURITY
from backend.pipeline.events.event_model import Event, EventType, Source, make_event
from backend.pipeline.events.event_store import EventStore
from backend.pipeline.risk.safety_rules import SAFETY_RULES, SEVERITY_ORDER, Hit
from backend.pipeline.risk.security_rules import SECURITY_RULES, unexpected_controller_command
from backend.pipeline.risk.task_rules import TASK_RULES
from backend.pipeline.risk.zone_rules import ZONE_RULES


class AnomalyEngine:
    def __init__(self, store: EventStore, repeat_s: float = 2.0, safety: dict | None = None, security: dict | None = None):
        self.store = store
        self.repeat_s = repeat_s
        self.safety = dict(SAFETY, **(safety or {}))
        self.security = security or SECURITY
        self._active: dict[str, dict] = {}   # rule -> {since, last_emit, hit}
        self._pstop_reported = False

    def evaluate(self, twin: dict, trigger: Event) -> None:
        ts = trigger.timestamp
        hits: list[Hit] = []
        for rule in SAFETY_RULES + TASK_RULES + ZONE_RULES:
            h = rule(twin, self.safety)
            if h:
                hits.append(h)
        for rule in SECURITY_RULES:
            h = rule(twin, self.security)
            if h:
                hits.append(h)
        h = unexpected_controller_command(twin, self.security, self.safety)
        if h:
            hits.append(h)

        # dedupe by rule name keeping the highest severity
        by_rule: dict[str, Hit] = {}
        for h in hits:
            if h.rule not in by_rule or SEVERITY_ORDER[h.severity] > SEVERITY_ORDER[by_rule[h.rule].severity]:
                by_rule[h.rule] = h

        # transitions
        for rule, h in by_rule.items():
            st = self._active.get(rule)
            if st is None:
                self._active[rule] = {"since": ts, "last_emit": ts, "hit": h}
                self._emit(h, ts, twin, first=True)
            elif ts - st["last_emit"] >= self.repeat_s:
                st["last_emit"] = ts
                st["hit"] = h
                self._emit(h, ts, twin, first=False)
        for rule in list(self._active):
            if rule not in by_rule:
                st = self._active.pop(rule)
                self.store.append(make_event(EventType.AnomalyDetected, Source.PLATFORM, twin["identity"]["robot_id"], {
                    "rule": rule, "severity": st["hit"].severity, "category": st["hit"].category,
                    "message": f"{rule} cleared", "cleared": True, "active_since": st["since"]}, 1.0, ts))

        # informational: a protective stop that followed a sensed intrusion (the safety function worked)
        if twin["physical"].get("protective_stop") and not self._pstop_reported:
            self._pstop_reported = True
            self.store.append(make_event(EventType.SafetyThresholdExceeded, Source.PLATFORM, twin["identity"]["robot_id"], {
                "rule": "PROTECTIVE_STOP_OBSERVED", "severity": "LOW", "category": "safety", "informational": True, "first": True,
                "message": "Controller issued a protective stop after a sensed scanner intrusion",
                "evidence": {"scanner": twin["sensors"].get("area_scanner"), "controller_state": twin["software"]["controller"].get("state")}}, 1.0, ts))
        if not twin["physical"].get("protective_stop"):
            self._pstop_reported = False
        # risk summary on the twin
        active = [{"rule": r, "severity": s["hit"].severity, "category": s["hit"].category,
                   "message": s["hit"].message, "since": s["since"]} for r, s in self._active.items()]
        score = sum(SEVERITY_ORDER[a["severity"]] * 25 for a in active)
        level = "NONE"
        for a in active:
            if SEVERITY_ORDER[a["severity"]] > SEVERITY_ORDER[level]:
                level = a["severity"]
        twin["risk"] = {"level": level, "score": min(100, score), "active_rules": active}

    def _emit(self, h: Hit, ts: float, twin: dict, first: bool) -> None:
        self.store.append(make_event(EventType(h.event_type), Source.PLATFORM, twin["identity"]["robot_id"], {
            "rule": h.rule, "severity": h.severity, "category": h.category, "message": h.message,
            "evidence": h.evidence, "first": first, "cleared": False,
            "expected": {"position": twin["expected"]["position"], "velocity": twin["expected"]["velocity"], "ee": twin["expected"]["ee"],
                         "goal": twin["expected"]["goal"]},
            "actual": {"position": twin["physical"]["position"], "velocity": twin["physical"]["velocity"], "ee": twin["physical"]["ee"]},
            "divergence": twin["divergence"],
        }, 1.0, ts))

    def active_rules(self) -> list[str]:
        return list(self._active)
