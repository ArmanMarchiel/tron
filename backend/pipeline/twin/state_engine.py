"""State engine: folds normalised events into the digital twin.

* applies every observation to the OBSERVED side of the twin
* feeds accepted trajectories to the EXPECTED model
* recomputes divergence on every physical observation
* emits ``StateDivergenceObserved`` when divergence first crosses the *notice* threshold
  (this is what "first detected divergence" in a reconstruction refers to)
* hands the twin to the risk engine, then snapshots it (twin history / timeline)
"""
from __future__ import annotations

import copy
import math
from collections.abc import Callable

from backend.app.config import EFFORT_LIMITS, ENVIRONMENT, JOINTS, ROBOT_ID, ROBOT_MODEL, ROBOT_NAME, SAFETY, SECURITY, SNAPSHOT_HZ, SNAPSHOT_RETENTION_S
from backend.pipeline.events.event_model import Event, EventType, Source, make_event
from backend.pipeline.events.event_store import EventStore
from backend.pipeline.twin import kinematics
from backend.pipeline.twin.expected_state import ExpectedStateModel, compute_divergence
from backend.pipeline.twin.robot_model import new_robot_twin

RiskHook = Callable[[dict, Event], None]


def default_identity() -> dict:
    return {"name": ROBOT_NAME, "model": ROBOT_MODEL, "joints": list(JOINTS),
            "limits": {"effort": list(EFFORT_LIMITS), "max_joint_velocity": SAFETY["max_joint_velocity"],
                       "tracking_notice": SAFETY["joint_tracking_notice_rad"], "tracking_alert": SAFETY["joint_tracking_alert_rad"]},
            "ros2": {"command_topic": SECURITY["command_topic"]}}


def default_environment() -> dict:
    return {"scenario": None, "obstacles": ENVIRONMENT["obstacles"],
            "humans": [{"id": h["id"], "x": h["pos"][0], "y": h["pos"][1], "z": h["pos"][2]} for h in ENVIRONMENT["humans"]],
            "zones": [{"id": z["id"], "type": "restricted", "min": z["min"], "max": z["max"]} for z in ENVIRONMENT["forbidden_zones"]],
            "sensors": [], "machine": None}


class StateEngine:
    def __init__(self, store: EventStore, robot_id: str = ROBOT_ID, identity: dict | None = None, environment: dict | None = None,
                 safety: dict | None = None, security: dict | None = None):
        self.store = store
        self.robot_id = robot_id
        self.safety = dict(SAFETY, **(safety or {}))
        self.security = security or SECURITY
        self.twin = new_robot_twin(robot_id, identity or default_identity(), environment or default_environment())
        n = len(self.twin["identity"]["joints"])
        cmd_topic = self.twin["identity"].get("ros2", {}).get("command_topic", self.security["command_topic"])
        self.expected = ExpectedStateModel(self.security["expected_publishers"].get(cmd_topic, self.security["expected_publishers"][self.security["command_topic"]]), n)
        self.risk_hooks: list[RiskHook] = []
        self.shadow = None            # optional ShadowSim (predicted efforts)
        self._last_phys_ts: float | None = None
        self._last_snapshot_ts = 0.0
        self._last_prune_ts = 0.0
        self._prev_divergence_level = "none"

    def _emit(self, event_type: EventType, payload: dict, ts: float, confidence: float = 1.0) -> None:
        self.store.append(make_event(event_type, Source.PLATFORM, self.robot_id, payload, confidence, ts))

    # ------------------------------------------------------------------ main entry
    def handle(self, ev: Event) -> None:
        t = self.twin
        et, pl, ts = ev.event_type, ev.payload, ev.timestamp
        if ev.source not in t["identity"]["adapters"] and ev.source != Source.PLATFORM:
            t["identity"]["adapters"].append(ev.source)

        if et == EventType.RobotStateObserved:
            self._apply_physical(pl, ev.source, ts)
        elif et == EventType.PositionObserved:
            b = t["software_belief"]
            b["position"] = [float(x) for x in pl["position"]]
            b["velocity"] = [float(x) for x in pl.get("velocity", b["velocity"])]
            b["effort"] = [float(x) for x in pl.get("effort", b["effort"])]
            b["ee"] = kinematics.fk(b["position"])
            b["source"], b["ts"] = ev.source, ts
            t["sensors"]["encoders"] = {"position": b["position"], "ts": ts}
        elif et == EventType.SensorObservation:
            if pl.get("sensor") == "camera":
                t["sensors"]["camera"] = {"fps": pl.get("fps"), "ts": ts}
            elif pl.get("sensor") == "area_scanner":
                sc = t["sensors"]["area_scanner"]
                intr = bool(pl.get("intrusion"))
                if intr and not sc.get("intrusion"):
                    sc["since"] = ts
                if not intr:
                    sc["since"] = None
                sc.update({"intrusion": intr, "ts": ts, "id": pl.get("id")})
                cfg = next((x for x in t["environment"]["sensors"] if x.get("id") == pl.get("id")), None)
                if cfg is None or cfg.get("protective_stop", True):
                    self.expected.on_intrusion(intr, ts, (cfg or {}).get("stop_within_s", 0.5))
        elif et == EventType.EnvironmentObserved:
            if "humans" in pl:
                t["environment"]["humans"] = pl["humans"]
            if "obstacles" in pl:
                t["environment"]["obstacles"] = pl["obstacles"]
        elif et == EventType.CommandReceived:
            self._apply_command(pl, ts)
        elif et == EventType.CommandExecuted:
            t["actuators"].update({"targets": pl.get("targets", t["actuators"]["targets"]), "gripper": pl.get("gripper"), "state": pl.get("state", "active")})
            t["software"]["controller"].update({"state": pl.get("state", "active"), "progress": pl.get("progress", 0.0), "goal_name": pl.get("goal_name")})
        elif et == EventType.ControllerStateChanged:
            t["software"]["controller"].update({"name": pl.get("name", t["software"]["controller"]["name"]), "state": pl.get("state", "unknown")})
        elif et == EventType.ROSNodeStarted:
            t["software"]["nodes"][pl["node"]] = {"state": "running", "since": ts, "namespace": pl.get("namespace", "/")}
        elif et == EventType.ROSNodeStopped:
            if pl["node"] in t["software"]["nodes"]:
                t["software"]["nodes"][pl["node"]].update({"state": "stopped", "stopped_at": ts})
        elif et == EventType.ROSTopicObserved:
            t["software"]["topics"][pl["topic"]] = {"publishers": pl.get("publishers", []), "subscribers": pl.get("subscribers", []),
                                                    "rate_hz": pl.get("rate_hz"), "type": pl.get("type"), "ts": ts}
            t["network"]["message_activity"][pl["topic"]] = {"rate_hz": pl.get("rate_hz"), "count": pl.get("count")}
        elif et == EventType.NetworkConnectionObserved:
            names = {p["name"] for p in pl.get("participants", [])}
            for p in pl.get("participants", []):
                t["network"]["participants"][p["name"]] = {"guid": p.get("guid"), "address": p.get("address"), "ts": ts}
            for g in set(t["network"]["participants"]) - names:
                t["network"]["participants"].pop(g, None)
            t["network"]["connections"] = pl.get("connections", [])
        elif et == EventType.TrajectoryGoalReceived:
            t["software"]["planner"].update({"goal": {"name": pl.get("goal_name"), "positions": pl.get("positions"), "duration_s": pl.get("duration_s"),
                                                      "ee_target": pl.get("ee_target"), "source_node": pl.get("source_node")},
                                             "status": "active", "goal_received_ts": ts})
        elif et == EventType.TrajectoryGoalReached:
            t["software"]["planner"].update({"goal": None, "status": "reached"})
            self.expected.on_goal_reached(ts)
        elif et == EventType.TaskStepStarted:
            t["task"].update({"step": pl.get("step"), "index": pl.get("index"), "action": pl.get("action"), "started_ts": ts,
                              "duration_s": pl.get("duration_s"), "timeout_s": pl.get("timeout_s"), "status": "active",
                              "target": pl.get("target"), "precondition": pl.get("precondition")})
            if pl.get("cycles_target") is not None or pl.get("cycles_done") is not None:
                t["task"]["cycles_target"] = pl.get("cycles_target", t["task"]["cycles_target"])
                t["task"]["cycles_done"] = pl.get("cycles_done", t["task"]["cycles_done"])
            t["software"]["planner"]["machine_belief"] = pl.get("machine_belief")
            self.expected.on_task_step(pl, ts)
        elif et in (EventType.TaskStepCompleted, EventType.TaskStepFailed):
            t["task"]["status"] = "completed" if et == EventType.TaskStepCompleted else "failed"
            t["task"]["completed"] = (t["task"]["completed"] + [pl.get("step")])[-30:]
            if et == EventType.TaskStepFailed:
                t["task"]["last_failure"] = {"step": pl.get("step"), "reason": pl.get("reason"), "ts": ts}
            self.expected.on_task_done(ts)
        elif et == EventType.TaskCycleCompleted:
            t["task"]["cycles_done"] = pl.get("cycle", t["task"]["cycles_done"])
            t["task"]["cycles_target"] = pl.get("cycles_target")
        elif et == EventType.TaskRunCompleted:
            t["task"].update({"run_done": True, "status": "done", "step": None,
                              "cycles_done": pl.get("cycles_completed", t["task"]["cycles_done"]),
                              "cycles_target": pl.get("cycles_target"), "finished_ts": ts})
        elif et == EventType.MachineStateObserved:
            if t["machine"] is None:
                t["machine"] = {"id": pl.get("machine_id"), "last_command": None, "last_interlock": None, "allowed_clients": []}
            t["machine"].update({k: pl.get(k) for k in ("state", "door", "chuck", "alarm", "cycle_count", "part_loaded", "cycle_progress")})
            t["machine"]["ts"] = ts
        elif et == EventType.MachineCommandReceived:
            cmd = {"command": pl.get("command"), "client": pl.get("client"), "accepted": pl.get("accepted"), "reason": pl.get("reason"), "ts": ts}
            if t["machine"] is not None:
                t["machine"]["last_command"] = cmd
            t["software"]["last_machine_command"] = cmd
        elif et == EventType.InterlockViolated:
            if t["machine"] is not None:
                t["machine"]["last_interlock"] = {"command": pl.get("command"), "client": pl.get("client"), "reason": pl.get("reason"), "ts": ts}
        elif et == EventType.CollisionDetected:
            t["physical"]["collision"] = True
            t["physical"]["collision_with"] = pl.get("with")
            t["physical"]["collision_event_ts"] = ts
        elif et == EventType.IncidentCreated:
            if pl["incident_id"] not in t["incidents"]["active"]:
                t["incidents"]["active"].append(pl["incident_id"])
        elif et == EventType.IncidentClosed:
            if pl["incident_id"] in t["incidents"]["active"]:
                t["incidents"]["active"].remove(pl["incident_id"])

        t["ts"] = ts
        if et == EventType.RobotStateObserved:
            self._after_physical(ev)

    # ------------------------------------------------------------------ appliers
    def _apply_physical(self, pl: dict, source: str, ts: float) -> None:
        p = self.twin["physical"]
        p["position"] = [float(x) for x in pl["position"]]
        p["velocity"] = [float(x) for x in pl.get("velocity", p["velocity"])]
        p["effort"] = [float(x) for x in pl.get("effort", p["effort"])]
        p["ee"] = pl.get("ee", p["ee"])
        p["ee_speed"] = float(pl.get("ee_speed", 0.0))
        p["gripper"] = pl.get("gripper")
        if "collision" in pl:
            p["collision"] = bool(pl["collision"])
            if pl.get("collision_with"):
                p["collision_with"] = pl["collision_with"]
        elif p.get("collision") and ts - p.get("collision_event_ts", ts) > 1.0:
            p["collision"] = False
        p["protective_stop"] = bool(pl.get("protective_stop", False))
        p["human_in_scanner_field"] = bool(pl.get("human_in_field", False))
        p["payload_kg"] = pl.get("payload_kg", p.get("payload_kg"))
        for k in ("protective_stop_source", "human_in_reach_envelope"):
            if k in pl:
                p[k] = pl[k]
        p["in_zones"] = self._zones_containing(p["ee"]) if p["ee"] else []
        p["in_forbidden_zone"] = next((z for z in p["in_zones"] if self._zone_type(z).startswith("restricted") and self._zone_active(z)), None)
        env = self.twin["environment"]
        env["nearest_obstacle_distance"] = pl.get("nearest_obstacle_distance", env["nearest_obstacle_distance"])
        env["nearest_human_distance"] = pl.get("nearest_human_distance", env["nearest_human_distance"])
        p["source"], p["ts"] = source, ts
        moving = max(abs(v) for v in p["velocity"]) > 0.02 if p["velocity"] else False
        self.twin["status"] = "operational" if moving or self.twin["software"]["planner"]["status"] == "active" else "idle"

    def _zones_containing(self, ee: dict) -> list[str]:
        """Zone ids whose volume holds this point.  Most zones are axis-aligned boxes; a
        reach_envelope is a cylinder about the robot base, so it has a radius instead of min/max."""
        out = []
        for z in self.twin["environment"]["zones"]:
            if z.get("type") == "reach_envelope":
                r, h = z.get("radius"), z.get("height")
                if r and math.hypot(ee["x"], ee["y"]) <= r and (h is None or 0.0 <= ee["z"] <= h):
                    out.append(z["id"])
                continue
            lo, hi = z.get("min") or [], z.get("max") or []
            if len(lo) < 3 or len(hi) < 3:
                continue
            if lo[0] <= ee["x"] <= hi[0] and lo[1] <= ee["y"] <= hi[1] and lo[2] <= ee["z"] <= hi[2]:
                out.append(z["id"])
        return out

    def _zone(self, zid: str) -> dict | None:
        return next((z for z in self.twin["environment"]["zones"] if z["id"] == zid), None)

    def _zone_type(self, zid: str) -> str:
        z = self._zone(zid)
        return z["type"] if z else ""

    def _zone_active(self, zid: str) -> bool:
        """restricted_while zones apply only when their condition (on the reported machine state) holds."""
        z = self._zone(zid)
        if not z or z["type"] != "restricted_while":
            return True
        cond = (z.get("condition") or "").replace(" ", "")
        m = self.twin.get("machine") or {}
        if cond.startswith("machine.") and "==" in cond:
            key, val = cond[len("machine."):].split("==")
            return str(m.get(key)) == val
        return True

    def _apply_command(self, pl: dict, ts: float) -> None:
        sw = self.twin["software"]
        cmd = {"topic": pl.get("topic", SECURITY["command_topic"]), "source_node": pl.get("source_node"), "goal_name": pl.get("goal_name"),
               "positions": pl.get("positions"), "duration_s": pl.get("duration_s"), "start_positions": pl.get("start_positions"), "ts": ts}
        cmd["accepted_as_expected"] = self.expected.on_command(pl, cmd["source_node"], ts)
        sw["last_command"] = cmd
        sw["controller"]["last_command"] = cmd

    # ------------------------------------------------------------------ post-physical
    def _after_physical(self, ev: Event) -> None:
        ts = ev.timestamp
        t = self.twin
        self.expected.advance(ts)
        if t["physical"].get("protective_stop") and self.expected.task and self._last_phys_ts is not None:
            self.expected.task["window_end"] += max(0.0, ts - self._last_phys_ts)   # the clock stops while safely stopped
        self._last_phys_ts = ts
        self.expected.reanchor_if_idle(ts, t["physical"]["position"])
        t["expected"] = self.expected.state()
        t["expected"]["ee"] = kinematics.fk(t["expected"]["position"])
        t["expected"]["effort"] = self.shadow.predict(t) if self.shadow else None
        t["divergence"] = compute_divergence(t, self.safety)
        # torque-stall tracking: saturated effort while not tracking the target (a blocked joint), per joint
        limits = t["identity"].get("limits", {}).get("effort") or EFFORT_LIMITS
        stall = t["physical"].get("torque_stall_since") or [None] * len(limits)
        for i, (tau, lim) in enumerate(zip(t["physical"]["effort"], limits)):
            over = abs(tau) >= lim * self.safety["effort_alert_ratio"] and abs(t["divergence"]["joint_errors"][i]) >= self.safety.get("effort_stall_error_rad", 0.08)
            stall[i] = (stall[i] if stall[i] is not None else ts) if over else None
        t["physical"]["torque_stall_since"] = stall
        resid = t["divergence"].get("torque_residual")
        if resid is not None:
            thr = self.safety.get("torque_residual_nm", [])
            rs = t["physical"].get("torque_residual_since") or [None] * len(resid)
            for i, r in enumerate(resid):
                over = abs(r) > (thr[i] if i < len(thr) else 10.0)
                rs[i] = (rs[i] if rs[i] is not None else ts) if over else None
            t["physical"]["torque_residual_since"] = rs
        # encoder disagreement persistence (filters sampling latency during fast motion)
        if t["divergence"]["encoder_max_abs"] > self.safety["encoder_divergence_rad"]:
            t["physical"]["encoder_over_since"] = t["physical"].get("encoder_over_since") or ts
        else:
            t["physical"]["encoder_over_since"] = None
        # human-in-field persistence (for the scanner divergence rule)
        if t["physical"].get("human_in_scanner_field"):
            t["physical"]["human_in_field_since"] = t["physical"].get("human_in_field_since") or ts
        else:
            t["physical"]["human_in_field_since"] = None

        level = t["divergence"]["level"]
        if level != "none" and self._prev_divergence_level == "none":
            self._emit(EventType.StateDivergenceObserved, {
                "level": level, "divergence": t["divergence"],
                "expected": {"position": t["expected"]["position"], "velocity": t["expected"]["velocity"], "ee": t["expected"]["ee"]},
                "actual": {"position": t["physical"]["position"], "velocity": t["physical"]["velocity"], "ee": t["physical"]["ee"]},
            }, ts)
        self._prev_divergence_level = level

        for hook in self.risk_hooks:
            hook(t, ev)

        if ts - self._last_snapshot_ts >= 0.9 / SNAPSHOT_HZ:
            self._last_snapshot_ts = ts
            self.store.save_snapshot(self.robot_id, ts, self.snapshot())
        if ts - self._last_prune_ts > 60:
            self._last_prune_ts = ts
            with self.store._lock:
                self.store._conn.execute("DELETE FROM snapshots WHERE timestamp < ?", (ts - SNAPSHOT_RETENTION_S,))
                self.store._conn.commit()

    def snapshot(self) -> dict:
        return copy.deepcopy(self.twin)
