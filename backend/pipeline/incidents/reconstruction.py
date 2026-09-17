"""Incident reconstruction: rebuild the causal timeline and explain it.

Given an incident we look back ``lookback_s`` before its first anomaly and collect everything
that could have contributed: trajectory goals, commands, ROS graph changes, motion changes,
divergence notices, anomalies, contacts.  From those we produce an ordered timeline, the
EXPECTED behaviour at the first divergence, the ACTUAL narrative, the first detected divergence
and root-cause *candidates* labelled Observed / Inferred / Possible cause with evidence ids.
We never claim a root cause; we rank candidates, and candidates whose evidence begins after
the first anomaly are demoted as consequences.
"""
from __future__ import annotations

import math

from backend.app.config import ENVIRONMENT, INCIDENTS, JOINTS, SAFETY, SECURITY
from backend.pipeline.events.event_model import Event, EventType
from backend.pipeline.events.event_store import EventStore
from backend.pipeline.twin import kinematics
from backend.pipeline.incidents.timeline import state_at

OPERATOR_TYPES = {EventType.FaultInjected, EventType.FaultCleared}


def _fmt_ee(p: dict | None) -> str:
    return f"({p['x']:.2f}, {p['y']:.2f}, {p['z']:.2f})" if p else "n/a"


def _allowed() -> list[str]:
    return SECURITY["expected_publishers"].get(SECURITY["command_topic"], [])


def _timeline(store: EventStore, inc: dict, since: float, until: float, include_operator: bool) -> tuple[list[dict], list[Event]]:
    evs = store.query(since=since, until=until, limit=20000, entity_id=inc["entity_id"])
    out: list[dict] = []
    kept: list[Event] = []
    last_cmd: dict[str, str] = {}
    last_vel: float | None = None
    last_err: float | None = None
    last_collision_ts: float | None = None
    last_ctrl_state: str | None = None
    last_machine: str | None = None
    last_scanner = None
    last_in_field = None
    for e in evs:
        et, pl = e.event_type, e.payload
        entry = None
        if et in OPERATOR_TYPES:
            if include_operator:
                entry = {"kind": "operator", "text": f"Fault scenario {'injected' if et == EventType.FaultInjected else 'cleared'}: {pl.get('scenario')} — {pl.get('description', '')}"}
        elif et == EventType.TrajectoryGoalReceived:
            entry = {"kind": "command", "text": f"Planner goal '{pl.get('goal_name')}' ({pl.get('duration_s', 0):.1f} s) → EE target {_fmt_ee({'x': pl['ee_target'][0], 'y': pl['ee_target'][1], 'z': pl['ee_target'][2]}) if pl.get('ee_target') else 'n/a'}"}
        elif et == EventType.TrajectoryGoalReached:
            entry = {"kind": "command", "text": f"Trajectory '{pl.get('goal_name')}' completed (final error {pl.get('final_error_rad', 0):.3f} rad)"}
        elif et == EventType.CommandReceived:
            src = pl.get("source_node") or "unknown"
            key = f"{pl.get('goal_name')}|{pl.get('duration_s')}"
            if last_cmd.get(src) != key:
                tag = "" if src in _allowed() else "  [UNAUTHORISED SOURCE]"
                entry = {"kind": "command", "text": f"Joint trajectory published by {src}: '{pl.get('goal_name')}' over {pl.get('duration_s', 0):.1f} s{tag}",
                         "unauthorised": src not in _allowed()}
                last_cmd[src] = key
        elif et == EventType.CommandExecuted:
            if pl.get("state") != last_ctrl_state:
                entry = {"kind": "controller", "text": f"Controller {pl.get('state')}" + (f" on '{pl.get('goal_name')}'" if pl.get("goal_name") else "")}
                last_ctrl_state = pl.get("state")
        elif et == EventType.RobotStateObserved:
            vel = max((abs(v) for v in pl.get("velocity", [])), default=0.0)
            if pl.get("human_in_field") != last_in_field and last_in_field is not None:
                out.append({"kind": "observation", "text": f"Ground truth: a person is {'inside' if pl.get('human_in_field') else 'no longer inside'} the scanner field",
                            "ts": e.timestamp, "event_id": e.event_id, "event_type": str(et), "source": e.source})
                kept.append(e)
            last_in_field = pl.get("human_in_field")
            if pl.get("protective_stop") and last_ctrl_state != "pstop":
                out.append({"kind": "controller", "text": "Controller in PROTECTIVE STOP", "ts": e.timestamp, "event_id": e.event_id, "event_type": str(et), "source": e.source})
                kept.append(e); last_ctrl_state = "pstop"
            if last_vel is None or abs(vel - last_vel) > 0.25:
                i = max(range(len(pl["velocity"])), key=lambda k: abs(pl["velocity"][k]))
                entry = {"kind": "observation", "text": f"Actual: peak joint speed {vel:.2f} rad/s ({JOINTS[i]}), EE {_fmt_ee(pl.get('ee'))}, obstacle {pl.get('nearest_obstacle_distance', 0):.2f} m, human {pl.get('nearest_human_distance', 0):.2f} m"}
                last_vel = vel
        elif et == EventType.TaskStepStarted:
            mb = pl.get("machine_belief") or {}
            entry = {"kind": "task", "text": f"Task step '{pl.get('step')}' started ({pl.get('action')}"
                     + (f" → {pl.get('target')}" if pl.get("target") else "") + (f", machine cmd {pl.get('command')}" if pl.get("command") else "") + ")"
                     + (f"; planner believes machine {mb.get('state')}" if mb.get("state") else "")}
        elif et == EventType.TaskStepFailed:
            entry = {"kind": "task", "text": f"Task step '{pl.get('step')}' FAILED after {pl.get('elapsed_s', 0):.1f} s: {pl.get('reason')}", "severity": "MEDIUM"}
        elif et == EventType.MachineStateObserved:
            key = f"{pl.get('state')}|{pl.get('door')}|{pl.get('chuck')}|{pl.get('alarm')}"
            if key != last_machine:
                entry = {"kind": "machine", "text": f"Machine {pl.get('machine_id')} reports state {pl.get('state')}, door {pl.get('door')}, chuck {pl.get('chuck')}"
                         + (f", ALARM {pl.get('alarm')}" if pl.get("alarm") else "")}
                last_machine = key
        elif et == EventType.MachineCommandReceived:
            entry = {"kind": "machine", "text": f"Machine command '{pl.get('command')}' from {pl.get('client')}: {'accepted' if pl.get('accepted') else 'REJECTED — ' + str(pl.get('reason'))}",
                     "unauthorised": False}
        elif et == EventType.InterlockViolated:
            entry = {"kind": "anomaly", "text": f"Interlock: machine rejected '{pl.get('command')}' from {pl.get('client')} ({pl.get('reason')})", "severity": "HIGH"}
        elif et == EventType.SensorObservation and pl.get("sensor") == "area_scanner":
            if pl.get("intrusion") != last_scanner:
                entry = {"kind": "observation", "text": f"Area scanner {pl.get('id')} reports {'INTRUSION' if pl.get('intrusion') else 'field clear'}"}
                last_scanner = pl.get("intrusion")
        elif et == EventType.ROSNodeStarted:
            entry = {"kind": "ros", "text": f"ROS node started: {pl.get('node')}"}
        elif et == EventType.ROSNodeStopped:
            entry = {"kind": "ros", "text": f"ROS node stopped: {pl.get('node')}"}
        elif et == EventType.ROSTopicObserved:
            allowed = SECURITY["expected_publishers"].get(pl.get("topic"))
            extra = [p for p in pl.get("publishers", []) if allowed is not None and p not in allowed]
            if extra:
                entry = {"kind": "ros", "text": f"New publisher on {pl.get('topic')}: {', '.join(extra)}"}
        elif et == EventType.StateDivergenceObserved:
            d = pl.get("divergence", {})
            entry = {"kind": "divergence", "text": f"Expected ≠ actual ({pl.get('level')}): tracking error {d.get('joint_max_abs', 0):.3f} rad on {JOINTS[d.get('joint_max_index', 0)]}, "
                                                    f"velocity Δ {d.get('velocity_max_abs', 0):.2f} rad/s" + (f", EE {d['ee_m']:.3f} m" if d.get("ee_m") is not None else "")}
        elif et in (EventType.AnomalyDetected, EventType.SafetyThresholdExceeded):
            if pl.get("cleared"):
                entry = {"kind": "cleared", "text": f"{pl.get('rule')} cleared"}
            elif pl.get("first"):
                entry = {"kind": "anomaly", "text": f"{pl.get('rule')}: {pl.get('message')}", "severity": pl.get("severity")}
        elif et == EventType.CollisionDetected:
            if last_collision_ts is None or e.timestamp - last_collision_ts > 1.0:
                entry = {"kind": "collision", "text": f"Contact: {pl.get('arm_link') or 'arm'} with {pl.get('with', 'unknown')}" + (f" during task step '{pl['task_step']}'" if pl.get("task_step") else "") + (f" at EE {_fmt_ee({'x': pl['ee'][0], 'y': pl['ee'][1], 'z': pl['ee'][2]})}" if pl.get('ee') else ''), "severity": "CRITICAL"}
            last_collision_ts = e.timestamp
        elif et == EventType.IncidentCreated:
            entry = {"kind": "incident", "text": f"Incident {pl.get('incident_id')} opened: {pl.get('title')}"}
        elif et == EventType.IncidentClosed:
            entry = {"kind": "incident", "text": f"Incident {pl.get('incident_id')} closed"}
        if entry:
            entry.update({"ts": e.timestamp, "event_id": e.event_id, "event_type": str(et), "source": e.source})
            out.append(entry)
            kept.append(e)
    seen, dedup = set(), []
    for x in out:
        k = (x["kind"], x["text"]) if x["kind"] == "ros" else None
        if k and k in seen:
            continue
        if k:
            seen.add(k)
        dedup.append(x)
    return dedup, kept


def _candidates(store: EventStore, inc: dict, since: float, until: float, anomalies: list[Event]) -> list[dict]:
    rules = {a.payload.get("rule") for a in anomalies}
    cands: list[dict] = []
    evs = store.query(since=since, until=until, limit=20000, entity_id=inc["entity_id"])
    allowed = _allowed()

    unauth_cmds = [e for e in evs if e.event_type == EventType.CommandReceived and e.payload.get("source_node") and e.payload["source_node"] not in allowed]
    new_nodes = [e for e in evs if e.event_type == EventType.ROSNodeStarted and e.payload.get("node") not in SECURITY["expected_nodes"]]
    if unauth_cmds:
        srcs = sorted({e.payload["source_node"] for e in unauth_cmds})
        cands.append({"label": "Observed", "rank": 1, "title": f"Joint trajectories from unauthorised node {', '.join(srcs)}",
                      "detail": f"{len(unauth_cmds)} trajectory messages were received from a publisher that is not on the allow-list. "
                                f"The controller executes the last trajectory it receives, so these commands directly moved the arm.",
                      "evidence": [e.event_id for e in unauth_cmds[:5]] + [e.event_id for e in new_nodes[:2]]})

    vel = [a for a in anomalies if a.payload.get("rule") == "JOINT_VELOCITY_LIMIT_EXCEEDED"]
    if vel and not unauth_cmds:
        ev0 = vel[0].payload.get("evidence", {})
        tr = ev0.get("trajectory") or {}
        cands.append({"label": "Inferred", "rank": 1, "title": "Controller executed the trajectory faster than commanded",
                      "detail": f"The authorised trajectory '{tr.get('name')}' was commanded over {tr.get('duration_s', 0):.1f} s (expected peak "
                                f"{abs(ev0.get('expected_velocity', 0)):.2f} rad/s on {ev0.get('joint')}) but the joint physically moved at "
                                f"{ev0.get('observed_velocity', 0):.2f} rad/s. The command stream is clean, so the timing changed between the "
                                f"trajectory message and the actuators (controller interpolation / time scaling).",
                      "evidence": [vel[0].event_id]})

    sens = [a for a in anomalies if a.payload.get("rule") == "SENSOR_STATE_DIVERGENCE"]
    if sens:
        ev0 = sens[0].payload.get("evidence", {})
        cands.append({"label": "Observed", "rank": 1 if not cands else 2, "title": f"{ev0.get('joint')} encoder disagrees with the physical joint angle",
                      "detail": f"/joint_states reported {ev0.get('reported', 0):.3f} rad while the joint was physically at {ev0.get('physical', 0):.3f} rad "
                                f"(Δ {ev0.get('difference_rad', 0):+.3f} rad). Every consumer of /joint_states (planner, TF, safety checks) acted on a wrong "
                                f"arm configuration; the reported end-effector was {_fmt_ee(ev0.get('reported_ee'))} vs physical {_fmt_ee(ev0.get('physical_ee'))}.",
                      "evidence": [a.event_id for a in sens[:3]]})

    traj = [a for a in anomalies if a.payload.get("rule") == "TRAJECTORY_DIVERGENCE"]
    coll = [a for a in anomalies if a.payload.get("rule") == "COLLISION_EVENT" or a.event_type == EventType.CollisionDetected]
    if traj and not unauth_cmds and not vel and not (coll and coll[0].timestamp < traj[0].timestamp):
        ev0 = traj[0].payload.get("evidence", {})
        errs = ev0.get("joint_errors", [])
        worst = sorted(range(len(errs)), key=lambda i: -abs(errs[i]))[:2]
        cands.append({"label": "Inferred", "rank": 1 if not cands else 2, "title": f"Actuator under-performance or external load on {', '.join(JOINTS[i] for i in worst)}",
                      "detail": f"The commanded trajectory was valid and came from the authorised planner, yet {', '.join(f'{JOINTS[i]} lagged its target by {errs[i]:+.3f} rad' for i in worst)}. "
                                f"Joints that carry the arm's weight are the ones that drifted, which points at lost actuator torque/gain or an unexpected external load rather than a software cause.",
                      "evidence": [a.event_id for a in traj[:3]]})

    if coll:
        t_coll = coll[0].timestamp
        for g in [e for e in evs if e.event_type == EventType.TrajectoryGoalReceived and e.timestamp <= t_coll]:
            ee = kinematics.fk(g.payload.get("positions")) or (g.payload.get("ee_target") and {"x": g.payload["ee_target"][0], "y": g.payload["ee_target"][1], "z": g.payload["ee_target"][2]})
            if not ee:
                continue
            for o in ENVIRONMENT["obstacles"]:
                inside = all(abs(ee[k] - o["pos"][i]) <= o["size"][i] + 0.06 for i, k in enumerate("xyz"))
                if inside:
                    cands.append({"label": "Observed", "rank": 1, "title": f"Planner goal '{g.payload.get('goal_name')}' lies inside {o['id']}",
                                  "detail": f"The trajectory target places the end-effector at {_fmt_ee(ee)}, within the volume of {o['id']} centred at "
                                            f"({o['pos'][0]:.2f}, {o['pos'][1]:.2f}, {o['pos'][2]:.2f}). Reaching it requires driving through the obstacle; no collision check rejected it.",
                                  "evidence": [g.event_id]})
                    break
        close = [e for e in evs if e.event_type == EventType.RobotStateObserved and e.payload.get("nearest_obstacle_distance") is not None
                 and e.payload["nearest_obstacle_distance"] < 0.05 and t_coll - 3.0 <= e.timestamp <= t_coll]
        active_cmds = [e for e in evs if e.event_type == EventType.CommandExecuted and e.payload.get("state") == "active"
                       and close and close[0].timestamp <= e.timestamp <= t_coll]
        if close and active_cmds:
            cands.append({"label": "Observed", "rank": 2, "title": "Controller kept executing the trajectory while the arm closed on the obstacle",
                          "detail": f"Ground truth showed the arm within {close[0].payload['nearest_obstacle_distance']:.3f} m of the obstacle "
                                    f"{t_coll - close[0].timestamp:.1f} s before contact, yet the controller reported an active trajectory in {len(active_cmds)} "
                                    f"state messages until contact. No stop or re-plan occurred.",
                          "evidence": [close[0].event_id] + [e.event_id for e in active_cmds[:3]]})
        torque = [a for a in anomalies if a.payload.get("rule") == "TORQUE_LIMIT_EXCEEDED"]
        cands.append({"label": "Observed", "rank": 3, "title": "Arm contacted an obstacle",
                      "detail": "Physical contact confirmed by the simulator contact report" + (" and by joint torque saturation while pushing against it" if torque else "") + ".",
                      "evidence": [a.event_id for a in coll[:2]] + [a.event_id for a in torque[:1]]})

    mism = [a for a in anomalies if a.payload.get("rule") == "MACHINE_STATE_MISMATCH"]
    inter = [a for a in anomalies if a.payload.get("rule") == "INTERLOCK_VIOLATION"]
    if mism:
        ev0 = mism[0].payload.get("evidence", {})
        cands.append({"label": "Observed", "rank": 1, "title": "Planner acted on a machine state that differs from the machine's own report",
                      "detail": f"When step '{ev0.get('task_step')}' started, the planner's copy of the machine state said {(ev0.get('planner_belief') or {}).get('state')} "
                                f"while the machine reported {(ev0.get('machine_report') or {}).get('state')}. The planner's machine-state feed, not the machine, is the "
                                f"inconsistent element; every downstream action (door, entry into the interior) followed from it.",
                      "evidence": [a.event_id for a in mism[:2]] + [a.event_id for a in inter[:2]]})
    elif inter:
        ev0 = inter[0].payload.get("evidence", {})
        cands.append({"label": "Observed", "rank": 1 if not cands else 2, "title": f"Machine interlock: {ev0.get('reason') or ev0.get('alarm')}",
                      "detail": f"The machine rejected or alarmed on '{ev0.get('command')}' from {ev0.get('client')} while in state {ev0.get('state')}.",
                      "evidence": [a.event_id for a in inter[:3]]})

    scan_miss = [a for a in anomalies if a.payload.get("rule") == "SENSOR_STATE_DIVERGENCE" and (a.payload.get("evidence") or {}).get("sensor")]
    if scan_miss:
        ev0 = scan_miss[0].payload.get("evidence", {})
        cands.append({"label": "Observed", "rank": 1, "title": f"Area scanner {ev0.get('sensor')} failed to report a person in its field",
                      "detail": "Ground truth shows a person inside the scanner field while the scanner reported it clear. With no sensed intrusion the safety PLC "
                                "had no reason to issue a protective stop; the sensing chain, not the controller, is the failed element.",
                      "evidence": [a.event_id for a in scan_miss[:3]]})

    pstop = [a for a in anomalies if a.payload.get("rule") == "PROTECTIVE_STOP_NOT_ISSUED"]
    if pstop:
        ev0 = pstop[0].payload.get("evidence", {})
        cands.append({"label": "Observed", "rank": 1, "title": "Scanner intrusion was sensed but the controller kept moving",
                      "detail": f"The scanner reported an intrusion at {ev0.get('sensed_since')} and a stop was required by {ev0.get('stop_required_by')}; the controller "
                                f"state stayed '{ev0.get('controller_state')}'. The safety function between scanner and controller did not act.",
                      "evidence": [a.event_id for a in pstop[:3]]})

    ssm = [a for a in anomalies if a.payload.get("rule") in ("SSM_VIOLATION", "REDUCED_SPEED_ZONE_VIOLATION")]
    if ssm and not unauth_cmds and not cands:
        ev0 = ssm[0].payload.get("evidence", {})
        cands.append({"label": "Possible cause", "rank": 2, "title": "Motion planned without regard to the operator's position",
                      "detail": f"Separation/speed rule '{ssm[0].payload.get('rule')}' fired (TCP {ev0.get('tcp_speed', 0):.2f} m/s, separation {ev0.get('separation', ev0.get('tcp_speed', 0)):.2f} m) "
                                f"with an authorised trajectory: the planner did not reduce speed or stop for the operator.",
                      "evidence": [a.event_id for a in ssm[:3]]})

    load = [a for a in anomalies if a.payload.get("rule") == "UNEXPECTED_LOAD"]
    if load and not coll:
        ev0 = load[0].payload.get("evidence", {})
        cands.append({"label": "Inferred", "rank": 1 if not cands else 2, "title": f"Unmodelled load or contact on {ev0.get('joint')}",
                      "detail": f"Observed torque {ev0.get('observed_effort', 0):.1f} N·m vs shadow-model prediction {ev0.get('predicted_effort', 0):.1f} N·m "
                                f"(residual {ev0.get('residual_nm', 0):+.1f}, payload reported {ev0.get('payload_kg')} kg). The trajectory and commands were valid; "
                                f"the physical arm is carrying or pushing against something the software does not know about.",
                      "evidence": [a.event_id for a in load[:3]]})

    tout = [a for a in anomalies if a.payload.get("rule") == "TASK_STEP_TIMEOUT"]
    if tout and not cands:
        ev0 = tout[0].payload.get("evidence", {})
        cands.append({"label": "Observed", "rank": 2, "title": f"Task step '{ev0.get('step')}' did not complete in its window",
                      "detail": f"Step '{ev0.get('step')}' ({ev0.get('action')}) ran past {ev0.get('window_s', 0):.0f} s; machine state at the time: {(ev0.get('machine') or {}).get('state')}.",
                      "evidence": [a.event_id for a in tout[:2]]})

    if new_nodes and not unauth_cmds:
        cands.append({"label": "Observed", "rank": len(cands) + 1, "title": f"Unexpected ROS node(s): {', '.join(sorted({e.payload['node'] for e in new_nodes}))}",
                      "detail": "A node outside the expected graph appeared shortly before the incident.", "evidence": [e.event_id for e in new_nodes[:3]]})

    if not cands:
        cands.append({"label": "Possible cause", "rank": 1, "title": "Insufficient evidence to rank a cause",
                      "detail": f"Rules fired: {', '.join(sorted(r for r in rules if r))}. No command, graph or sensor evidence explains the divergence.",
                      "evidence": [a.event_id for a in anomalies[:3]]})

    first_anomaly_ts = min((a.timestamp for a in anomalies), default=None)
    for c in cands:
        ts_list = [e.timestamp for e in (store.get(i) for i in c["evidence"]) if e is not None]
        t0 = min(ts_list) if ts_list else None
        c["first_evidence_ts"] = t0
        c["timing"] = "follows" if (t0 is not None and first_anomaly_ts is not None and t0 > first_anomaly_ts + 0.05) else "precedes"
        if c["timing"] == "follows":
            c["detail"] = "Began after the first anomaly, so more likely a consequence than a cause. " + c["detail"]
    cands.sort(key=lambda c: (c["timing"] == "follows", c["rank"], c["first_evidence_ts"] or 0))
    for i, c in enumerate(cands, 1):
        c["rank"] = i
    return cands


def reconstruct(store: EventStore, inc: dict, include_operator: bool = False) -> dict:
    since = inc["created_at"] - INCIDENTS["lookback_s"]
    until = (inc.get("closed_at") or inc["updated_at"]) + 1.0
    timeline, kept = _timeline(store, inc, since, until, include_operator)
    anomalies = [e for e in store.query(since=since, until=until, entity_id=inc["entity_id"], limit=5000,
                                        types=[EventType.AnomalyDetected, EventType.SafetyThresholdExceeded, EventType.CollisionDetected])
                 if not e.payload.get("cleared")]
    first_anoms = [a for a in anomalies if a.payload.get("first") or a.event_type == EventType.CollisionDetected]
    divs = store.query(since=since, until=inc["created_at"] + 0.01, entity_id=inc["entity_id"], limit=50, types=[EventType.StateDivergenceObserved])
    first_div = divs[0] if divs else (first_anoms[0] if first_anoms else None)

    at = state_at(store, inc["entity_id"], first_div.timestamp if first_div else inc["created_at"], 0.0)
    exp_txt = ""
    if at:
        tw = at["twin"]
        e, pl = tw["expected"], tw["software"]["planner"]
        tr = e.get("trajectory")
        if tr:
            ee_t = kinematics.fk(tr["to"])
            exp_txt = (f"Robot should follow trajectory '{tr.get('name')}' from {tw['software']['controller']['name']} over {tr['duration_s']:.1f} s "
                       f"(progress {e.get('progress', 0) * 100:.0f}%), moving the end-effector to {_fmt_ee(ee_t)} with peak joint speed below "
                       f"{SAFETY['max_joint_velocity']:.1f} rad/s and joints tracking within {SAFETY['joint_tracking_alert_rad']:.2f} rad.")
        elif e["valid"]:
            exp_txt = f"Robot should hold its pose (end-effector at {_fmt_ee(e.get('ee'))}) with no motion commanded."
        else:
            exp_txt = "No expected state established yet."
    snaps = store.snapshots(inc["entity_id"], since, until, limit=20000)
    parts = []
    rules = {a.payload.get("rule") for a in anomalies}
    if snaps:
        vmax = max(max((abs(v) for v in s["physical"]["velocity"]), default=0.0) for s in snaps)
        emax = max(s["divergence"]["joint_max_abs"] for s in snaps)
        eem = max((s["divergence"]["ee_m"] or 0.0) for s in snaps)
        enc = max(s["divergence"]["encoder_max_abs"] for s in snaps)
        worst = max(snaps, key=lambda s: s["divergence"]["joint_max_abs"])
        if vmax > SAFETY["max_joint_velocity"]:
            parts.append(f"moved joints at up to {vmax:.2f} rad/s (limit {SAFETY['max_joint_velocity']:.1f})")
        if emax > SAFETY["joint_tracking_alert_rad"]:
            parts.append(f"deviated from its commanded trajectory by up to {emax:.3f} rad on {JOINTS[worst['divergence']['joint_max_index']]}")
        if eem > SAFETY["ee_alert_m"]:
            parts.append(f"put its end-effector up to {eem:.3f} m from where it should have been")
        if enc > SAFETY["encoder_divergence_rad"]:
            parts.append(f"reported joint angles up to {enc:.3f} rad away from its physical configuration")
    if "WORKSPACE_VIOLATION" in rules:
        parts.append("entered a restricted zone")
    if "PROXIMITY_RISK" in rules:
        parts.append("came within the human safety distance")
    if "TORQUE_LIMIT_EXCEEDED" in rules:
        parts.append("saturated joint torque")
    if "COLLISION_EVENT" in rules or any(a.event_type == EventType.CollisionDetected for a in anomalies):
        parts.append("made contact with an obstacle")
    if "INTERLOCK_VIOLATION" in rules:
        parts.append("triggered a machine interlock")
    if "MACHINE_STATE_MISMATCH" in rules:
        parts.append("acted on a wrong machine state")
    if "PROTECTIVE_STOP_NOT_ISSUED" in rules:
        parts.append("kept moving after a sensed intrusion")
    if "SSM_VIOLATION" in rules:
        parts.append("violated the speed-and-separation distance to the operator")
    if "UNEXPECTED_LOAD" in rules:
        parts.append("carried an unmodelled load")
    if "UNEXPECTED_COMMAND_SOURCE" in rules:
        parts.append("executed trajectories from an unauthorised node")
    act_txt = ("Robot " + ", ".join(parts) + ".") if parts else "No physical deviation beyond thresholds was recorded."

    cands = _candidates(store, inc, since, until, anomalies)
    ev_by_id = {e.event_id: e for e in kept}
    for a in anomalies:
        ev_by_id[a.event_id] = a
    evidence = []
    for i in {i for c in cands for i in c["evidence"]}:
        e = ev_by_id.get(i) or store.get(i)
        if e:
            evidence.append(e.model_dump())
    evidence.sort(key=lambda e: e["timestamp"])

    def _q(v):
        return "[" + ", ".join(f"{x:+.2f}" for x in v) + "]" if v else "n/a"

    return {
        "incident_id": inc["incident_id"],
        "window": {"since": since, "until": until},
        "timeline": timeline,
        "summary": {
            "expected": exp_txt, "actual": act_txt,
            "first_detected_divergence": first_div.timestamp if first_div else None,
            "first_detected_divergence_event": first_div.model_dump() if first_div else None,
            "rules_triggered": sorted(r for r in rules if r),
            "what_changed": list(dict.fromkeys(t["text"] for t in timeline if t["kind"] in ("ros", "command") and (t.get("unauthorised") or t["kind"] == "ros")))[:5]
                            or [c["title"] for c in cands[:2]],
            "robot_believed": (f"joints {_q(at['twin']['software_belief']['position'])}, EE {_fmt_ee(at['twin']['software_belief']['ee'])}, "
                               f"executing '{(at['twin']['software']['planner'].get('goal') or {}).get('name')}'"
                               + (f", task step '{at['twin'].get('task', {}).get('step')}'" if at['twin'].get('task', {}).get('step') else "")
                               + (f", machine believed {(at['twin']['software']['planner'].get('machine_belief') or {}).get('state')}" if (at['twin']['software']['planner'].get('machine_belief') or {}).get('state') else "")) if at else None,
            "task_step": (at["twin"].get("task") or {}).get("step") if at else None,
            "machine_state": ((at["twin"].get("machine") or {}).get("state") if at else None),
            "physically": (f"joints {_q(at['twin']['physical']['position'])}, EE {_fmt_ee(at['twin']['physical']['ee'])}, "
                           f"peak joint speed {max((abs(v) for v in at['twin']['physical']['velocity']), default=0):.2f} rad/s") if at else None,
        },
        "root_cause_candidates": cands,
        "evidence": evidence,
    }
