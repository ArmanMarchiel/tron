"""Deterministic safety rules for a manipulator, evaluated against the twin on every physical
observation.  Each rule returns ``None`` (not firing) or a ``Hit`` with evidence.  Rules never
mutate the twin.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from backend.app.config import EFFORT_LIMITS, JOINT_LIMITS, JOINTS


def _joints(twin: dict) -> list[str]:
    return twin["identity"].get("joints") or JOINTS


def _effort_limits(twin: dict) -> list[float]:
    return twin["identity"].get("limits", {}).get("effort") or EFFORT_LIMITS


def _joint_limits(twin: dict) -> list:
    return twin["identity"].get("limits", {}).get("joint") or (JOINT_LIMITS if len(_joints(twin)) == 7 else [])


def _vlimit(twin: dict, cfg: dict) -> float:
    return twin["identity"].get("limits", {}).get("max_joint_velocity") or cfg["max_joint_velocity"]


@dataclass
class Hit:
    rule: str
    severity: str            # LOW | MEDIUM | HIGH | CRITICAL
    category: str            # safety | security
    message: str
    evidence: dict = field(default_factory=dict)
    event_type: str = "AnomalyDetected"   # or SafetyThresholdExceeded


SEVERITY_ORDER = {"NONE": 0, "LOW": 1, "MEDIUM": 2, "HIGH": 3, "CRITICAL": 4}


def _argmax_abs(xs):
    i = max(range(len(xs)), key=lambda k: abs(xs[k])) if xs else 0
    return i, (abs(xs[i]) if xs else 0.0)


def joint_velocity_limit(twin: dict, cfg: dict) -> Hit | None:
    i, v = _argmax_abs(twin["physical"]["velocity"])
    limit = _vlimit(twin, cfg)
    J = _joints(twin)
    ev = twin["expected"]["velocity"][i] if twin["expected"]["valid"] else 0.0
    if v > limit:
        return Hit("JOINT_VELOCITY_LIMIT_EXCEEDED", "HIGH", "safety",
                   f"{J[i]} moving at {v:.2f} rad/s (limit {limit:.2f}; expected {abs(ev):.2f} rad/s)",
                   {"joint": J[i], "observed_velocity": v, "expected_velocity": ev, "limit": limit,
                    "trajectory": twin["expected"]["trajectory"], "last_command": twin["software"]["last_command"]},
                   "SafetyThresholdExceeded")
    return None


def joint_limits(twin: dict, cfg: dict) -> Hit | None:
    J = _joints(twin)
    for i, (q, (lo, hi)) in enumerate(zip(twin["physical"]["position"], _joint_limits(twin))):
        if hi - lo < 1e-6:
            continue
        if q < lo + cfg["joint_limit_margin_rad"] or q > hi - cfg["joint_limit_margin_rad"]:
            return Hit("JOINT_LIMIT_EXCEEDED", "HIGH", "safety", f"{J[i]} at {q:.3f} rad, limit [{lo:.3f}, {hi:.3f}]",
                       {"joint": J[i], "position": q, "limits": [lo, hi]}, "SafetyThresholdExceeded")
    return None


def effort_limit(twin: dict, cfg: dict) -> Hit | None:
    """Torque saturated *and* the joint is not reaching its target for a sustained period: a blocked joint.
    (Brief saturation while accelerating is normal for high-gain position control and is not flagged.)"""
    stall = twin["physical"].get("torque_stall_since") or []
    ts = twin["ts"] or 0.0
    J = _joints(twin)
    for i, (tau, lim) in enumerate(zip(twin["physical"]["effort"], _effort_limits(twin))):
        since = stall[i] if i < len(stall) else None
        if since is not None and ts - since >= cfg["effort_stall_s"]:
            return Hit("TORQUE_LIMIT_EXCEEDED", "HIGH", "safety",
                       f"{J[i]} has been saturated at {tau:.1f} N·m (limit {lim:.0f}) for {ts - since:.1f} s while "
                       f"{twin['divergence']['joint_errors'][i]:+.3f} rad from its target: the joint is blocked",
                       {"joint": J[i], "effort": tau, "limit": lim, "stalled_for_s": ts - since, "collision": twin["physical"]["collision"],
                        "tracking_error": twin["divergence"]["joint_errors"][i]}, "SafetyThresholdExceeded")
    return None


def workspace(twin: dict, cfg: dict) -> Hit | None:
    z = twin["physical"]["in_forbidden_zone"]
    if z:
        m = twin.get("machine") or {}
        return Hit("WORKSPACE_VIOLATION", "HIGH", "safety", f"End-effector entered restricted zone {z}" + (f" while machine state is {m.get('state')}" if m.get("state") else ""),
                   {"zone": z, "ee": twin["physical"]["ee"], "goal": twin["software"]["planner"]["goal"], "task_step": twin["task"].get("step"),
                    "machine_state": m.get("state")}, "SafetyThresholdExceeded")
    return None


def human_proximity(twin: dict, cfg: dict) -> Hit | None:
    d = twin["environment"]["nearest_human_distance"]
    if d is not None and d < cfg["human_safety_distance_m"]:
        sev = "CRITICAL" if d < cfg["human_safety_distance_m"] * 0.5 else "HIGH"
        return Hit("PROXIMITY_RISK", sev, "safety", f"Arm within {d:.2f} m of a human (threshold {cfg['human_safety_distance_m']:.2f} m)",
                   {"human_distance": d, "threshold": cfg["human_safety_distance_m"], "ee": twin["physical"]["ee"],
                    "ee_speed": twin["physical"]["ee_speed"]}, "SafetyThresholdExceeded")
    return None


def collision(twin: dict, cfg: dict) -> Hit | None:
    if twin["physical"].get("collision"):
        return Hit("COLLISION_EVENT", "CRITICAL", "safety", f"Contact with {twin['physical'].get('collision_with') or 'unknown object'}",
                   {"with": twin["physical"].get("collision_with"), "ee": twin["physical"]["ee"], "effort": twin["physical"]["effort"],
                    "goal": twin["software"]["planner"]["goal"]}, "SafetyThresholdExceeded")
    return None


def sensor_divergence(twin: dict, cfg: dict) -> Hit | None:
    d = twin["divergence"]
    J = _joints(twin)
    # area scanner (when the cell has one): person physically in the field, sensor silent
    sc = twin["sensors"].get("area_scanner") or {}
    p = twin["physical"]
    if sc and p.get("human_in_scanner_field") and sc.get("intrusion") is False and sc.get("ts") is not None:
        since = p.get("human_in_field_since")
        if since is not None and (twin["ts"] or 0) - since >= cfg["scanner_miss_s"]:
            return Hit("SENSOR_STATE_DIVERGENCE", "CRITICAL", "safety",
                       f"A person has been inside the area scanner field for {(twin['ts'] or 0) - since:.1f} s but scanner {sc.get('id')} reports no intrusion",
                       {"sensor": sc.get("id"), "reported_intrusion": False, "physical_in_field": True, "since": since,
                        "human_distance": twin["environment"]["nearest_human_distance"], "tcp_speed": p.get("ee_speed")})
    since = p.get("encoder_over_since")
    if d["encoder_max_abs"] > cfg["encoder_divergence_rad"] and since is not None and (twin["ts"] or 0) - since >= 0.3:
        i = d["encoder_max_index"]
        return Hit("SENSOR_STATE_DIVERGENCE", "HIGH", "safety",
                   f"{J[i]} reported at {twin['software_belief']['position'][i]:.3f} rad on /joint_states; physically at "
                   f"{twin['physical']['position'][i]:.3f} rad (Δ {d['encoder_errors'][i]:+.3f} rad)",
                   {"joint": J[i], "reported": twin["software_belief"]["position"][i], "physical": twin["physical"]["position"][i],
                    "difference_rad": d["encoder_errors"][i], "threshold": cfg["encoder_divergence_rad"],
                    "reported_ee": twin["software_belief"]["ee"], "physical_ee": twin["physical"]["ee"]})
    return None


def trajectory_divergence(twin: dict, cfg: dict) -> Hit | None:
    d = twin["divergence"]
    if not twin["expected"]["valid"]:
        return None
    if twin["physical"].get("protective_stop"):
        return None   # a protective stop legitimately freezes the arm mid-trajectory
    if d["joint_max_abs"] > cfg["joint_tracking_alert_rad"] or (d["ee_m"] is not None and d["ee_m"] > cfg["ee_alert_m"]):
        i = d["joint_max_index"]
        J = _joints(twin)
        return Hit("TRAJECTORY_DIVERGENCE", "MEDIUM", "safety",
                   f"Tracking error {d['joint_max_abs']:.3f} rad on {J[i]}"
                   + (f"; end-effector {d['ee_m']:.3f} m from expected" if d["ee_m"] is not None else ""),
                   {"joint": J[i], "joint_errors": d["joint_errors"], "ee_m": d["ee_m"], "expected_ee": twin["expected"]["ee"],
                    "actual_ee": twin["physical"]["ee"], "goal": twin["expected"]["goal"], "effort": twin["physical"]["effort"]})
    return None


def unexpected_load(twin: dict, cfg: dict) -> Hit | None:
    """Observed joint torque differs from the shadow model's prediction for a sustained period: an
    external load or contact the software does not know about (model-based collision detection)."""
    d = twin["divergence"]
    if d.get("torque_residual") is None:
        return None
    J = _joints(twin)
    since = twin["physical"].get("torque_residual_since") or []
    ts = twin["ts"] or 0.0
    thr = cfg.get("torque_residual_nm", [])
    for i, r in enumerate(d["torque_residual"]):
        t = thr[i] if i < len(thr) else 10.0
        s = since[i] if i < len(since) else None
        if abs(r) > t and s is not None and ts - s >= cfg["torque_residual_s"] and not twin["physical"].get("collision"):
            return Hit("UNEXPECTED_LOAD", "HIGH", "safety",
                       f"{J[i]} torque is {r:+.1f} N·m away from the shadow model's prediction for {ts - s:.1f} s (threshold {t:.0f}): an unmodelled load or contact",
                       {"joint": J[i], "residual_nm": r, "threshold_nm": t, "observed_effort": twin["physical"]["effort"][i],
                        "predicted_effort": (twin["expected"].get("effort") or [None] * len(J))[i], "payload_kg": twin["physical"].get("payload_kg"),
                        "gripper": twin["physical"].get("gripper")})
    return None


SAFETY_RULES = [joint_velocity_limit, joint_limits, effort_limit, workspace, human_proximity, collision,
                sensor_divergence, trajectory_divergence, unexpected_load]
