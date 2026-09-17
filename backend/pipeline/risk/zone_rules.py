"""Zone and speed-and-separation rules, configured per scenario (not in code).

Zone types: restricted / restricted_while -> WORKSPACE_VIOLATION (safety_rules.workspace);
reduced_speed -> REDUCED_SPEED_ZONE_VIOLATION; ssm -> SSM_VIOLATION (protective separation distance grows
with TCP speed, in the style of ISO/TS 15066: S = S_min + v_tcp * T_stop).  A sensed intrusion with
protective_stop=true creates the expectation that the controller stops within stop_within_s:
PROTECTIVE_STOP_NOT_ISSUED fires when motion continues.  Standards are design guidance for rule shape, not
a compliance claim.
"""
from __future__ import annotations

from backend.pipeline.risk.safety_rules import Hit


def _zone(twin: dict, zid: str) -> dict | None:
    return next((z for z in twin["environment"]["zones"] if z["id"] == zid), None)


def reduced_speed_zone(twin: dict, cfg: dict) -> Hit | None:
    p = twin["physical"]
    for zid in p.get("in_zones", []):
        z = _zone(twin, zid)
        if z and z["type"] == "reduced_speed" and z.get("max_tcp_speed") is not None and p["ee_speed"] > z["max_tcp_speed"]:
            return Hit("REDUCED_SPEED_ZONE_VIOLATION", "HIGH", "safety",
                       f"TCP moving at {p['ee_speed']:.2f} m/s inside reduced-speed zone {zid} (limit {z['max_tcp_speed']:.2f} m/s)",
                       {"zone": zid, "tcp_speed": p["ee_speed"], "limit": z["max_tcp_speed"], "ee": p["ee"], "task_step": twin["task"].get("step")},
                       "SafetyThresholdExceeded")
    return None


def ssm_violation(twin: dict, cfg: dict) -> Hit | None:
    p = twin["physical"]
    d = twin["environment"]["nearest_human_distance"]
    if d is None or p["ee_speed"] < 0.05:
        return None
    for z in twin["environment"]["zones"]:
        if z["type"] != "ssm" or z.get("min_separation") is None:
            continue
        hum = twin["environment"]["humans"][0] if twin["environment"]["humans"] else None
        if hum and not (z["min"][0] <= hum["x"] <= z["max"][0] and z["min"][1] <= hum["y"] <= z["max"][1]):
            continue
        required = z["min_separation"] + p["ee_speed"] * z.get("stop_time_s", 0.5)
        if d < required:
            return Hit("SSM_VIOLATION", "CRITICAL", "safety",
                       f"Separation {d:.2f} m is below the protective distance {required:.2f} m for TCP speed {p['ee_speed']:.2f} m/s (zone {z['id']})",
                       {"zone": z["id"], "separation": d, "required": required, "tcp_speed": p["ee_speed"], "min_separation": z["min_separation"],
                        "stop_time_s": z.get("stop_time_s", 0.5), "protective_stop": p.get("protective_stop")}, "SafetyThresholdExceeded")
    return None


def protective_stop_not_issued(twin: dict, cfg: dict) -> Hit | None:
    e = twin["expected"]
    p = twin["physical"]
    due = e.get("stop_required_by")
    if due is None:
        return None
    ts = twin["ts"] or 0.0
    moving = max((abs(v) for v in p["velocity"]), default=0.0) > 0.05
    if ts > due and moving and not p.get("protective_stop"):
        return Hit("PROTECTIVE_STOP_NOT_ISSUED", "CRITICAL", "safety",
                   f"Scanner intrusion was sensed {ts - (due - 0.5):.1f} s ago but the controller has not stopped (peak joint speed {max(abs(v) for v in p['velocity']):.2f} rad/s)",
                   {"sensed_since": (twin["sensors"].get("area_scanner") or {}).get("since"), "stop_required_by": due, "controller_state": twin["software"]["controller"].get("state"),
                    "tcp_speed": p["ee_speed"], "human_distance": twin["environment"]["nearest_human_distance"]}, "SafetyThresholdExceeded")
    return None


ZONE_RULES = [reduced_speed_zone, ssm_violation, protective_stop_not_issued]
