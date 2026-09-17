"""Task-level and machine rules: the robot's behaviour is checked against the task it is executing and
the machine it works with, not only against its own trajectory."""
from __future__ import annotations

from backend.pipeline.risk.safety_rules import Hit


def task_step_timeout(twin: dict, cfg: dict) -> Hit | None:
    e = twin["expected"].get("task")
    if not e or twin["task"].get("status") != "active":
        return None
    ts = twin["ts"] or 0.0
    if ts > e["window_end"]:
        return Hit("TASK_STEP_TIMEOUT", "MEDIUM", "safety",
                   f"Task step '{e['step']}' has run for {ts - e['started']:.1f} s, beyond its window of {e['window_end'] - e['started']:.1f} s",
                   {"step": e["step"], "action": e.get("action"), "started": e["started"], "window_s": e["window_end"] - e["started"],
                    "machine": twin.get("machine"), "controller": twin["software"]["controller"]})
    return None


def task_order_violation(twin: dict, cfg: dict) -> Hit | None:
    f = twin["task"].get("last_failure")
    if f and (twin["ts"] or 0) - f["ts"] < 5.0 and "order" in (f.get("reason") or ""):
        return Hit("TASK_ORDER_VIOLATION", "MEDIUM", "safety", f"Task step '{f['step']}' started out of sequence: {f['reason']}",
                   {"step": f["step"], "reason": f["reason"]})
    return None


def interlock_violation(twin: dict, cfg: dict) -> Hit | None:
    m = twin.get("machine")
    if not m:
        return None
    il = m.get("last_interlock")
    ts = twin["ts"] or 0.0
    if il and ts - il["ts"] < 3.0:
        return Hit("INTERLOCK_VIOLATION", "HIGH", "safety",
                   f"Machine {m.get('id')} rejected '{il['command']}' from {il['client']}: {il['reason']}",
                   {"machine": m.get("id"), "command": il["command"], "client": il["client"], "reason": il["reason"],
                    "state": m.get("state"), "task_step": twin["task"].get("step")}, "SafetyThresholdExceeded")
    if m.get("alarm"):
        return Hit("INTERLOCK_VIOLATION", "CRITICAL", "safety", f"Machine {m.get('id')} is in ALARM: {m['alarm']}",
                   {"machine": m.get("id"), "alarm": m["alarm"], "state": m.get("state"), "door": m.get("door"),
                    "task_step": twin["task"].get("step")}, "SafetyThresholdExceeded")
    return None


def machine_state_mismatch(twin: dict, cfg: dict) -> Hit | None:
    """The planner acted on a machine state that differs from what the machine itself reports."""
    m = twin.get("machine")
    belief = twin["software"]["planner"].get("machine_belief")
    if not m or not belief or twin["task"].get("status") != "active":
        return None
    if belief.get("state") and m.get("state") and belief["state"] != m["state"] and m["state"] in ("RUNNING", "ALARM"):
        return Hit("MACHINE_STATE_MISMATCH", "CRITICAL", "safety",
                   f"Planner started step '{twin['task'].get('step')}' believing the machine is {belief['state']}, but the machine reports {m['state']}",
                   {"machine": m.get("id"), "planner_belief": belief, "machine_report": {k: m.get(k) for k in ("state", "door", "chuck", "alarm")},
                    "task_step": twin["task"].get("step")}, "SafetyThresholdExceeded")
    return None


def unexpected_machine_command_source(twin: dict, cfg: dict) -> Hit | None:
    m = twin.get("machine")
    cmd = twin["software"].get("last_machine_command")
    if not m or not cmd:
        return None
    if (twin["ts"] or 0) - cmd["ts"] > 3.0:
        return None
    allowed = m.get("allowed_clients") or []
    if allowed and cmd["client"] not in allowed:
        return Hit("UNEXPECTED_MACHINE_COMMAND_SOURCE", "CRITICAL", "security",
                   f"Machine command '{cmd['command']}' from unauthorised client {cmd['client']}",
                   {"machine": m.get("id"), "command": cmd, "allowed_clients": allowed})
    return None


TASK_RULES = [task_step_timeout, task_order_violation, interlock_violation, machine_state_mismatch, unexpected_machine_command_source]
