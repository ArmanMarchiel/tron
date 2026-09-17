"""Deterministic expected-state model for a manipulator.

No ML.  Expected state answers "what was the robot *supposed* to be doing?" and is derived
only from software intent: joint trajectories accepted from the authorised command source.

    trajectory (from -> to over duration D) accepted at t0
    expected q(t)  = from + (to - from) * s(tau),   tau = (t - t0) / D, s = minimum-jerk profile
    expected qd(t) = (to - from) * s'(tau) / D
    after tau >= 1 the robot is expected to hold ``to``

``from`` is the expected position at the moment the trajectory was accepted (continuity), or
the reported start positions for the very first trajectory.  Trajectories from an unexpected
node are *observed* but never become part of what the robot was supposed to do - that is
precisely the divergence we want to expose.
"""
from __future__ import annotations

import math


def minjerk(tau: float) -> tuple[float, float]:
    tau = min(1.0, max(0.0, tau))
    return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5, 30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4


def minjerk_acc(tau: float) -> float:
    tau = min(1.0, max(0.0, tau))
    return 60 * tau - 180 * tau ** 2 + 120 * tau ** 3


class ExpectedStateModel:
    def __init__(self, allowed_command_sources: list[str], n_joints: int = 7):
        self.allowed = set(allowed_command_sources)
        self.n = n_joints
        self.position: list[float] | None = None
        self.velocity: list[float] = [0.0] * n_joints
        self.acceleration: list[float] = [0.0] * n_joints
        self.traj: dict | None = None
        self.goal: dict | None = None
        self.cmd_source: str | None = None
        self.basis = "idle"
        self.anchor_ts: float | None = None
        self.progress = 0.0
        self.task: dict | None = None            # {step, index, started, duration_s, timeout_s, window_end}
        self.stop_required_by: float | None = None   # protective stop expected by this time (sensed intrusion)

    # ---------------------------------------------------------------- inputs
    def on_command(self, payload: dict, source_node: str | None, ts: float) -> bool:
        """Returns True if the trajectory came from an authorised source and was accepted."""
        if source_node is not None and source_node not in self.allowed:
            return False
        to = [float(x) for x in payload["positions"]][: self.n]
        self.advance(ts)
        frm = list(self.position) if self.position is not None else [float(x) for x in payload.get("start_positions", to)][: self.n]
        dur = max(0.05, float(payload.get("duration_s", 1.0)))
        self.traj = {"name": payload.get("goal_name"), "from": frm, "to": to, "start": ts, "duration_s": dur, "source": source_node}
        self.goal = {"name": payload.get("goal_name"), "positions": to}
        self.cmd_source = source_node
        self.position = frm
        self.anchor_ts = ts
        self.basis = "trajectory"
        self.progress = 0.0
        return True

    def on_goal_reached(self, ts: float) -> None:
        if self.traj is not None:
            self.position = list(self.traj["to"])
            self.velocity = [0.0] * self.n
            self.acceleration = [0.0] * self.n
        self.traj = None
        self.basis = "hold" if self.position is not None else "idle"
        self.progress = 1.0

    def reanchor_if_idle(self, ts: float, physical_positions: list[float]) -> None:
        if self.position is None:
            self.position = list(physical_positions[: self.n])
            self.anchor_ts = ts
            self.basis = "idle"

    def on_task_step(self, payload: dict, ts: float) -> None:
        dur = float(payload.get("duration_s") or 0.0)
        to = payload.get("timeout_s")
        self.task = {"step": payload.get("step"), "index": payload.get("index"), "action": payload.get("action"),
                     "started": ts, "duration_s": dur, "timeout_s": to, "window_end": ts + (to if to else dur * 1.5 + 2.5)}

    def on_task_done(self, ts: float) -> None:
        self.task = None

    def on_intrusion(self, sensed: bool, ts: float, stop_within_s: float) -> None:
        if sensed and self.stop_required_by is None:
            self.stop_required_by = ts + stop_within_s
        elif not sensed:
            self.stop_required_by = None

    # ---------------------------------------------------------------- integration
    def advance(self, ts: float) -> None:
        if self.traj is None:
            return
        t = self.traj
        tau = (ts - t["start"]) / t["duration_s"]
        s, ds = minjerk(tau)
        self.position = [f + (g - f) * s for f, g in zip(t["from"], t["to"])]
        self.velocity = [(g - f) * ds / t["duration_s"] for f, g in zip(t["from"], t["to"])] if tau < 1.0 else [0.0] * self.n
        dda = minjerk_acc(tau)
        self.acceleration = [(g - f) * dda / t["duration_s"] ** 2 for f, g in zip(t["from"], t["to"])] if tau < 1.0 else [0.0] * self.n
        self.progress = min(1.0, max(0.0, tau))
        if tau >= 1.0:
            self.basis = "hold"

    def state(self) -> dict:
        return {
            "valid": self.position is not None,
            "position": self.position,
            "velocity": self.velocity,
            "acceleration": self.acceleration,
            "ee": None,  # filled by the state engine via FK
            "goal": self.goal,
            "trajectory": self.traj,
            "progress": self.progress,
            "task": self.task,
            "stop_required_by": self.stop_required_by,
            "controller": "joint_trajectory_controller",
            "command_source": self.cmd_source,
            "basis": self.basis,
            "anchor_ts": self.anchor_ts,
        }


def compute_divergence(twin: dict, safety: dict) -> dict:
    p, b, e = twin["physical"], twin["software_belief"], twin["expected"]
    n = len(p["position"])
    if e["valid"] and e["position"]:
        joint_errors = [round(pa - ea, 4) for pa, ea in zip(p["position"], e["position"])]
        vel_err = [abs(pv - ev) for pv, ev in zip(p["velocity"], e["velocity"])]
    else:
        joint_errors = [0.0] * n
        vel_err = [0.0] * n
    jmax_i = max(range(n), key=lambda i: abs(joint_errors[i])) if n else 0
    vmax_i = max(range(n), key=lambda i: vel_err[i]) if n else 0
    jmax = abs(joint_errors[jmax_i]) if n else 0.0
    vmax = vel_err[vmax_i] if n else 0.0
    rms = math.sqrt(sum(x * x for x in joint_errors) / n) if n else 0.0
    ee_m = None
    if p["ee"] and e.get("ee"):
        ee_m = math.dist([p["ee"]["x"], p["ee"]["y"], p["ee"]["z"]], [e["ee"]["x"], e["ee"]["y"], e["ee"]["z"]])
    enc = [round(bb - pp, 4) for bb, pp in zip(b["position"], p["position"])]
    emax_i = max(range(n), key=lambda i: abs(enc[i])) if n else 0
    emax = abs(enc[emax_i]) if n else 0.0

    resid = None
    if e.get("effort") and p.get("effort"):
        resid = [round(pa - ea, 3) for pa, ea in zip(p["effort"], e["effort"])]
    ri = max(range(n), key=lambda i: abs(resid[i])) if resid else None
    level = "none"
    if jmax >= safety["joint_tracking_notice_rad"] or (ee_m is not None and ee_m >= safety["ee_notice_m"]) or vmax >= safety["velocity_divergence_notice"]:
        level = "notice"
    if jmax >= safety["joint_tracking_alert_rad"] or (ee_m is not None and ee_m >= safety["ee_alert_m"]) or vmax >= safety["velocity_divergence_alert"]:
        level = "alert"
    return {
        "joint_errors": joint_errors, "joint_max_abs": round(jmax, 4), "joint_max_index": jmax_i, "joint_rms": round(rms, 4),
        "velocity_max_abs": round(vmax, 4), "velocity_max_index": vmax_i,
        "ee_m": (round(ee_m, 4) if ee_m is not None else None),
        "encoder_errors": enc, "encoder_max_abs": round(emax, 4), "encoder_max_index": emax_i,
        "torque_residual": resid, "torque_residual_max": (round(abs(resid[ri]), 3) if resid else None), "torque_residual_index": ri,
        "level": level,
    }
