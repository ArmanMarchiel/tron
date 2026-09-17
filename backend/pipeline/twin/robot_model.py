"""Persistent digital representation of one manipulator.

Three things are deliberately kept apart and never merged:

* ``physical``        - OBSERVED physical state (what the simulator / world reported)
* ``software_belief`` - OBSERVED software state (what the robot's own stack reports on /joint_states)
* ``expected``        - EXPECTED state: what the robot was *supposed* to be doing, derived
                        deterministically from accepted trajectories.

The model is a plain, JSON-serialisable tree so it can be snapshotted every tick and
reconstructed at any point in time.
"""
from __future__ import annotations

import copy
from typing import Any

from backend.app.config import JOINTS


def new_robot_twin(robot_id: str, identity: dict, environment: dict) -> dict[str, Any]:
    """identity: from RobotProfile.identity() (name, model, joints, limits, ros2); environment: scenario-derived."""
    joints = list(identity.get("joints") or JOINTS)
    n = len(joints)
    return {
        "identity": {"robot_id": robot_id, "name": identity.get("name", robot_id), "model": identity.get("model", "unknown"),
                     "joints": joints, "limits": identity.get("limits", {}), "ros2": identity.get("ros2", {}),
                     "scenario": environment.get("scenario"), "adapters": []},
        "physical": {  # observed ground truth
            "position": [0.0] * n, "velocity": [0.0] * n, "effort": [0.0] * n,
            "ee": None, "ee_speed": 0.0, "gripper": None, "payload_kg": None,
            "collision": False, "collision_with": None, "in_forbidden_zone": None, "in_zones": [],
            "protective_stop": False, "human_in_scanner_field": False,
            "protective_stop_source": None, "human_in_reach_envelope": False,
            "source": None, "ts": None,
        },
        "software_belief": {"position": [0.0] * n, "velocity": [0.0] * n, "effort": [0.0] * n, "ee": None, "source": None, "ts": None},
        "sensors": {
            "encoders": {"position": None, "ts": None},
            "camera": {"fps": None, "ts": None},
            "area_scanner": {"intrusion": None, "since": None, "ts": None, "id": None},
        },
        "actuators": {"targets": [0.0] * n, "gripper": None, "state": "unknown"},
        "software": {
            "nodes": {}, "topics": {},
            "controller": {"name": "joint_trajectory_controller", "state": "unknown", "progress": 0.0, "goal_name": None, "last_command": None},
            "planner": {"goal": None, "status": "idle", "goal_received_ts": None, "machine_belief": None},
            "last_command": None,
            "last_machine_command": None,
        },
        "task": {"scenario": environment.get("scenario"), "step": None, "index": None, "action": None, "started_ts": None,
                 "duration_s": None, "timeout_s": None, "status": "idle", "completed": [], "last_failure": None,
                 "cycles_done": 0, "cycles_target": None, "run_done": False, "finished_ts": None},
        "machine": ({"id": environment["machine"].get("id"), "state": None, "door": None, "chuck": None, "alarm": None,
                     "cycle_count": None, "part_loaded": None, "cycle_progress": 0.0, "last_command": None, "last_interlock": None,
                     "allowed_clients": environment["machine"].get("allowed_clients", []), "ts": None}
                    if environment.get("machine") else None),
        "network": {"participants": {}, "connections": [], "message_activity": {}},
        "environment": {
            "obstacles": copy.deepcopy(environment.get("obstacles", [])),
            "humans": copy.deepcopy(environment.get("humans", [])),
            "zones": copy.deepcopy(environment.get("zones", [])),
            "sensors": copy.deepcopy(environment.get("sensors", [])),
            "nearest_obstacle_distance": None, "nearest_human_distance": None,
        },
        "expected": {
            "valid": False, "position": None, "velocity": [0.0] * n, "ee": None, "effort": None,
            "goal": None, "trajectory": None, "progress": 0.0,
            "task": None, "stop_required_by": None,
            "controller": "joint_trajectory_controller", "command_source": None, "basis": "idle", "anchor_ts": None,
        },
        "divergence": {
            "joint_errors": [0.0] * n, "joint_max_abs": 0.0, "joint_max_index": 0, "joint_rms": 0.0,
            "velocity_max_abs": 0.0, "velocity_max_index": 0, "ee_m": None,
            "encoder_errors": [0.0] * n, "encoder_max_abs": 0.0, "encoder_max_index": 0,
            "torque_residual": None, "torque_residual_max": None, "torque_residual_index": None, "level": "none",
        },
        "risk": {"level": "NONE", "score": 0, "active_rules": []},
        "incidents": {"active": []},
        "status": "initialising",
        "ts": None,
    }


def robot_summary(twin: dict) -> dict:
    p, e, d = twin["physical"], twin["expected"], twin["divergence"]
    return {
        "robot_id": twin["identity"]["robot_id"],
        "status": twin["status"],
        "risk": twin["risk"]["level"],
        "active_incidents": len(twin["incidents"]["active"]),
        "joints": twin["identity"]["joints"],
        "expected": {"position": e["position"], "velocity": e["velocity"], "ee": e["ee"], "goal": e["goal"], "controller": e["controller"]},
        "actual": {"position": p["position"], "velocity": p["velocity"], "ee": p["ee"], "goal": twin["software"]["planner"]["goal"],
                   "controller": twin["software"]["controller"]["name"]},
        "divergence": d,
    }
