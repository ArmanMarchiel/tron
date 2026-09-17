"""Controlled, simulation-only fault scenarios.

Each scenario is an explicit, named perturbation applied by the *adapter* (local sim or
Isaac Sim script).  The platform never modifies the robot; it records the operator's
request as a ``FaultInjected`` event and exposes the active fault to adapters via
``GET /api/faults/active``.  Nothing here is random.
"""
from __future__ import annotations

SCENARIOS: dict[str, dict] = {
    "A": {
        "id": "A", "name": "Excessive velocity", "expected_detection": "JOINT_VELOCITY_LIMIT_EXCEEDED",
        "description": "Controller executes trajectories 2.5x faster than commanded (timing fault); joint speeds exceed the 1.0 rad/s limit.",
        "params": {"speed_factor": 2.5}, "duration_s": 10.0,
    },
    "B": {
        "id": "B", "name": "Unexpected command source", "expected_detection": "UNEXPECTED_COMMAND_SOURCE",
        "description": "A rogue node (/teleop_override) joins the graph and publishes a joint trajectory that swings the arm toward the human.",
        "params": {"rogue_node": "/teleop_override", "goal": "rogue", "rate_hz": 10.0, "duration_s": 2.0}, "duration_s": 9.0,
    },
    "C": {
        "id": "C", "name": "Sensor disagreement", "expected_detection": "SENSOR_STATE_DIVERGENCE",
        "description": "joint2 encoder reports +0.25 rad relative to the physical joint angle; /joint_states no longer matches reality.",
        "params": {"encoder_offset": {"joint": 1, "rad": 0.25}}, "duration_s": 10.0,
    },
    "D": {
        "id": "D", "name": "Trajectory deviation", "expected_detection": "TRAJECTORY_DIVERGENCE",
        "description": "Actuator degradation: joint2 and joint4 lose 95% of position gain (current-limited drives); the arm sags under gravity while commands stay valid.",
        "params": {"actuator_gain_scale": 0.05, "joints": [1, 3]}, "duration_s": 10.0,
    },
    "E": {
        "id": "E", "name": "Collision", "expected_detection": "COLLISION_EVENT",
        "description": "Planner sends a trajectory whose target lies inside obstacle-1 with the collision check disabled; the gripper drives into the obstacle.",
        "params": {"goal": "obstacle", "disable_collision_check": True}, "duration_s": 12.0,
    },
}


def get_scenario(scenario_id: str) -> dict | None:
    return SCENARIOS.get(scenario_id.upper())
