"""Platform configuration: robot identity, joint/safety limits, security allow-lists, environment."""
from __future__ import annotations

import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent     # repo root (frontend/, tests/)
BACKEND_DIR = Path(__file__).resolve().parent.parent     # backend/ (data, io, conf, pipeline, scripts)
DATA_DIR = BACKEND_DIR / "data"
DB_PATH = os.environ.get("TRON_DB", str(DATA_DIR / "tron.sqlite"))

ROBOT_ID = "robot-001"
ROBOT_NAME = "Franka Emika Panda (7-DoF)"
ROBOT_MODEL = "franka_panda"

# "mujoco": run the MuJoCo cell in-process (physics ground truth + simulated ROS 2 graph).
# "external": do nothing in-process; wait for the ROS 2 collector / real robot to POST /api/ingest.
ADAPTER = os.environ.get("TRON_ADAPTER", "mujoco")

SIM = {
    "scene": str(BACKEND_DIR / "io" / "sim" / "assets" / "scene.xml"),
    "waypoints": str(BACKEND_DIR / "io" / "sim" / "assets" / "waypoints.json"),
    "camera": "twin_cam",
    "cameras": {"environment": "twin_cam", "robot": "wrist_cam"},
    "render_hz": 15, "width": 880, "height": 780, "jpeg_quality": 80,
    "control_hz": 50,
}

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
JOINT_LIMITS = [(-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973), (-3.0718, -0.0698),
                (-2.8973, 2.8973), (-0.0175, 3.7525), (-2.8973, 2.8973)]
EFFORT_LIMITS = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]

DEFAULT_ROBOT = os.environ.get("TRON_ROBOT", "franka_panda")
DEFAULT_SCENARIO = os.environ.get("TRON_SCENARIO", "cnc_tending")

# ---------------------------------------------------------------- safety limits (robot profiles may override)
SAFETY = {
    "max_joint_velocity": 1.0,          # rad/s  (planner trajectories peak ~0.6 rad/s)
    "velocity_divergence_notice": 0.2,  # rad/s  |actual - expected| joint velocity
    "velocity_divergence_alert": 0.4,
    "joint_tracking_notice_rad": 0.05,
    "joint_tracking_alert_rad": 0.15,
    "ee_notice_m": 0.04,
    "ee_alert_m": 0.08,
    "encoder_divergence_rad": 0.10,     # reported joint angle vs physical joint angle
    "effort_alert_ratio": 0.85,         # |effort| > ratio * joint effort limit ...
    "effort_stall_s": 0.3,
    "effort_stall_error_rad": 0.08,     # ... and be at least this far from its target              # ... while failing to track its target for this long (torque saturated AND stalled)
    "joint_limit_margin_rad": 0.03,
    "human_safety_distance_m": 0.35,
    "cmd_max_joint_velocity": 1.2,      # a trajectory whose implied peak velocity exceeds this is abnormal
    "scanner_miss_s": 0.5,              # human physically in a scanner field but not reported for this long
    "torque_residual_nm": [10.0, 10.0, 8.0, 8.0, 6.0, 6.0, 6.0],   # shadow-sim residual per joint (Nm); wrist limits are 12
    "torque_residual_s": 0.6,
}

# ---------------------------------------------------------------- security allow-lists
COMMAND_TOPIC = "/joint_trajectory_controller/joint_trajectory"
SECURITY = {
    "expected_nodes": [
        "/motion_planner", "/joint_trajectory_controller", "/joint_state_broadcaster", "/camera_driver",
        "/robot_state_publisher", "/tron_collector", "/mujoco_bridge",
    ],
    "expected_publishers": {
        COMMAND_TOPIC: ["/motion_planner"],
        "/joint_states": ["/joint_state_broadcaster", "/mujoco_bridge"],
        "/joint_trajectory_controller/state": ["/joint_trajectory_controller"],
        "/camera/image_raw": ["/camera_driver", "/mujoco_bridge"],
        "/tf": ["/robot_state_publisher"],
    },
    "expected_subscribers": {
        COMMAND_TOPIC: ["/joint_trajectory_controller", "/tron_collector"],
    },
    "expected_rates_hz": {   # (min, max)
        COMMAND_TOPIC: (0.0, 5.0),
        "/joint_states": (10.0, 200.0),
        "/joint_trajectory_controller/state": (5.0, 100.0),
    },
    "expected_participants": [
        "motion_planner", "joint_trajectory_controller", "joint_state_broadcaster", "camera_driver",
        "robot_state_publisher", "tron_collector", "mujoco_bridge",
    ],
    "command_topic": COMMAND_TOPIC,
}

# ---------------------------------------------------------------- environment (metres, robot base frame)
ENVIRONMENT = {
    "obstacles": [{"id": "obstacle-1", "pos": [0.55, 0.0, 0.12], "size": [0.07, 0.07, 0.12]}],
    "table": {"pos": [0.55, 0.0, -0.02], "size": [0.45, 0.6, 0.02]},
    "humans": [{"id": "human-1", "pos": [0.35, 1.05, 0.15], "patrol": [[0.35, 1.05], [0.35, 0.92]], "speed": 0.04, "radius": 0.16, "half_height": 0.45}],
    "forbidden_zones": [{"id": "restricted-1", "min": [0.0, 0.55, 0.0], "max": [0.5, 0.95, 0.7]}],
}

# ---------------------------------------------------------------- incidents
INCIDENTS = {
    "group_window_s": 10.0,
    "close_after_s": 6.0,
    "lookback_s": 10.0,
}

SNAPSHOT_HZ = 10.0
SNAPSHOT_RETENTION_S = 3600.0
