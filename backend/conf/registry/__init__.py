"""Asset registry: robots (and, via scenarios, machines/humans/props).

A ``RobotProfile`` is everything the platform needs to know about an arm that is not derivable from
the twin: joint names and limits, effort limits, home pose, tool frame, gripper, wrist camera, ROS 2
topic names and per-robot safety threshold overrides.  Joint ranges are read from the MJCF at scene
composition time so the YAML never duplicates them.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

ASSETS = Path(__file__).resolve().parent.parent.parent / "io" / "sim" / "assets"
REGISTRY_FILE = Path(__file__).resolve().parent / "robots.yaml"


@dataclass
class RobotProfile:
    id: str
    name: str
    mjcf: str
    joints: list[str]
    actuators: list[str]
    home: list[float]
    effort_limits: list[float]
    ee_body: str
    ee_offset: list[float]
    ee_quat: list[float]
    gripper: dict
    wrist_camera: dict
    pedestal_height: float
    reach_m: float
    thresholds: dict = field(default_factory=dict)
    ros2: dict = field(default_factory=dict)
    joint_limits: list[list[float]] = field(default_factory=list)   # filled after compile

    @property
    def n(self) -> int:
        return len(self.joints)

    @property
    def mjcf_path(self) -> Path:
        return ASSETS / self.mjcf

    @property
    def gripper_actuator(self) -> str:
        return self.gripper.get("actuator", "tron_gripper")

    def identity(self) -> dict:
        return {"model": self.id, "name": self.name, "joints": list(self.joints),
                "limits": {"joint": self.joint_limits, "effort": list(self.effort_limits),
                           "max_joint_velocity": self.thresholds.get("max_joint_velocity", 1.0),
                           "tracking_notice": 0.05, "tracking_alert": 0.15},
                "ros2": dict(self.ros2)}


_cache: dict[str, RobotProfile] | None = None


def load_robots() -> dict[str, RobotProfile]:
    global _cache
    if _cache is None:
        raw = yaml.safe_load(REGISTRY_FILE.read_text())["robots"]
        _cache = {rid: RobotProfile(id=rid, **spec) for rid, spec in raw.items()}
    return _cache


def get_robot(robot_id: str) -> RobotProfile:
    robots = load_robots()
    if robot_id not in robots:
        raise KeyError(f"unknown robot '{robot_id}'; known: {', '.join(robots)}")
    return robots[robot_id]


def list_robots() -> list[dict]:
    return [{"id": r.id, "name": r.name, "dof": r.n, "reach_m": r.reach_m} for r in load_robots().values()]
