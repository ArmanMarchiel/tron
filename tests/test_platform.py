"""End-to-end tests of the platform core using synthetic normalised joint-space events (no simulator)."""
from __future__ import annotations

import math

import pytest

from backend.app.api.platform import Platform
from backend.app.config import COMMAND_TOPIC, JOINTS
from backend.pipeline.events.event_model import EventType, make_event
from backend.pipeline.twin import kinematics
from backend.pipeline.twin.expected_state import ExpectedStateModel, minjerk

ROBOT = "robot-001"
PHYS, ROS = "test/physics", "test/ros2"
HOME = [0.0, 0.0, 0.0, -1.571, 0.0, 1.571, -0.785]
PICK = [-0.404, 0.638, -0.175, -1.807, 0.16, 2.431, -1.452]


@pytest.fixture
def platform():
    return Platform(":memory:")


def _boot(p: Platform, t0: float) -> None:
    for n in ["/motion_planner", "/joint_trajectory_controller", "/joint_state_broadcaster"]:
        p.ingest([make_event(EventType.ROSNodeStarted, ROS, ROBOT, {"node": n}, timestamp=t0)])
    p.ingest([make_event(EventType.ROSTopicObserved, ROS, ROBOT, {"topic": COMMAND_TOPIC, "publishers": ["/motion_planner"],
                                                                  "subscribers": ["/joint_trajectory_controller"], "rate_hz": 0.3}, timestamp=t0)])


def _traj(p: Platform, t: float, name: str, frm: list, to: list, dur: float, source: str = "/motion_planner") -> None:
    pl = {"topic": COMMAND_TOPIC, "source_node": source, "goal_name": name, "positions": to, "duration_s": dur, "start_positions": frm, "joint_names": JOINTS}
    if source == "/motion_planner":
        p.ingest([make_event(EventType.TrajectoryGoalReceived, ROS, ROBOT, {"goal_name": name, "positions": to, "duration_s": dur, "source_node": source}, timestamp=t)])
    p.ingest([make_event(EventType.CommandReceived, ROS, ROBOT, pl, timestamp=t)])


def _observe(p: Platform, t: float, q: list, qd: list | None = None, reported: list | None = None, effort: list | None = None,
             collision: bool = False, human: float = 1.0, obstacle: float = 0.3, zone: str | None = None, ee: dict | None = None) -> None:
    qd = qd or [0.0] * 7
    p.ingest([make_event(EventType.PositionObserved, ROS, ROBOT, {"topic": "/joint_states", "joint_names": JOINTS, "position": reported or q, "velocity": qd}, timestamp=t)])
    p.ingest([make_event(EventType.RobotStateObserved, PHYS, ROBOT, {
        "frame": "ground_truth", "joint_names": JOINTS, "position": q, "velocity": qd, "effort": effort or [0.0] * 7,
        "ee": ee or kinematics.fk(q), "ee_speed": 0.0, "collision": collision, "collision_with": "obstacle-1" if collision else None,
        "nearest_obstacle_distance": obstacle, "nearest_human_distance": human, "in_forbidden_zone": zone}, timestamp=t)])


def _interp(a, b, tau):
    s, _ = minjerk(tau)
    return [x + (y - x) * s for x, y in zip(a, b)]


def test_expected_model_follows_minjerk_and_ignores_unauthorised():
    m = ExpectedStateModel(["/motion_planner"])
    assert m.on_command({"positions": PICK, "duration_s": 4.0, "start_positions": HOME, "goal_name": "pick"}, "/motion_planner", 0.0)
    assert not m.on_command({"positions": HOME, "duration_s": 1.0, "start_positions": HOME, "goal_name": "rogue"}, "/rogue", 0.5)
    m.advance(2.0)
    s = m.state()
    assert s["goal"]["name"] == "pick"
    assert s["position"] == pytest.approx(_interp(HOME, PICK, 0.5), abs=1e-6)
    assert max(abs(v) for v in s["velocity"]) > 0
    m.advance(5.0)
    assert m.state()["position"] == pytest.approx(PICK, abs=1e-6)
    assert m.state()["velocity"] == [0.0] * 7 and m.state()["basis"] == "hold"


def test_no_divergence_when_tracking_is_good(platform):
    t0 = 1000.0
    _boot(platform, t0)
    _observe(platform, t0, HOME)
    _traj(platform, t0 + 0.1, "pick", HOME, PICK, 4.0)
    for i in range(45):
        t = t0 + 0.1 + 0.1 * (i + 1)
        tau = min(1.0, (t - (t0 + 0.1)) / 4.0)
        qd = [(y - x) * minjerk(tau)[1] / 4.0 for x, y in zip(HOME, PICK)] if tau < 1.0 else [0.0] * 7
        _observe(platform, t, _interp(HOME, PICK, tau), qd=qd)
    tw = platform.state.twin
    assert tw["divergence"]["level"] == "none"
    assert tw["risk"]["level"] == "NONE"
    assert tw["expected"]["ee"] is not None and tw["physical"]["ee"] is not None
    assert platform.incidents.list() == []


def test_velocity_fault_creates_incident_with_reconstruction(platform):
    t0 = 2000.0
    _boot(platform, t0)
    _observe(platform, t0, HOME)
    _traj(platform, t0 + 0.1, "pick", HOME, PICK, 4.0)
    # the controller runs 2.5x too fast: joint velocities exceed 1 rad/s and tracking diverges
    t_fault = t0 + 0.2
    for i in range(20):
        t = t_fault + 0.1 * i
        tau = min(1.0, (t - (t0 + 0.1)) / 1.6)
        qd = [(y - x) * minjerk(tau)[1] / 1.6 for x, y in zip(HOME, PICK)]
        _observe(platform, t, _interp(HOME, PICK, tau), qd=qd)
    tw = platform.state.twin
    rules = [r["rule"] for r in tw["risk"]["active_rules"]]
    assert "JOINT_VELOCITY_LIMIT_EXCEEDED" in rules or "TRAJECTORY_DIVERGENCE" in rules
    incs = platform.incidents.list()
    assert len(incs) == 1
    rec = platform.incidents.get(incs[0]["incident_id"])["reconstruction"]
    assert rec["summary"]["first_detected_divergence"] is not None
    assert any(c["label"] == "Inferred" and "faster" in c["title"] for c in rec["root_cause_candidates"])
    kinds = [e["kind"] for e in rec["timeline"]]
    assert "anomaly" in kinds and "incident" in kinds and "command" in kinds
    for entry in rec["timeline"]:
        assert platform.store.get(entry["event_id"]) is not None


def test_unexpected_command_source_is_security_incident(platform):
    t0 = 3000.0
    _boot(platform, t0)
    _observe(platform, t0, HOME)
    _traj(platform, t0 + 0.1, "pick", HOME, PICK, 4.0)
    t1 = t0 + 1.0
    ROGUE = [0.9, 0.6, 0.3, -1.4, 0.0, 1.9, 0.2]
    platform.ingest([make_event(EventType.ROSNodeStarted, ROS, ROBOT, {"node": "/teleop_override"}, timestamp=t1)])
    platform.ingest([make_event(EventType.ROSTopicObserved, ROS, ROBOT, {"topic": COMMAND_TOPIC, "publishers": ["/motion_planner", "/teleop_override"],
                                                                         "subscribers": ["/joint_trajectory_controller"], "rate_hz": 10.0}, timestamp=t1)])
    for i in range(15):
        t = t1 + 0.1 * (i + 1)
        _traj(platform, t - 0.01, "rogue", HOME, ROGUE, 2.0, source="/teleop_override")
        _observe(platform, t, _interp(HOME, ROGUE, min(1.0, (t - t1) / 2.0)), human=0.9 - 0.05 * i)
    rules = {r["rule"] for r in platform.state.twin["risk"]["active_rules"]}
    assert {"UNEXPECTED_COMMAND_SOURCE", "UNEXPECTED_ROS_NODE", "UNEXPECTED_MESSAGE_RATE", "TRAJECTORY_DIVERGENCE"} <= rules
    assert platform.state.twin["expected"]["goal"]["name"] == "pick"   # the rogue trajectory never became "expected"
    inc = platform.incidents.get(platform.incidents.list()[0]["incident_id"])
    top = inc["reconstruction"]["root_cause_candidates"][0]
    assert top["label"] == "Observed" and "/teleop_override" in top["title"]
    assert any("[UNAUTHORISED SOURCE]" in t["text"] for t in inc["reconstruction"]["timeline"])


def test_sensor_divergence(platform):
    t0 = 4000.0
    _boot(platform, t0)
    reported = list(HOME); reported[1] += 0.25
    for i in range(5):
        _observe(platform, t0 + 0.1 * (i + 1), HOME, reported=reported)
    tw = platform.state.twin
    assert "SENSOR_STATE_DIVERGENCE" in [r["rule"] for r in tw["risk"]["active_rules"]]
    assert tw["divergence"]["encoder_max_abs"] == pytest.approx(0.25) and tw["divergence"]["encoder_max_index"] == 1
    assert tw["divergence"]["level"] == "none"   # expected vs physical is still fine: this is a reporting fault


def test_collision_workspace_torque_and_proximity_rules(platform):
    t0 = 5000.0
    _boot(platform, t0)
    platform.ingest([make_event(EventType.CollisionDetected, PHYS, ROBOT, {"with": "obstacle-1"}, timestamp=t0 + 0.1)])
    _traj(platform, t0 + 0.05, "pick", HOME, PICK, 0.5)   # target moves away while the joint stays put (blocked)
    for i in range(8):
        _observe(platform, t0 + 0.1 * (i + 1), HOME, effort=[80.0, 0, 0, 0, 0, 0, 0], collision=True, human=0.2, ee={"x": 0.3, "y": 0.7, "z": 0.3})
    rules = {r["rule"] for r in platform.state.twin["risk"]["active_rules"]}
    assert {"COLLISION_EVENT", "WORKSPACE_VIOLATION", "TORQUE_LIMIT_EXCEEDED", "PROXIMITY_RISK"} <= rules
    assert platform.state.twin["risk"]["level"] == "CRITICAL"


def test_timeline_snapshot_reconstructs_past_state(platform):
    t0 = 6000.0
    _boot(platform, t0)
    _traj(platform, t0, "pick", HOME, PICK, 4.0)
    for i in range(30):
        _observe(platform, t0 + 0.1 * (i + 1), _interp(HOME, PICK, min(1.0, 0.1 * (i + 1) / 4.0)))
    from backend.pipeline.incidents.timeline import state_at, track
    past = state_at(platform.store, ROBOT, t0 + 1.0)
    assert past is not None
    assert past["twin"]["physical"]["position"] == pytest.approx(_interp(HOME, PICK, 0.25), abs=1e-4)
    assert past["twin"]["software"]["nodes"]["/motion_planner"]["state"] == "running"
    assert past["twin"]["software"]["planner"]["goal"]["name"] == "pick"
    pts = track(platform.store, ROBOT, t0, t0 + 3.1)
    assert len(pts) >= 25 and "vel" in pts[0] and "err" in pts[0]


def test_incident_closes_after_quiet_period(platform):
    t0 = 7000.0
    _boot(platform, t0)
    _observe(platform, t0, HOME)
    _traj(platform, t0 + 0.1, "pick", HOME, PICK, 4.0)
    for i in range(10):
        _observe(platform, t0 + 0.2 + 0.1 * i, HOME, qd=[1.5, 0, 0, 0, 0, 0, 0])
    assert platform.incidents.list()[0]["status"] == "open"
    platform.ingest([make_event(EventType.TrajectoryGoalReached, ROS, ROBOT, {"goal_name": "pick"}, timestamp=t0 + 1.3)])
    for i in range(120):
        _observe(platform, t0 + 1.4 + 0.1 * i, PICK)
    inc = platform.incidents.list()[0]
    assert inc["status"] == "closed" and inc["closed_at"] is not None
    assert platform.store.get_incident(inc["incident_id"])["reconstruction"] is not None


def test_event_store_query_roundtrip(platform):
    e = make_event(EventType.ROSNodeStarted, ROS, ROBOT, {"node": "/x"}, timestamp=1.0)
    platform.ingest([e])
    got = platform.store.query(types=[EventType.ROSNodeStarted])
    assert got and got[0].event_id == e.event_id and got[0].payload == {"node": "/x"} and got[0].seq is not None
