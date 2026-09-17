"""Phase 2 rules, registry/composition and MCAP round-trip (no live simulator needed)."""
from __future__ import annotations

import tempfile
from pathlib import Path

import pytest

from backend.app.api.platform import Platform
from backend.app.config import COMMAND_TOPIC, JOINTS
from backend.pipeline.events.event_model import EventType, make_event
from backend.pipeline.twin import kinematics

ROBOT = "robot-001"
PHYS, ROS, MACH = "test/physics", "test/ros2", "machine"
HOME = [0.0, 0.0, 0.0, -1.571, 0.0, 1.571, -0.785]


def env_cnc():
    return {"scenario": "test", "obstacles": [], "humans": [{"id": "human-1", "x": 0.3, "y": 1.2, "z": 0.95}],
            "zones": [{"id": "cnc_interior", "type": "restricted_while", "min": [0.66, -0.4, 0.0], "max": [1.02, 0.4, 0.85], "condition": "machine.state == RUNNING"},
                      {"id": "operator_side", "type": "ssm", "min": [-0.5, 0.6, 0.0], "max": [1.6, 1.8, 2.0], "min_separation": 0.5, "stop_time_s": 0.5},
                      {"id": "collab", "type": "reduced_speed", "min": [-0.2, 0.62, 0.0], "max": [1.4, 1.0, 1.2], "max_tcp_speed": 0.25}],
            "sensors": [{"id": "scanner-1", "type": "area_scanner", "field_min": [-0.3, 0.6, 0.0], "field_max": [1.5, 1.0, 0.0], "protective_stop": True, "stop_within_s": 0.5}],
            "machine": {"id": "cnc-1", "allowed_clients": ["/motion_planner", "cell_plc"]}}


@pytest.fixture
def platform():
    return Platform(":memory:", environment=env_cnc())


def _observe(p, t, q=HOME, ee=None, ee_speed=0.0, human=1.0, in_field=False, pstop=False, effort=None, qd=None):
    p.ingest([make_event(EventType.PositionObserved, ROS, ROBOT, {"joint_names": JOINTS, "position": q, "velocity": qd or [0.0] * 7}, timestamp=t)])
    p.ingest([make_event(EventType.RobotStateObserved, PHYS, ROBOT, {
        "joint_names": JOINTS, "position": q, "velocity": qd or [0.0] * 7, "effort": effort or [0.0] * 7, "ee": ee or kinematics.fk(q), "ee_speed": ee_speed,
        "collision": False, "nearest_obstacle_distance": 0.3, "nearest_human_distance": human, "human_in_field": in_field, "protective_stop": pstop,
        "gripper": 255.0, "payload_kg": 0.0}, timestamp=t)])


def _machine(p, t, state="COMPLETE", door="CLOSED", chuck="CLAMPED", alarm=None):
    p.ingest([make_event(EventType.MachineStateObserved, MACH, ROBOT, {"machine_id": "cnc-1", "state": state, "door": door, "chuck": chuck,
                                                                        "alarm": alarm, "cycle_count": 1, "part_loaded": True}, timestamp=t)])


def rules(p):
    return {r["rule"] for r in p.state.twin["risk"]["active_rules"]}


def test_task_step_timeout(platform):
    t0 = 1000.0
    _machine(platform, t0, state="RUNNING")
    platform.ingest([make_event(EventType.TaskStepStarted, ROS, ROBOT, {"step": "wait_cycle", "index": 0, "action": "wait", "duration_s": 5, "timeout_s": 6,
                                                                         "machine_belief": {"state": "RUNNING"}}, timestamp=t0)])
    _observe(platform, t0 + 3)
    assert "TASK_STEP_TIMEOUT" not in rules(platform)
    _observe(platform, t0 + 7)
    assert "TASK_STEP_TIMEOUT" in rules(platform)
    platform.ingest([make_event(EventType.TaskStepCompleted, ROS, ROBOT, {"step": "wait_cycle"}, timestamp=t0 + 7.1)])
    _observe(platform, t0 + 7.2)
    assert "TASK_STEP_TIMEOUT" not in rules(platform)


def test_machine_state_mismatch_and_restricted_while_zone(platform):
    t0 = 2000.0
    _machine(platform, t0, state="RUNNING")
    platform.ingest([make_event(EventType.TaskStepStarted, ROS, ROBOT, {"step": "to_chuck", "index": 2, "action": "move", "duration_s": 4,
                                                                         "machine_belief": {"state": "COMPLETE"}}, timestamp=t0 + 0.1)])
    _observe(platform, t0 + 0.2, ee={"x": 0.8, "y": 0.0, "z": 0.3})   # inside cnc_interior while RUNNING
    r = rules(platform)
    assert "MACHINE_STATE_MISMATCH" in r and "WORKSPACE_VIOLATION" in r
    _machine(platform, t0 + 0.3, state="COMPLETE")
    _observe(platform, t0 + 0.4, ee={"x": 0.8, "y": 0.0, "z": 0.3})
    assert "WORKSPACE_VIOLATION" not in rules(platform)     # condition no longer holds


def test_interlock_and_unauthorised_machine_command(platform):
    t0 = 3000.0
    _machine(platform, t0, state="RUNNING")
    platform.ingest([make_event(EventType.MachineCommandReceived, MACH, ROBOT, {"machine_id": "cnc-1", "command": "door_open", "client": "/hmi_debug", "accepted": False, "reason": "door interlock"}, timestamp=t0 + 0.1)])
    platform.ingest([make_event(EventType.InterlockViolated, MACH, ROBOT, {"machine_id": "cnc-1", "command": "door_open", "client": "/hmi_debug", "reason": "door interlock", "state": "RUNNING"}, timestamp=t0 + 0.1)])
    _observe(platform, t0 + 0.2)
    r = rules(platform)
    assert "INTERLOCK_VIOLATION" in r and "UNEXPECTED_MACHINE_COMMAND_SOURCE" in r


def test_ssm_and_reduced_speed(platform):
    t0 = 4000.0
    _observe(platform, t0, ee={"x": 0.5, "y": 0.8, "z": 0.4}, ee_speed=0.6, human=0.55)
    r = rules(platform)
    assert "SSM_VIOLATION" in r and "REDUCED_SPEED_ZONE_VIOLATION" in r
    _observe(platform, t0 + 0.1, ee={"x": 0.5, "y": 0.8, "z": 0.4}, ee_speed=0.0, human=0.55)
    assert "SSM_VIOLATION" not in rules(platform)   # stationary arm: no separation requirement


def test_protective_stop_expectation_and_scanner_miss(platform):
    t0 = 5000.0
    # sensed intrusion, controller keeps moving -> PROTECTIVE_STOP_NOT_ISSUED
    platform.ingest([make_event(EventType.SensorObservation, ROS, ROBOT, {"sensor": "area_scanner", "id": "scanner-1", "intrusion": True}, timestamp=t0)])
    _observe(platform, t0 + 0.1, qd=[0.5] + [0.0] * 6, in_field=True)
    assert "PROTECTIVE_STOP_NOT_ISSUED" not in rules(platform)
    _observe(platform, t0 + 0.8, qd=[0.5] + [0.0] * 6, in_field=True)
    assert "PROTECTIVE_STOP_NOT_ISSUED" in rules(platform)
    # controller stops -> cleared, and PROTECTIVE_STOP_OBSERVED informational event
    _observe(platform, t0 + 1.0, in_field=True, pstop=True)
    assert "PROTECTIVE_STOP_NOT_ISSUED" not in rules(platform)
    obs = platform.store.query(types=[EventType.SafetyThresholdExceeded], limit=50)
    assert any(e.payload.get("rule") == "PROTECTIVE_STOP_OBSERVED" for e in obs)
    assert not any(i["rules"] == ["PROTECTIVE_STOP_OBSERVED"] for i in platform.incidents.list())
    # scanner blind: person physically in field, scanner clear for > 0.5 s
    platform.ingest([make_event(EventType.SensorObservation, ROS, ROBOT, {"sensor": "area_scanner", "id": "scanner-1", "intrusion": False}, timestamp=t0 + 2.0)])
    for i in range(8):
        _observe(platform, t0 + 2.1 + 0.1 * i, in_field=True)
    assert "SENSOR_STATE_DIVERGENCE" in rules(platform)


def test_unexpected_load_from_torque_residual(platform):
    class FakeShadow:
        def predict(self, twin):
            return [0.0] * 7
    platform.state.shadow = FakeShadow()
    t0 = 6000.0
    for i in range(10):
        _observe(platform, t0 + 0.1 * i, effort=[0, 30.0, 0, 0, 0, 0, 0])
    assert "UNEXPECTED_LOAD" in rules(platform)
    assert platform.state.twin["divergence"]["torque_residual_max"] == pytest.approx(30.0)


def test_registry_and_composition():
    from backend.conf.registry import load_robots
    from backend.conf.scenarios.loader import load_scenario
    from backend.app.session import scenario_targets
    from backend.io.sim.compose import compose
    import mujoco
    sc = load_scenario("cnc_tending")
    targets = scenario_targets(sc)
    assert "machine.chuck" in targets and "raw_tray.slot0+0.15" in targets and "home" not in targets
    for rid, r in load_robots().items():
        m = mujoco.MjModel.from_xml_path(str(compose(r, sc, f"unit_{rid}")))
        names = [m.camera(i).name for i in range(m.ncam)]
        assert "wrist_cam" in names and m.site("ee_site").id >= 0
        assert m.actuator(r.gripper_actuator).id >= 0


def test_mcap_record_and_replay_roundtrip():
    from backend.io.collectors.replay_adapter import ReplayAdapter, record_mcap
    src = Platform(":memory:")
    t0 = 7000.0
    src.ingest([make_event(EventType.ROSNodeStarted, ROS, ROBOT, {"node": "/motion_planner"}, timestamp=t0)])
    src.ingest([make_event(EventType.CommandReceived, ROS, ROBOT, {"topic": COMMAND_TOPIC, "source_node": "/motion_planner", "goal_name": "pick", "positions": HOME,
                                                                   "duration_s": 4.0, "start_positions": HOME, "joint_names": JOINTS}, timestamp=t0 + 0.1)])
    for i in range(10):
        _observe(src, t0 + 0.2 + 0.1 * i, qd=[1.5, 0, 0, 0, 0, 0, 0])   # over-speed -> anomaly in the source run
    assert src.incidents.list()
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "run.mcap"
        n = record_mcap(src.store, out, t0 - 1, t0 + 5, src.state.twin["identity"])
        assert n > 10 and out.stat().st_size > 0
        dst = Platform(":memory:")
        ad = ReplayAdapter(dst.ingest, out, speed=0.0, robot_id=ROBOT)
        ad.run()
        assert ad.state == "finished" and ad.count >= n
        # the replayed twin derives the same detection from the log
        assert "JOINT_VELOCITY_LIMIT_EXCEEDED" in {r["rule"] for r in dst.state.twin["risk"]["active_rules"]}
        assert dst.incidents.list() and dst.incidents.list()[0]["primary_rule"] == src.incidents.list()[0]["primary_rule"]
        # ROS channels are present for a real collector to consume
        from mcap.reader import make_reader
        with open(out, "rb") as f:
            topics = {c.topic for c in make_reader(f).get_summary().channels.values()}
        assert "/joint_states" in topics and COMMAND_TOPIC in topics and "/tron/ground_truth" in topics
