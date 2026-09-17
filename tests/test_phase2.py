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


# ---------------------------------------------------------------- cell semantics (ROM, end state, part shape)

def _cnc():
    from backend.conf.scenarios.loader import load_scenario
    return load_scenario("cnc_tending")


def test_reach_envelope_zone_is_sized_from_the_robot_and_drawn_as_a_cylinder():
    """The ROM is the arm's reach plus a safety margin, so it scales with whichever robot is loaded."""
    import mujoco
    from backend.conf.registry import get_robot
    from backend.io.sim.compose import compose

    sc = _cnc()
    zone = next(z for z in sc.zones if z.type == "reach_envelope")
    for rid in ("franka_panda", "ur10e"):
        robot = get_robot(rid)
        m = mujoco.MjModel.from_xml_path(str(compose(robot, sc, f"test_rom_{rid}")))
        geom = m.geom("zone_robot_rom")
        assert geom.type == mujoco.mjtGeom.mjGEOM_CYLINDER
        assert geom.size[0] == pytest.approx(robot.reach_m + zone.margin_m, abs=1e-6)
        # non-colliding: the envelope is an annotation, it must never touch the physics
        assert geom.contype == 0 and geom.conaffinity == 0


def test_human_inside_the_reach_envelope_stops_the_robot_and_leaving_resumes_it():
    """A person in the arm's range of motion is a protective stop in its own right -- even with the
    area scanner blinded, which is exactly fault H."""
    from backend.conf.registry import get_robot
    from backend.io.collectors.mujoco_adapter import MujocoAdapter
    from backend.io.sim.compose import compose
    from backend.io.sim.ik import bake
    from backend.app.session import scenario_targets

    sc, robot = _cnc(), get_robot("franka_panda")
    scene = compose(robot, sc, "test_rom_stop")
    a = MujocoAdapter(lambda evs: None, robot, sc, scene, bake(scene, robot, scenario_targets(sc), "test_rom_stop"), ROBOT)
    a.scanner_blind = True                       # only the ROM can stop the arm now

    assert not a._human_in_reach()               # the ordinary patrol lane is outside the envelope
    a.human.x, a.human.y = 0.3, 0.3              # step the operator inside it
    assert a._human_in_reach()

    a.human_in_reach = True
    a._controller_step(1000.0)
    assert a.protective_stop and a.pstop_source == "reach_envelope"

    a.human_in_reach = False                     # clear of the cell -> the stop releases itself
    a._controller_step(1001.0)
    assert not a.protective_stop and a.pstop_source is None


def test_run_ends_after_the_scenario_cycle_count():
    """cycles: 4 means four parts, then a terminal state that dispatches nothing further."""
    from backend.conf.registry import get_robot
    from backend.io.collectors.mujoco_adapter import MujocoAdapter, TaskRunner
    from backend.io.sim.compose import compose
    from backend.io.sim.ik import bake
    from backend.app.session import scenario_targets

    sc, robot = _cnc(), get_robot("franka_panda")
    assert sc.task.cycles == 4                   # four raw blocks -> four finished parts
    scene = compose(robot, sc, "test_cycles")
    seen = []
    a = MujocoAdapter(lambda evs: seen.extend(evs), robot, sc, scene,
                      bake(scene, robot, scenario_targets(sc), "test_cycles"), ROBOT)
    run: TaskRunner = a.task
    for _ in range(sc.task.cycles):              # wrap the step list once per cycle
        run.index = len(run.steps) - 1
        run._start_next(1000.0)

    assert run.cycles_done == 4 and run.status == "done"
    assert run.view() == {"cycles_done": 4, "cycles_target": 4, "done": True, "finished_at": 1000.0}
    kinds = [str(e.event_type) for e in seen]
    assert kinds.count("TaskCycleCompleted") == 4 and kinds.count("TaskRunCompleted") == 1

    before = run.index                           # terminal: further ticks are no-ops
    run.step(1001.0)
    assert run.index == before and a.traj is None


def test_finished_parts_are_cylinders_and_raw_stock_is_square():
    """The CNC's output is a turned cylinder, so a finished part is distinguishable on sight."""
    import mujoco
    from backend.conf.registry import get_robot
    from backend.io.sim.compose import compose

    sc = _cnc()
    finished = [b for b in sc.blocks if b.finished]
    raw = [b for b in sc.blocks if not b.finished]
    assert finished and all(b.shape == "cylinder" for b in finished)
    assert len(raw) == 4 and all(b.shape == "box" for b in raw)   # one per cycle

    m = mujoco.MjModel.from_xml_path(str(compose(get_robot("franka_panda"), sc, "test_shapes")))
    assert m.geom(f"{finished[0].id}_geom").type == mujoco.mjtGeom.mjGEOM_CYLINDER
    assert m.geom(f"{raw[0].id}_geom").type == mujoco.mjtGeom.mjGEOM_BOX


# ---------------------------------------------------------------- machine registry / Haas VF-2

def test_machine_registry_resolves_the_scenario_ref_to_its_own_mjcf():
    """`machine: {ref: ...}` selects the model, so swapping machines is a registry entry + an XML."""
    from backend.conf.machines import get_machine, list_machines

    ids = {mm["id"] for mm in list_machines()}
    assert "haas_vf2_cad" in ids

    for mid in ids:
        assert get_machine(mid).mjcf_path.exists()
    with pytest.raises(KeyError):
        get_machine("no_such_machine")


def test_haas_vf2_matches_the_published_machine_layout_drawing():
    """Dimensions come from the Haas MLD for the VF-2 (2023-01-17), not from guesswork."""
    from backend.conf.machines import get_machine

    vf2 = get_machine("haas_vf2_cad")
    # the CAD is the bare machine, so its footprint sits inside the published *operating* envelope
    # (3147 x 2249 x 2724 mm), which includes the swung-out pendant and the raised spindle
    assert 2.2 < vf2.width <= 3.147
    assert 2.2 < vf2.depth <= 2.40                      # depth is the closest match to spec
    assert vf2.travels == {"x": pytest.approx(0.762), "y": pytest.approx(0.406), "z": pytest.approx(0.508)}
    assert vf2.spindle["max_rpm"] == 8100


def test_the_cnc_scenario_composes_the_vf2_and_every_waypoint_is_reachable():
    """The cell is laid out around a true-scale machine, so the arm must still reach the vise."""
    import mujoco
    from backend.conf.registry import get_robot, load_robots
    from backend.io.sim.compose import compose
    from backend.io.sim.ik import bake
    from backend.app.session import scenario_targets

    sc = _cnc()
    assert sc.machine.ref.startswith("haas_vf2")     # whichever VF-2 model the cell is built on

    for rid in load_robots():
        robot = get_robot(rid)
        scene = compose(robot, sc, f"test_vf2_{rid}")
        model = mujoco.MjModel.from_xml_path(str(scene))
        # the adapter's contract: named door/jaw joints and the chuck site
        for jnt in ("cnc_door_joint", "cnc_jaw_l_joint", "cnc_jaw_r_joint"):
            assert model.joint(jnt) is not None
        assert model.site("cnc_chuck_site") is not None

        wp = bake(scene, robot, scenario_targets(sc), f"test_vf2_{rid}")
        unreachable = {k: v["err"] for k, v in wp.items() if v.get("err", 0) > 0.02}
        assert not unreachable, f"{rid}: unreachable waypoints {unreachable}"


def test_the_interior_zone_guards_the_working_volume_not_the_whole_footprint():
    """Standing beside a 3.1 m wide machine is not a zone breach; reaching through the door is."""
    from backend.conf.machines import get_machine

    sc = _cnc()
    zone = next(z for z in sc.zones if z.id == "cnc_interior")
    assert not zone.min and not zone.max          # resolved from the machine profile at runtime

    prof = get_machine(sc.machine.ref)
    lo, hi = prof.interior_bounds(sc.machine.pose)
    assert hi[0] - lo[0] < prof.depth                         # narrower than the machine itself
    assert hi[1] - lo[1] < prof.width
    chuck = [sc.machine.pose[i] + sc.machine.chuck_offset[i] for i in range(3)]
    assert all(lo[i] <= chuck[i] <= hi[i] for i in range(3))  # but it does contain the vise


def test_twin_door_machines_drive_both_panels_from_one_plc_target():
    """The VF-2's doors part sideways; the adapter drives one joint and mirrors the other."""
    from backend.conf.machines import get_machine
    from backend.io.machines.cnc import CNCMachine

    vf2 = get_machine("haas_vf2_cad")
    m = CNCMachine("cnc-1", 12.0, ["/motion_planner"], part_loaded=True,
                   door_travel=vf2.door["travel"], jaw_clamped=vf2.jaw_clamped)
    assert m.door_travel == pytest.approx(vf2.door["travel"])   # not the platform default of 0.78

    m.state = "COMPLETE"
    ok, _ = m.command("door_open", "/motion_planner", 0.0)
    assert ok and m.door_target == pytest.approx(m.door_travel)
    m.set_door_position(m.door_travel)
    assert m.door == "OPEN"                       # reported against this machine's own travel


def test_registry_door_travel_matches_what_the_model_can_actually_slide():
    """A travel the joint cannot reach would leave the door reporting MOVING forever."""
    import mujoco
    from backend.conf.machines import get_machine
    from backend.conf.registry import get_robot
    from backend.io.sim.compose import compose

    sc = _cnc()
    model = mujoco.MjModel.from_xml_path(str(compose(get_robot("franka_panda"), sc, "test_door")))
    lo, hi = model.jnt_range[model.joint("cnc_door_joint").id]
    assert get_machine(sc.machine.ref).door["travel"] == pytest.approx(hi)


# ---------------------------------------------------------------- shop floor, aisle, overlay toggles

def test_the_operator_aisle_never_crosses_the_machine_or_the_marked_cell():
    """The patrol runs the aisle outside the painted boundary -- a person cannot walk through a
    3.1 m machine. The human body is deliberately non-colliding (mocap-driven), so the route, not
    contact physics, is what keeps them out."""
    import mujoco
    from backend.conf.machines import get_machine
    from backend.conf.registry import get_robot
    from backend.io.sim.compose import cell_bounds, compose
    from backend.io.sim.human import HumanOperator

    sc, robot = _cnc(), get_robot("franka_panda")
    prof, pose = get_machine(sc.machine.ref), sc.machine.pose
    mx0, mx1 = pose[0] + prof.front_offset, pose[0] - prof.front_offset
    my0, my1 = pose[1] - prof.width / 2, pose[1] + prof.width / 2
    x0, x1, y0, y1 = cell_bounds(sc, robot)

    model = mujoco.MjModel.from_xml_path(str(compose(robot, sc, "test_aisle")))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    human = HumanOperator(model, data, sc.human, [])   # no area scanner in this cell

    t = 0.0
    for _ in range(4000):                        # ~80 s of patrol, several laps
        t += 0.02
        human.step(0.02, t)
        x, y, _z = human.position()
        assert not (mx0 <= x <= mx1 and my0 <= y <= my1), f"operator walked into the machine at ({x}, {y})"
        assert not (x0 <= x <= x1 and y0 <= y <= y1), f"operator crossed the cell boundary at ({x}, {y})"


def test_floor_markings_enclose_the_machine_and_the_robot_envelope():
    """The painted rectangle is derived from the cell's contents, so it follows a machine swap."""
    import mujoco
    from backend.conf.machines import get_machine
    from backend.conf.registry import get_robot
    from backend.io.sim.compose import cell_bounds, compose

    sc, robot = _cnc(), get_robot("franka_panda")
    x0, x1, y0, y1 = cell_bounds(sc, robot)
    prof, pose = get_machine(sc.machine.ref), sc.machine.pose

    assert x0 < pose[0] + prof.front_offset and x1 > pose[0] - prof.front_offset   # machine inside
    rom = robot.reach_m + next(z.margin_m for z in sc.zones if z.type == "reach_envelope")
    assert x0 < -rom and x1 > rom and y0 < -rom and y1 > rom                       # ROM inside

    model = mujoco.MjModel.from_xml_path(str(compose(robot, sc, "test_lines")))
    lines = [model.geom(i).name for i in range(model.ngeom) if model.geom(i).name.startswith("floor_line")]
    assert len(lines) == 4                                    # one per side
    for name in lines:                                        # paint must never be a physics obstacle
        assert model.geom(name).contype == 0 and model.geom(name).conaffinity == 0


def test_overlay_groups_can_be_hidden_without_touching_physics():
    """Zones, the ROM, the scanner field and the floor paint are annotations the viewer can switch
    off; hiding one only zeroes its alpha."""
    import mujoco
    from backend.conf.registry import get_robot
    from backend.io.sim.compose import compose
    from backend.io.sim.renderer import RenderService

    sc, robot = _cnc(), get_robot("franka_panda")
    model = mujoco.MjModel.from_xml_path(str(compose(robot, sc, "test_overlays")))
    svc = RenderService(model, lambda: (None, None))

    state = svc.overlay_state()
    # one toggle per zone, named after the zone itself; floor paint is not switchable
    assert {z.id for z in _cnc().zones} == set(state["overlays"])
    assert not any(k.startswith("floor") for k in state["overlays"])
    assert all(state["overlays"].values())                    # everything starts visible

    rom_geom = model.geom("zone_robot_rom").id
    before = float(model.geom_rgba[rom_geom][3])
    assert before > 0

    svc.set_overlay("robot_rom", False)
    assert model.geom_rgba[rom_geom][3] == 0.0                # hidden
    assert svc.overlay_state()["overlays"]["robot_rom"] is False
    # hiding one zone must not touch another
    assert model.geom_rgba[model.geom("zone_cnc_interior").id][3] > 0

    svc.set_overlay("robot_rom", True)
    assert model.geom_rgba[rom_geom][3] == pytest.approx(before)   # restored, not guessed

    with pytest.raises(KeyError):
        svc.set_overlay("not_an_overlay", False)


def test_the_cad_vf2_keeps_its_enclosure_open_to_the_robot():
    """A mesh imported from CAD collides as its convex hull, so the shell is visual only and the
    machine's collision comes from primitives that leave the door aperture clear."""
    import mujoco
    from backend.conf.machines import get_machine
    from backend.conf.registry import get_robot
    from backend.io.sim.compose import compose

    cad = get_machine("haas_vf2_cad")
    assert cad.meshes and cad.mjcf_path.exists()
    for spec in cad.meshes.values():                      # the tessellated shell ships with the repo
        assert (cad.mjcf_path.parent / spec["file"]).exists()
        assert len(spec["rgba"]) == 4                     # each colour group keeps its CAD colour

    model = mujoco.MjModel.from_xml_path(str(compose(get_robot("franka_panda"), _cnc(), "test_cad")))
    shells = [model.geom(i) for i in range(model.ngeom)
              if (model.geom(i).name or "").startswith("cnc_shell")]
    assert len(shells) == len(cad.meshes)                 # one geom per colour group
    for shell in shells:
        assert shell.type == mujoco.mjtGeom.mjGEOM_MESH
        assert shell.contype == 0 and shell.conaffinity == 0   # visual only: never collides

    # the collision primitives that stand in for it do collide, and skip the door opening
    solid = [model.geom(i).name for i in range(model.ngeom)
             if (model.geom(i).name or "").startswith("cnc_") and model.geom_contype[i] != 0]
    assert "cnc_base" in solid and "cnc_side_l" in solid and "cnc_back" in solid
    assert not any(n.startswith("cnc_front_header") for n in solid)   # no lintel across the aperture
