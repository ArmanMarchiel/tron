"""Session manager: robot + scenario -> composed scene, baked targets, adapters, engines.

A session is one cell configuration running in one platform instance.  Starting a session stops the
previous adapters and renderer, composes the MJCF scene for the selected robot and scenario, bakes the
scenario's joint-space targets with IK (cached), reconfigures the platform engines with the robot
identity and the scenario environment, and starts the MuJoCo adapter, the shadow simulation and the
renderer.  ``adapter="external"`` starts no simulator (real robot / replay).
"""
from __future__ import annotations

import time
import uuid
from pathlib import Path

from backend.app.config import DATA_DIR, ROBOT_ID, SAFETY, SECURITY, SIM
from backend.pipeline.events.event_model import EventType, Source, make_event
from backend.conf.registry import RobotProfile, get_robot
from backend.conf.scenarios.loader import load_scenario
from backend.conf.scenarios.schema import Scenario
from backend.io.sim.compose import compose
from backend.io.sim.ik import bake
from backend.pipeline.twin import kinematics

SESSIONS_DIR = Path(DATA_DIR) / "sessions"


def scenario_targets(sc: Scenario) -> dict[str, list[float]]:
    """Every named EE target the task runner may ask for (tray slots with height variants, machine points)."""
    t: dict[str, list[float]] = dict(sc.targets)
    grasp_z = (sc.blocks[0].size if sc.blocks else 0.03)          # fingertips at the block's centre height
    offsets = {s.height_offset for s in sc.task.steps if s.action == "move"} | {0.0}
    for tray in sc.trays:
        for i, slot in enumerate(tray.slots or [[0.0, 0.0]]):
            base = [tray.pose[0] + slot[0], tray.pose[1] + slot[1], grasp_z]
            for h in offsets:
                key = f"{tray.id}.slot{i}" + (f"+{h:g}" if h else "")
                t[key] = [base[0], base[1], base[2] + h]
    if sc.machine:
        mp, co = sc.machine.pose, sc.machine.chuck_offset
        chuck = [mp[0] + co[0], mp[1] + co[1], mp[2] + co[2]]
        for h in offsets:
            suffix = f"+{h:g}" if h else ""
            t[f"machine.chuck{suffix}"] = [chuck[0], chuck[1], chuck[2] + h]
            t[f"machine.chuck_above{suffix}"] = [chuck[0], chuck[1], chuck[2] + 0.22 + h]
        t["machine.door_frame"] = [mp[0] - 0.19, mp[1] - 0.05, 0.40]   # into the closed door panel
        t["machine.front"] = [mp[0] - 0.42, mp[1], 0.40]                # retract point in front of the door, clear of the side walls
        t["machine.obstacle"] = [mp[0] - 0.02, mp[1] + 0.20, 0.05]        # into the machine bed beside the chuck (always present)
    return t


def scenario_environment(sc: Scenario) -> dict:
    return {
        "scenario": sc.id,
        "obstacles": [{"id": b.id, "pos": b.pose, "size": [b.size] * 3} for b in sc.blocks if b.mass >= 2.0],
        "humans": [{"id": sc.human.id, "x": sc.human.pose[0], "y": sc.human.pose[1], "z": sc.human.pose[2]}] if sc.human else [],
        "zones": [z.model_dump() for z in sc.zones],
        "sensors": [s.model_dump() for s in sc.sensors],
        "machine": sc.machine.model_dump() if sc.machine else None,
    }


class SessionManager:
    def __init__(self, platform):
        self.platform = platform
        self.current: dict | None = None
        self._threads = []

    def stop(self) -> None:
        p = self.platform
        if p.renderer:
            p.renderer.stop()
        if p.adapter and hasattr(p.adapter, "stop"):
            p.adapter.stop()
        p.renderer = None
        p.adapter = None
        time.sleep(0.15)

    def start(self, robot_id: str, scenario_id: str, adapter: str = "mujoco", fresh_db: bool = True) -> dict:
        self.stop()
        robot = get_robot(robot_id)
        sc = load_scenario(scenario_id)
        session_id = f"{robot_id}_{scenario_id}_{uuid.uuid4().hex[:6]}"
        scene = compose(robot, sc, session_id)
        import mujoco
        m = mujoco.MjModel.from_xml_path(str(scene))
        robot.joint_limits = [[float(m.jnt_range[m.joint(j).id][0]), float(m.jnt_range[m.joint(j).id][1])] for j in robot.joints]
        wp = bake(scene, robot, scenario_targets(sc), f"{robot_id}_{scenario_id}")
        kinematics.configure(scene, robot.joints)
        SESSIONS_DIR.mkdir(parents=True, exist_ok=True)
        db = str(SESSIONS_DIR / f"{session_id}.sqlite")
        safety = dict(SAFETY, **robot.thresholds)
        cmd_topic = robot.ros2.get("command_topic", SECURITY["command_topic"])
        security = dict(SECURITY)
        security["expected_publishers"] = dict(SECURITY["expected_publishers"])
        security["expected_publishers"].setdefault(cmd_topic, ["/motion_planner"])
        security["expected_subscribers"] = dict(SECURITY["expected_subscribers"])
        security["expected_subscribers"].setdefault(cmd_topic, [f"/{robot.ros2.get('controller', 'joint_trajectory_controller')}", "/tron_collector"])
        security["expected_rates_hz"] = dict(SECURITY["expected_rates_hz"])
        security["expected_rates_hz"].setdefault(cmd_topic, (0.0, 5.0))
        security["expected_nodes"] = list(SECURITY["expected_nodes"]) + ["/safety_plc", f"/{robot.ros2.get('controller', 'joint_trajectory_controller')}"]
        security["expected_participants"] = list(SECURITY["expected_participants"]) + ["safety_plc", robot.ros2.get("controller", "joint_trajectory_controller")]
        security["command_topic"] = cmd_topic
        p = self.platform
        identity = robot.identity()
        identity["limits"]["tracking_notice"] = safety["joint_tracking_notice_rad"]
        identity["limits"]["tracking_alert"] = safety["joint_tracking_alert_rad"]
        p.configure(db, identity, scenario_environment(sc), safety, security, sc.faults)
        try:
            from backend.pipeline.twin.shadow_sim import ShadowSim
            p.state.shadow = ShadowSim(scene, robot)
        except Exception:
            p.state.shadow = None
        self.current = {"id": session_id, "robot": robot_id, "scenario": scenario_id, "adapter": adapter, "scene": str(scene),
                        "db": db, "started_at": time.time(), "targets": {k: v["ee"] for k, v in wp.items()}}
        p.session = self.current
        p.store.append(make_event(EventType.SessionStarted, Source.OPERATOR, ROBOT_ID,
                                  {"session_id": session_id, "robot": robot_id, "scenario": scenario_id, "adapter": adapter}))
        if adapter == "mujoco":
            from backend.io.collectors.mujoco_adapter import MujocoAdapter
            from backend.io.sim.renderer import RenderService
            ad = MujocoAdapter(p.ingest, robot, sc, scene, wp, ROBOT_ID)
            rd = RenderService(ad.m, ad.live_state, camera=SIM["camera"], width=SIM["width"], height=SIM["height"],
                               fps=SIM["render_hz"], quality=SIM["jpeg_quality"])
            p.adapter, p.renderer = ad, rd
            p.faults.register_adapter(ad)
            ad.start()
            rd.start()
        return self.current

    def info(self) -> dict | None:
        return self.current
