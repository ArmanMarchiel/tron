"""MuJoCo adapter: physics ground truth + a simulated ROS 2 runtime + cell devices for a scenario.

MuJoCo is the physics engine.  Around it this adapter plays every software and cell role a real
deployment has: a **motion planner node** that executes the scenario's task steps, a **joint-trajectory
controller** (minimum-jerk, with a safety-PLC protective stop), a **joint-state broadcaster**, the ROS
graph / DDS participant list, the **CNC machine** (PLC state machine behind a machine adapter), the
**operator** (articulated, scripted behaviours) and the **area scanner**.

Everything leaves as normalised events through the same ingest path an external adapter uses, under
distinct sources:  "mujoco/physics" (ground truth), "mujoco/ros2" (software state), "machine".

Fault scenarios are applied *inside* this adapter exactly where a real fault would act (actuator,
encoder, command path, planner belief, machine interlock, sensor, payload).
"""
from __future__ import annotations

import math
import queue
import threading
import time

import mujoco
import numpy as np

from backend.pipeline.events.event_model import EventType, make_event
from backend.io.machines.cnc import DOOR_OPEN_POS, CNCMachine
from backend.conf.registry import RobotProfile
from backend.io.sim.human import HumanOperator
from backend.io.collectors.base import Ingest
from backend.io.collectors.machine_adapter import MachineAdapter

PHYS = "mujoco/physics"
ROS = "mujoco/ros2"

NODES = ["/motion_planner", "/joint_trajectory_controller", "/joint_state_broadcaster", "/camera_driver",
         "/robot_state_publisher", "/safety_plc", "/tron_collector"]
OBSTACLE_GEOM_PREFIXES = ("cnc_", "obstacle", "raw_tray", "finished_tray", "bench")


def minjerk(tau: float) -> tuple[float, float]:
    tau = min(1.0, max(0.0, tau))
    return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5, 30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4


def _point_segment_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / max(1e-9, np.dot(ab, ab)), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


class MujocoAdapter:
    name = "mujoco"

    def __init__(self, ingest: Ingest, robot: RobotProfile, scenario, scene_path, waypoints: dict, robot_id: str):
        self.ingest = ingest
        self.robot = robot
        self.scenario = scenario
        self.robot_id = robot_id
        self.m = mujoco.MjModel.from_xml_path(str(scene_path))
        self.d = mujoco.MjData(self.m)
        self.lock = threading.Lock()
        self.wp = waypoints          # target name -> {"q": [...], "ee": [...]}
        self.n = robot.n
        self.qadr = [self.m.jnt_qposadr[self.m.joint(j).id] for j in robot.joints]
        self.dadr = [self.m.jnt_dofadr[self.m.joint(j).id] for j in robot.joints]
        self.act_ids = [self.m.actuator(a).id for a in robot.actuators]
        self.grip_id = self.m.actuator(robot.gripper_actuator).id
        self.grip_open = float(robot.gripper.get("ctrl_open", 255))
        self.grip_closed = float(robot.gripper.get("ctrl_closed", 0))
        self.ee_site = self.m.site("ee_site").id
        self.ee_body = self.m.body(robot.ee_body).id if robot.gripper.get("type") != "builtin" else self.m.body("tron_gripper").id
        self.tool_mass0 = float(self.m.body_mass[self.ee_body])
        self.tool_inertia0 = self.m.body_inertia[self.ee_body].copy()
        arm_bodies = set()
        b = self.ee_body
        while b != 0:
            arm_bodies.add(b); b = self.m.body_parentid[b]
        self.arm_body_ids = arm_bodies
        for i in range(self.m.nbody):
            p = self.m.body_parentid[i]
            if p in arm_bodies and i not in arm_bodies and self.m.body(i).name.startswith(("tron_finger", "left_finger", "right_finger")):
                arm_bodies.add(i)
        self.arm_geoms = [g for g in range(self.m.ngeom) if self.m.geom_bodyid[g] in arm_bodies and (self.m.geom_contype[g] or self.m.geom_conaffinity[g])]
        self.obstacle_geoms = {g: self.m.geom(g).name for g in range(self.m.ngeom)
                               if self.m.geom(g).name.startswith(OBSTACLE_GEOM_PREFIXES) and self.m.geom(g).name not in ("cnc_door_window",)}
        self.block_bodies = {blk.id: self.m.body(blk.id).id for blk in scenario.blocks}
        self.prox_bodies = sorted(arm_bodies)[-6:]
        # Menagerie grippers squeeze with ~3 N; scale the tendon actuator so a 0.6 kg block can be held (real Panda: 70 N)
        g = self.grip_id
        if abs(self.m.actuator_biasprm[g, 1] + 100.0) < 1e-6:
            self.m.actuator_gainprm[g, 0] *= 5.0
            self.m.actuator_biasprm[g, 1] *= 5.0
            self.m.actuator_biasprm[g, 2] *= 5.0
        self.kp0 = self.m.actuator_gainprm[:, 0].copy()
        self.bias0 = self.m.actuator_biasprm.copy()
        # reset to home
        for a, v in zip(self.qadr, robot.home):
            self.d.qpos[a] = v
        for i, a in enumerate(self.act_ids):
            self.d.ctrl[a] = robot.home[i]
        self.d.ctrl[self.grip_id] = self.grip_open
        mujoco.mj_forward(self.m, self.d)
        # cell devices
        self.machine = None
        self.machine_adapter = None
        if scenario.machine:
            self.machine = CNCMachine(scenario.machine.id, scenario.machine.cycle_time_s, scenario.machine.allowed_clients, part_loaded=True)
            self.machine_adapter = MachineAdapter(ingest, self.machine, robot_id, scenario.machine.protocol)
            self.door_q = self.m.jnt_qposadr[self.m.joint("cnc_door_joint").id]
            self.door_d = self.m.jnt_dofadr[self.m.joint("cnc_door_joint").id]
            self.jaw_q = [self.m.jnt_qposadr[self.m.joint(j).id] for j in ("cnc_jaw_l_joint", "cnc_jaw_r_joint")]
            self.jaw_d = [self.m.jnt_dofadr[self.m.joint(j).id] for j in ("cnc_jaw_l_joint", "cnc_jaw_r_joint")]
            self.chuck_site = self.m.site("cnc_chuck_site").id
            self.chuck_pos = np.array(scenario.machine.pose) + np.array(scenario.machine.chuck_offset)
        fields = [(s.field_min, s.field_max) for s in scenario.sensors if s.type == "area_scanner"]
        self.scanner = next((s for s in scenario.sensors if s.type == "area_scanner"), None)
        self.human = HumanOperator(self.m, self.d, scenario.human, fields) if scenario.human else None
        # controller / planner state
        self.traj: dict | None = None
        self.speed_factor = 1.0
        self.reached_since: float | None = None
        self.protective_stop = False
        self._pstop_hold: np.ndarray | None = None
        self._pstop_since: float | None = None
        self.task = TaskRunner(self, scenario)
        # faults
        self.fault: dict | None = None
        self._pending: queue.Queue = queue.Queue()
        self._rogue_active = False
        self.encoder_offset: tuple[int, float] | None = None
        self.scanner_blind = False
        self.machine_spoof: str | None = None
        self._gain_ramp: dict | None = None
        # bookkeeping
        self._stop = False
        self._t_emit: dict[str, float] = {}
        self._cmd_count = 0
        self._cmd_window_start = time.time()
        self._cmd_rate = 0.0
        self.collision = False
        self.collision_with: str | None = None
        self.sensed_intrusion = False
        self.held: str | None = None
        self.thread = threading.Thread(target=self.run, name="tron-mujoco", daemon=True)

    # ------------------------------------------------------------------ external API
    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self._stop = True

    def live_state(self):
        with self.lock:
            return self.d.qpos.copy(), ((self.d.mocap_pos.copy(), self.d.mocap_quat.copy()) if self.m.nmocap else None)

    def apply_fault(self, scenario: dict) -> None:
        self._pending.put(("apply", scenario))

    def clear_fault(self) -> None:
        self._pending.put(("clear", None))

    # ------------------------------------------------------------------ faults (sim thread)
    def _apply_pending(self, now: float) -> None:
        try:
            while True:
                op, sc = self._pending.get_nowait()
                self._do_apply(sc, now) if op == "apply" else self._do_clear(now)
        except queue.Empty:
            pass

    def _set_speed(self, factor: float, now: float) -> None:
        if self.traj:
            tau = (now - self.traj["start"]) / (self.traj["duration"] / self.speed_factor)
            self.traj["start"] = now - tau * (self.traj["duration"] / factor)
        self.speed_factor = factor

    def _do_apply(self, sc: dict, now: float) -> None:
        self.fault = sc
        p = sc["params"]
        if "speed_factor" in p:
            self._set_speed(p["speed_factor"], now)
        if p.get("rogue_node"):
            self._rogue_active = True
            self._emit_node_started(p["rogue_node"])
        if "encoder_offset" in p:
            self.encoder_offset = (int(p["encoder_offset"]["joint"]), float(p["encoder_offset"]["rad"]))
        if "actuator_gain_scale" in p:
            for j in p["joints"]:
                a = self.act_ids[j]
                self.m.actuator_gainprm[a, 0] = self.kp0[a] * p["actuator_gain_scale"]
                self.m.actuator_biasprm[a, 1] = self.bias0[a, 1] * p["actuator_gain_scale"]
        if p.get("goal"):
            self.task.override_goal(p["goal"], now, disable_checks=bool(p.get("disable_collision_check")))
        if p.get("machine_state_spoof") and self.machine:
            self.machine_spoof = p["machine_state_spoof"]
            self.machine.interlock_bypass = True
            if self.machine.state != "RUNNING":
                # the cell PLC (re)starts the spindle: the machine genuinely runs while the planner is fed COMPLETE
                self.machine.state, self.machine.cycle_started_at, self.machine.cycle_time_s = "RUNNING", now, max(self.machine.cycle_time_s, sc.get("duration_s", 20.0) + 5.0)
                if self.machine_adapter:
                    self.machine_adapter.tick(now)
        if p.get("human_behaviour") and self.human:
            self.human.set_behaviour(p["human_behaviour"], now)
        if p.get("scanner_blind"):
            self.scanner_blind = True
        if p.get("extra_payload_kg"):
            scale = (self.tool_mass0 + float(p["extra_payload_kg"])) / max(self.tool_mass0, 1e-6)
            self.m.body_mass[self.ee_body] = self.tool_mass0 + float(p["extra_payload_kg"])
            self.m.body_inertia[self.ee_body] = self.tool_inertia0 * scale
            self._set_const_keep_state()

    def _set_const_keep_state(self) -> None:
        """mj_setConst evaluates the model at qpos0 and overwrites the state; keep the live state."""
        q, v, c = self.d.qpos.copy(), self.d.qvel.copy(), self.d.ctrl.copy()
        mujoco.mj_setConst(self.m, self.d)
        self.d.qpos[:], self.d.qvel[:], self.d.ctrl[:] = q, v, c
        mujoco.mj_forward(self.m, self.d)

    def _set_const_keep_state(self) -> None:
        """mj_setConst evaluates the model at qpos0 and overwrites the state; keep the live state."""
        q, v, c = self.d.qpos.copy(), self.d.qvel.copy(), self.d.ctrl.copy()
        mujoco.mj_setConst(self.m, self.d)
        self.d.qpos[:], self.d.qvel[:], self.d.ctrl[:] = q, v, c
        mujoco.mj_forward(self.m, self.d)

    def _do_clear(self, now: float) -> None:
        if not self.fault:
            return
        p = self.fault["params"]
        self._set_speed(1.0, now)
        if p.get("rogue_node"):
            self._rogue_active = False
            self.ingest([make_event(EventType.ROSNodeStopped, ROS, self.robot_id, {"node": p["rogue_node"]})])
            self.traj = None
        self.encoder_offset = None
        if "actuator_gain_scale" in p:
            self._gain_ramp = {"t0": now, "dur": 1.0, "joints": [self.act_ids[j] for j in p["joints"]], "from": p["actuator_gain_scale"]}
        if p.get("goal"):
            self.traj = None
            self.task.clear_override(now)
        if p.get("machine_state_spoof") and self.machine:
            self.machine_spoof = None
            self.machine.interlock_bypass = False
            self.machine.cycle_time_s = self.scenario.machine.cycle_time_s
            if self.machine.state == "ALARM":
                self.machine_adapter.command("reset", "cell_plc", now)
        self.scanner_blind = False
        if p.get("extra_payload_kg"):
            self.m.body_mass[self.ee_body] = self.tool_mass0
            self.m.body_inertia[self.ee_body] = self.tool_inertia0
            self._set_const_keep_state()
        self.task.recover(now)
        self.fault = None

    # ------------------------------------------------------------------ boot / graph
    def _emit_node_started(self, node: str) -> None:
        self.ingest([make_event(EventType.ROSNodeStarted, ROS, self.robot_id, {"node": node, "namespace": "/"})])

    def _boot(self) -> None:
        for n in NODES:
            self._emit_node_started(n)
        self.ingest([make_event(EventType.ControllerStateChanged, ROS, self.robot_id, {"name": "joint_trajectory_controller", "state": "active"})])
        self._emit_graph()
        self._emit_network()
        if self.machine_adapter:
            self.machine_adapter.tick(time.time())

    def _topics(self) -> dict:
        cmd = self.robot.ros2.get("command_topic", "/joint_trajectory_controller/joint_trajectory")
        ctrl = self.robot.ros2.get("controller", "joint_trajectory_controller")
        return {
            cmd: {"type": "trajectory_msgs/msg/JointTrajectory", "publishers": ["/motion_planner"], "subscribers": [f"/{ctrl}", "/tron_collector"], "rate": self._cmd_rate},
            "/joint_states": {"type": "sensor_msgs/msg/JointState", "publishers": ["/joint_state_broadcaster"], "subscribers": ["/motion_planner", "/robot_state_publisher", "/tron_collector"], "rate": 20.0},
            f"/{ctrl}/state": {"type": "control_msgs/msg/JointTrajectoryControllerState", "publishers": [f"/{ctrl}"], "subscribers": ["/tron_collector"], "rate": 10.0},
            "/camera/image_raw": {"type": "sensor_msgs/msg/Image", "publishers": ["/camera_driver"], "subscribers": ["/tron_collector"], "rate": 15.0},
            "/tf": {"type": "tf2_msgs/msg/TFMessage", "publishers": ["/robot_state_publisher"], "subscribers": ["/motion_planner", "/tron_collector"], "rate": 20.0},
            "/safety/scanner": {"type": "sensor_msgs/msg/LaserScan", "publishers": ["/safety_plc"], "subscribers": [f"/{ctrl}", "/tron_collector"], "rate": 10.0},
        }

    def _emit_graph(self) -> None:
        evs = []
        cmd_topic = self.robot.ros2.get("command_topic", "/joint_trajectory_controller/joint_trajectory")
        for topic, info in self._topics().items():
            pubs = list(info["publishers"])
            if topic == cmd_topic and self._rogue_active:
                pubs.append(self.fault["params"]["rogue_node"])
            evs.append(make_event(EventType.ROSTopicObserved, ROS, self.robot_id,
                                  {"topic": topic, "type": info["type"], "publishers": pubs, "subscribers": info["subscribers"], "rate_hz": round(info["rate"], 2)}))
        self.ingest(evs)

    def _emit_network(self) -> None:
        parts = [{"name": n.strip("/"), "guid": f"01.0f.{i:02x}.5a", "address": "127.0.0.1", "domain_id": 0} for i, n in enumerate(NODES)]
        if self._rogue_active:
            parts.append({"name": self.fault["params"]["rogue_node"].strip("/"), "guid": "01.0f.e7.9c", "address": "192.168.1.77", "domain_id": 0})
        self.ingest([make_event(EventType.NetworkConnectionObserved, ROS, self.robot_id,
                                {"participants": parts, "connections": [{"from": p["name"], "to": "dds", "transport": "udp"} for p in parts]})])

    # ------------------------------------------------------------------ readers (ground truth)
    def q(self) -> np.ndarray:
        return np.array([self.d.qpos[a] for a in self.qadr])

    def qd(self) -> np.ndarray:
        return np.array([self.d.qvel[a] for a in self.dadr])

    def effort(self) -> np.ndarray:
        return np.array([self.d.actuator_force[a] for a in self.act_ids])

    def _reported_q(self) -> np.ndarray:
        q = self.q()
        if self.encoder_offset:
            q[self.encoder_offset[0]] += self.encoder_offset[1]
        return q

    def _contacts(self) -> tuple[bool, str | None]:
        arm = set(self.arm_geoms)
        self.collision_arm_geom = None
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            g1, g2 = c.geom1, c.geom2
            if g1 in arm and g2 in self.obstacle_geoms:
                self.collision_arm_geom = self.m.body(self.m.geom_bodyid[g1]).name
                return True, self.obstacle_geoms[g2]
            if g2 in arm and g1 in self.obstacle_geoms:
                self.collision_arm_geom = self.m.body(self.m.geom_bodyid[g2]).name
                return True, self.obstacle_geoms[g1]
        return False, None

    def _nearest_obstacle(self) -> float:
        best = 1.0
        fromto = np.zeros(6)
        for g in self.arm_geoms:
            for og in self.obstacle_geoms:
                dist = mujoco.mj_geomDistance(self.m, self.d, g, og, best, fromto)
                if dist < best:
                    best = dist
        return float(max(0.0, best))

    def _nearest_human(self) -> float | None:
        if not self.human:
            return None
        best = 9.0
        fromto = np.zeros(6)
        for g in self.arm_geoms:
            for hg in self.human.geoms:
                dist = mujoco.mj_geomDistance(self.m, self.d, g, hg, best, fromto)
                if dist < best:
                    best = dist
        return float(max(0.0, best))

    # ------------------------------------------------------------------ grasp (weld) handling
    def grasp_attach(self) -> str | None:
        """Weld the block that sits between the fingers to the tool body; returns the block id or None."""
        ee = self.d.site_xpos[self.ee_site]
        for bid, b in self.block_bodies.items():
            p = self.d.xpos[b]
            if math.hypot(p[0] - ee[0], p[1] - ee[1]) < 0.035 and abs(p[2] - ee[2]) < 0.045:
                try:
                    eq = self.m.equality(f"grasp_{bid}").id
                except KeyError:
                    return None
                p1, q1 = self.d.xpos[self.ee_body], self.d.xquat[self.ee_body]
                q1inv = np.zeros(4); mujoco.mju_negQuat(q1inv, q1)
                rel = np.zeros(3); mujoco.mju_rotVecQuat(rel, p - p1, q1inv)
                rq = np.zeros(4); mujoco.mju_mulQuat(rq, q1inv, self.d.xquat[b])
                self.m.eq_data[eq, 0:3] = 0.0
                self.m.eq_data[eq, 3:6] = rel
                self.m.eq_data[eq, 6:10] = rq
                self.d.eq_active[eq] = 1
                self.held = bid
                return bid
        return None

    def grasp_release(self) -> None:
        for bid in self.block_bodies:
            try:
                self.d.eq_active[self.m.equality(f"grasp_{bid}").id] = 0
            except KeyError:
                pass
        self.held = None

    def held_mass(self) -> float:
        if not self.held:
            return 0.0
        return float(self.m.body_mass[self.block_bodies[self.held]])

    def machine_view(self) -> dict | None:
        """What the planner believes about the machine (spoofable)."""
        if not self.machine:
            return None
        tags = self.machine.tags()
        if self.machine_spoof:
            tags = dict(tags, state=self.machine_spoof)
        return tags

    # ------------------------------------------------------------------ controller (software)
    def send_trajectory(self, goal_name: str, q_target: np.ndarray, duration: float, source: str, gripper: float | None = None) -> None:
        cmd_topic = self.robot.ros2.get("command_topic", "/joint_trajectory_controller/joint_trajectory")
        now = time.time()
        if source == "/motion_planner":
            max_delta = float(np.max(np.abs(q_target - self.q())))
            duration = max(duration, 1.875 * max_delta / 0.6)   # planner time-parameterisation: peak <= 0.6 rad/s
        self.traj = {"name": goal_name, "from": self.q().copy(), "to": np.array(q_target, dtype=float), "start": now, "duration": duration,
                     "gripper": gripper if gripper is not None else float(self.d.ctrl[self.grip_id]), "source": source}
        self.reached_since = None
        self.ingest([make_event(EventType.CommandReceived, ROS, self.robot_id,
                                {"topic": cmd_topic, "source_node": source, "goal_name": goal_name,
                                 "positions": [float(x) for x in q_target], "duration_s": duration,
                                 "start_positions": [round(float(x), 5) for x in self._reported_q()], "joint_names": self.robot.joints})])
        self._cmd_count += 1

    def _rogue_step(self, now: float) -> None:
        p = self.fault["params"]
        tgt = self.wp[p["goal"]]
        if self.traj is None or self.traj["source"] != p["rogue_node"]:
            self.send_trajectory("rogue", np.array(tgt["q"]), p.get("duration_s", 2.0), p["rogue_node"])
        else:
            cmd_topic = self.robot.ros2.get("command_topic", "/joint_trajectory_controller/joint_trajectory")
            self.ingest([make_event(EventType.CommandReceived, ROS, self.robot_id,
                                    {"topic": cmd_topic, "source_node": p["rogue_node"], "goal_name": "rogue",
                                     "positions": list(map(float, tgt["q"])), "duration_s": p.get("duration_s", 2.0),
                                     "start_positions": [round(float(x), 5) for x in self._reported_q()], "joint_names": self.robot.joints})])
            self._cmd_count += 1

    def _controller_step(self, now: float) -> tuple[float, str]:
        # safety PLC: sensed intrusion -> protective stop (freeze targets)
        if self.sensed_intrusion and self.scanner and self.scanner.protective_stop:
            if not self.protective_stop:
                self.protective_stop, self._pstop_since = True, now
                self._pstop_hold = self.q().copy()
                if self.traj:
                    self.traj["pstop_tau"] = (now - self.traj["start"]) / (self.traj["duration"] / self.speed_factor)
            for i, a in enumerate(self.act_ids):
                self.d.ctrl[a] = self._pstop_hold[i]
            return (self.traj and min(1.0, self.traj.get("pstop_tau", 0.0))) or 0.0, "protective_stop"
        if self.protective_stop:
            self.protective_stop = False
            if self.traj:   # resume with phase continuity
                self.traj["start"] = now - self.traj.get("pstop_tau", 0.0) * (self.traj["duration"] / self.speed_factor)
        if self.traj is None:
            return 0.0, "idle"
        t = self.traj
        tau = (now - t["start"]) / (t["duration"] / self.speed_factor)
        s, _ = minjerk(tau)
        target = t["from"] + (t["to"] - t["from"]) * s
        for i, a in enumerate(self.act_ids):
            self.d.ctrl[a] = target[i]
        self.d.ctrl[self.grip_id] = t["gripper"]
        if tau >= 1.0:
            err = float(np.max(np.abs(self.q() - t["to"])))
            if self.reached_since is None:
                self.reached_since = now
            if err < 0.02 or now - self.reached_since > 1.5:
                self.ingest([make_event(EventType.TrajectoryGoalReached, ROS, self.robot_id,
                                        {"goal_name": t["name"], "final_error_rad": round(err, 4), "source_node": t["source"]})])
                self.traj = None
                return 1.0, "idle"
        return min(1.0, tau), "active"

    # ------------------------------------------------------------------ cell devices (physics side)
    def _devices_step(self, dt: float, now: float) -> None:
        if self.machine:
            # door slide: kinematic drive toward the PLC's target at 0.3 m/s
            cur = self.d.qpos[self.door_q]
            tgt = self.machine.door_target
            step = min(abs(tgt - cur), 0.3 * dt)
            self.d.qpos[self.door_q] = cur + math.copysign(step, tgt - cur) if abs(tgt - cur) > 1e-6 else tgt
            self.d.qvel[self.door_d] = 0.0
            self.machine.set_door_position(float(self.d.qpos[self.door_q]))
            for qa, da in zip(self.jaw_q, self.jaw_d):
                self.d.qpos[qa] = self.machine.jaw_target
                self.d.qvel[da] = 0.0
            # a part is "loaded" when a block sits on the chuck
            loaded = any(np.linalg.norm(self.d.xpos[b][:2] - self.chuck_pos[:2]) < 0.05 and abs(self.d.xpos[b][2] - self.chuck_pos[2]) < 0.05
                         for b in self.block_bodies.values())
            self.machine.set_part_loaded(loaded)
        if self.human:
            self.human.step(dt, now)

    def _scanner_reading(self) -> bool:
        truth = self.human.feet_in_field() if self.human else False
        return False if self.scanner_blind else truth

    # ------------------------------------------------------------------ emitters
    def _due(self, key: str, hz: float, now: float) -> bool:
        if now - self._t_emit.get(key, 0.0) >= 1.0 / hz:
            self._t_emit[key] = now
            return True
        return False

    # ------------------------------------------------------------------ main loop (thread)
    def run(self) -> None:
        self._boot()
        dt = self.m.opt.timestep
        wall0, sim_t = time.time(), 0.0
        last_ctrl = -1.0
        progress, cstate = 0.0, "idle"
        while not self._stop:
            now = time.time()
            self._apply_pending(now)
            if self._gain_ramp:
                r = self._gain_ramp
                f = min(1.0, (now - r["t0"]) / r["dur"])
                scale = r["from"] + (1.0 - r["from"]) * f
                for a in r["joints"]:
                    self.m.actuator_gainprm[a, 0] = self.kp0[a] * scale
                    self.m.actuator_biasprm[a, 1] = self.bias0[a, 1] * scale
                if f >= 1.0:
                    self._gain_ramp = None
            target_t = now - wall0
            steps = 0
            with self.lock:
                while sim_t < target_t and steps < 200:
                    if sim_t - last_ctrl >= 0.02:   # 50 Hz control
                        last_ctrl = sim_t
                        self.sensed_intrusion = self._scanner_reading()
                        self.task.step(now)
                        if self._rogue_active and self._due("rogue", self.fault["params"].get("rate_hz", 10.0), now):
                            self._rogue_step(now)
                        progress, cstate = self._controller_step(now)
                    self._devices_step(dt, now)
                    mujoco.mj_step(self.m, self.d)
                    sim_t += dt
                    steps += 1
                if steps >= 200:
                    wall0 = now - sim_t
            if now - self._cmd_window_start >= 2.0:
                self._cmd_rate = self._cmd_count / (now - self._cmd_window_start)
                self._cmd_count, self._cmd_window_start = 0, now
            if self.machine_adapter and self._due("machine", 20.0, now):
                self.machine_adapter.tick(now)

            if self._due("joint_states", 20.0, now):
                self.ingest([make_event(EventType.PositionObserved, ROS, self.robot_id, {
                    "topic": "/joint_states", "joint_names": self.robot.joints,
                    "position": [round(float(x), 5) for x in self._reported_q()],
                    "velocity": [round(float(x), 4) for x in self.qd()],
                    "effort": [round(float(x), 3) for x in self.effort()]})])
            if self._due("ctrl_state", 10.0, now):
                self.ingest([make_event(EventType.CommandExecuted, ROS, self.robot_id, {
                    "targets": [round(float(self.d.ctrl[a]), 5) for a in self.act_ids], "gripper": float(self.d.ctrl[self.grip_id]),
                    "state": cstate, "progress": round(progress, 3), "controller": self.robot.ros2.get("controller", "joint_trajectory_controller"),
                    "goal_name": self.traj["name"] if self.traj else None})])
            if self._due("scanner", 10.0, now) and self.scanner:
                self.ingest([make_event(EventType.SensorObservation, ROS, self.robot_id, {
                    "sensor": "area_scanner", "id": self.scanner.id, "topic": "/safety/scanner", "intrusion": self.sensed_intrusion})])
            if self._due("truth", 10.0, now):
                with self.lock:
                    self.collision, self.collision_with = self._contacts()
                    ee = self.d.site_xpos[self.ee_site].copy()
                    ee_v = float(np.linalg.norm(self.d.sensordata[3:6]))
                    d_obs, d_hum = self._nearest_obstacle(), self._nearest_human()
                    q, qd, eff = self.q(), self.qd(), self.effort()
                    in_field = self.human.feet_in_field() if self.human else False
                if self.collision:
                    self.ingest([make_event(EventType.CollisionDetected, PHYS, self.robot_id,
                                            {"with": self.collision_with, "arm_link": getattr(self, "collision_arm_geom", None),
                                             "ee": [round(float(x), 4) for x in ee], "task_step": self.task.cur.id if self.task.cur else None})])
                self.ingest([make_event(EventType.RobotStateObserved, PHYS, self.robot_id, {
                    "frame": "ground_truth", "joint_names": self.robot.joints,
                    "position": [round(float(x), 5) for x in q], "velocity": [round(float(x), 4) for x in qd],
                    "effort": [round(float(x), 3) for x in eff],
                    "ee": {"x": round(float(ee[0]), 4), "y": round(float(ee[1]), 4), "z": round(float(ee[2]), 4)}, "ee_speed": round(ee_v, 4),
                    "collision": self.collision, "collision_with": self.collision_with,
                    "nearest_obstacle_distance": round(d_obs, 4), "nearest_human_distance": (round(d_hum, 4) if d_hum is not None else None),
                    "human_in_field": in_field, "protective_stop": self.protective_stop,
                    "gripper": float(self.d.ctrl[self.grip_id]), "held": self.held,
                    "payload_kg": round(float(self.m.body_mass[self.ee_body] - self.tool_mass0 + self.held_mass()), 2)})])
            if self._due("env", 2.0, now):
                humans = [{"id": self.human.id, "x": self.human.x, "y": self.human.y, "z": self.human.z, "behaviour": self.human.behaviour}] if self.human else []
                blocks = [{"id": bid, "pos": [round(float(x), 3) for x in self.d.xpos[b]]} for bid, b in self.block_bodies.items()]
                self.ingest([make_event(EventType.EnvironmentObserved, PHYS, self.robot_id, {"humans": humans, "blocks": blocks})])
            if self._due("camera", 1.0, now):
                self.ingest([make_event(EventType.SensorObservation, ROS, self.robot_id, {"sensor": "camera", "topic": "/camera/image_raw", "fps": 15})])
            if self._due("graph", 1.0, now):
                self._emit_graph()
            if self._due("net", 2.0, now):
                self._emit_network()
            time.sleep(0.004)


class TaskRunner:
    """The motion-planner node: executes the scenario's task steps against the planner's *belief* of
    the machine state (spoofable) and the robot's reported joint state."""

    def __init__(self, adapter: MujocoAdapter, scenario):
        self.a = adapter
        self.sc = scenario
        self.steps = scenario.task.steps
        self.index = -1
        self.cur = None
        self.started = 0.0
        self.status = "idle"
        self.pause_until = 0.0
        self.tray_counters: dict[str, int] = {}
        self.override: dict | None = None
        self.checks_disabled = False
        self._dwell_until = 0.0

    # ---------------------------------------------------------------- targets
    def resolve(self, target: str, height_offset: float = 0.0) -> tuple[str, np.ndarray]:
        key = target
        if target.endswith(".next"):
            tray = target[:-5]
            i = self.tray_counters.get(tray, 0)
            spec = next(t for t in self.sc.trays if t.id == tray)
            key = f"{tray}.slot{i % max(1, len(spec.slots))}"
        if height_offset:
            key = f"{key}+{height_offset:g}"
        return key, np.array(self.a.wp[key]["q"])

    def _advance_tray(self, target: str) -> None:
        if target.endswith(".next"):
            tray = target[:-5]
            self.tray_counters[tray] = self.tray_counters.get(tray, 0) + 1

    # ---------------------------------------------------------------- belief helpers
    def _eval(self, cond: str | None) -> bool:
        if not cond:
            return True
        cond = cond.replace(" ", "")
        mv = self.a.machine_view() or {}
        if cond.startswith("machine.") and "==" in cond:
            key, val = cond[len("machine."):].split("==")
            return str(mv.get(key)) == val
        return True

    # ---------------------------------------------------------------- faults
    def override_goal(self, goal: str, now: float, disable_checks: bool) -> None:
        self.override = {"goal": goal, "resend": True}
        self.checks_disabled = disable_checks
        self._finish(now, "aborted", "overridden by planner goal")
        self.a.traj = None

    def clear_override(self, now: float) -> None:
        self.override = None
        self.checks_disabled = False

    def recover(self, now: float) -> None:
        """After any fault: retreat straight up (if a retreat target exists), then restart the task."""
        self.override = None
        self.status = "recover"
        self.pause_until = now + 0.5
        self.index = -1

    # ---------------------------------------------------------------- execution
    def step(self, now: float) -> None:
        if self.a._rogue_active:
            return
        if self.override:
            if self.a.traj is None and now >= self.pause_until:
                key = self.override["goal"]
                q = np.array(self.a.wp[key]["q"])
                self._emit_goal(key, q, 3.0, now, note="planner override")
                self.a.send_trajectory(key, q, 3.0, "/motion_planner")
                self.pause_until = now + 4.0
            return
        if self.status == "recover":
            if now < self.pause_until:
                return
            key = "machine.front" if self.a.wp.get("machine.front") else "home"
            self._emit_goal(key, np.array(self.a.wp[key]["q"]), 3.0, now, note="recovery")
            self.a.send_trajectory(key, np.array(self.a.wp[key]["q"]), 3.0, "/motion_planner")
            self.status = "recovering"
            return
        if self.status == "recovering":
            if self.a.traj is None:
                self.status = "idle"
                self.pause_until = now + 0.5
            return
        if self.cur is None:
            if now >= self.pause_until:
                self._start_next(now)
            return
        # active step: check completion / timeout
        s = self.cur
        if s.action == "move":
            if self.a.traj is None:
                self._finish(now, "completed")
        elif s.action == "gripper":
            if now >= self._dwell_until:
                self._finish(now, "completed")
        elif s.action == "machine":
            if self._eval(s.expect):
                self._finish(now, "completed")
            elif now - self.started > (s.timeout_s or s.duration_s + 4.0):
                self._finish(now, "failed", f"machine did not reach '{s.expect}' (belief: {self.a.machine_view()})")
        elif s.action == "wait":
            if self._eval(s.precondition):
                self._finish(now, "completed")
            elif s.timeout_s and now - self.started > s.timeout_s:
                self._finish(now, "failed", f"precondition '{s.precondition}' not met within {s.timeout_s} s")

    def _start_next(self, now: float) -> None:
        self.index += 1
        if self.index >= len(self.steps):
            if not self.sc.task.loop:
                self.status = "done"
                return
            self.index = 0
        s = self.steps[self.index]
        self.cur, self.started, self.status = s, now, "active"
        dur = s.duration_s
        key = q = None
        if s.action == "move":
            key, q = self.resolve(s.target, s.height_offset)
            dur = max(dur, 1.875 * float(np.max(np.abs(q - self.a.q()))) / 0.6)   # the planner's time-parameterised duration
        payload = {"scenario": self.sc.id, "step": s.id, "index": self.index, "action": s.action, "target": s.target, "command": s.command,
                   "duration_s": round(dur, 2), "nominal_duration_s": s.duration_s, "timeout_s": s.timeout_s, "precondition": s.precondition, "expect": s.expect,
                   "machine_belief": self.a.machine_view(), "source_node": "/motion_planner"}
        self.a.ingest([make_event(EventType.TaskStepStarted, ROS, self.a.robot_id, payload)])
        if s.action == "move":
            grip = None
            if s.gripper:
                grip = self.a.grip_open if s.gripper == "open" else self.a.grip_closed
                if s.gripper == "open":
                    self.a.grasp_release()
            self._emit_goal(key, q, dur, now)
            self.a.send_trajectory(key, q, dur, "/motion_planner", gripper=grip)
        elif s.action == "gripper":
            val = self.a.grip_open if s.gripper == "open" else self.a.grip_closed
            self.a.d.ctrl[self.a.grip_id] = val
            if self.a.traj:
                self.a.traj["gripper"] = val
            if s.gripper == "closed":
                self.a.grasp_attach()
            else:
                self.a.grasp_release()
            self._dwell_until = now + s.duration_s
            # a placed/picked part advances the tray counter of the previous move
            prev = self.steps[self.index - 1] if self.index > 0 else None
            if prev and prev.action == "move" and prev.target and prev.target.endswith(".next"):
                self._advance_tray(prev.target)
        elif s.action == "machine":
            if self.a.machine_adapter:
                self.a.machine_adapter.command(s.command, "/motion_planner", now)
        elif s.action == "wait":
            pass

    def _emit_goal(self, key: str, q: np.ndarray, duration: float, now: float, note: str | None = None) -> None:
        ee = self.a.wp.get(key, {}).get("ee")
        self.a.ingest([make_event(EventType.TrajectoryGoalReceived, ROS, self.a.robot_id,
                                  {"goal_name": key, "positions": [float(x) for x in q], "duration_s": duration, "ee_target": ee,
                                   "source_node": "/motion_planner", "task_step": self.cur.id if self.cur else None, "note": note})])

    def _finish(self, now: float, status: str, reason: str | None = None) -> None:
        if self.cur is None:
            return
        et = EventType.TaskStepCompleted if status == "completed" else EventType.TaskStepFailed
        self.a.ingest([make_event(et, ROS, self.a.robot_id, {"scenario": self.sc.id, "step": self.cur.id, "index": self.index,
                                                            "elapsed_s": round(now - self.started, 2), "reason": reason, "status": status})])
        if status == "failed":
            target = self.cur.on_fail
            if target:
                self.index = next((i for i, st in enumerate(self.steps) if st.id == target), 0) - 1
            elif self.cur.action == "wait":
                self.index = -1     # restart the cycle from the top
        self.cur = None
        self.status = "idle"
        self.pause_until = now + 0.3
