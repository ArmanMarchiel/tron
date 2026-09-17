"""ROS 2 collector node (ROS 2 Jazzy / Humble) for a ros2_control manipulator.

Runs *inside* the ROS 2 environment (next to a real arm, or a simulator with a ROS 2 bridge)
and forwards normalised events to the TRON platform over HTTP (``POST /api/ingest``).  The
platform itself never imports rclpy, so this file is the only ROS-specific code.

  /joint_states                                 (sensor_msgs/JointState)        -> PositionObserved (software belief)
  /joint_trajectory_controller/joint_trajectory (trajectory_msgs/JointTrajectory)-> CommandReceived (+ TrajectoryGoalReceived
                                                                                   when it comes from the planner)
  /joint_trajectory_controller/state            (control_msgs/JointTrajectoryControllerState) -> CommandExecuted
  ROS graph (polled at 1 Hz)                    -> ROSNodeStarted/Stopped, ROSTopicObserved (rates), NetworkConnectionObserved

Physical ground truth (RobotStateObserved) must come from a second source: a simulator's
state collector, a motion-capture system, or an independent measurement.  On a real robot with
no independent truth the platform still compares expected vs *reported* state.

  ros2 run tron_ros2_collector collector --ros-args -p platform_url:=http://<host>:8000 -p robot_id:=robot-001
"""
from __future__ import annotations

import json
import threading
import time
import urllib.request
import uuid
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

try:
    from control_msgs.msg import JointTrajectoryControllerState
except ImportError:  # control_msgs is optional
    JointTrajectoryControllerState = None

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
CMD_TOPIC = "/joint_trajectory_controller/joint_trajectory"


class PlatformClient:
    def __init__(self, url: str, batch_interval: float = 0.05):
        self.url = url.rstrip("/") + "/api/ingest"
        self.q: deque = deque()
        self.lock = threading.Lock()
        self.batch_interval = batch_interval
        threading.Thread(target=self._loop, daemon=True).start()

    def emit(self, event_type: str, source: str, entity_id: str, payload: dict, confidence: float = 1.0):
        with self.lock:
            self.q.append({"event_id": uuid.uuid4().hex[:16], "timestamp": time.time(), "event_type": event_type,
                           "source": source, "entity_id": entity_id, "payload": payload, "confidence": confidence})

    def _loop(self):
        while True:
            time.sleep(self.batch_interval)
            with self.lock:
                if not self.q:
                    continue
                batch = list(self.q); self.q.clear()
            req = urllib.request.Request(self.url, data=json.dumps({"events": batch}).encode(), headers={"Content-Type": "application/json"})
            try:
                urllib.request.urlopen(req, timeout=2.0).read()
            except Exception as e:
                print(f"[tron_collector] ingest failed ({e}); dropped {len(batch)} events")


class Collector(Node):
    SOURCE = "ros2"

    def __init__(self):
        super().__init__("tron_collector")
        self.declare_parameter("platform_url", "http://127.0.0.1:8000")
        self.declare_parameter("robot_id", "robot-001")
        self.declare_parameter("joint_names", JOINTS)
        self.declare_parameter("planner_node", "/motion_planner")
        self.declare_parameter("watch_topics", [CMD_TOPIC, "/joint_states", "/joint_trajectory_controller/state", "/tf", "/camera/image_raw"])
        g = lambda n: self.get_parameter(n).value
        self.robot_id = g("robot_id")
        self.joints = list(g("joint_names"))
        self.planner = g("planner_node")
        self.watch = list(g("watch_topics"))
        self.client = PlatformClient(g("platform_url"))
        self.known_nodes: set[str] = set()
        self.msg_counts: dict[str, int] = defaultdict(int)
        self.rate_window_start = time.time()
        self.last_joint_positions: list[float] | None = None
        self.create_subscription(JointState, "/joint_states", self.on_joints, 50)
        self.create_subscription(JointTrajectory, CMD_TOPIC, self.on_trajectory, 10)
        if JointTrajectoryControllerState is not None:
            self.create_subscription(JointTrajectoryControllerState, "/joint_trajectory_controller/state", self.on_ctrl_state, 20)
        self.create_timer(1.0, self.on_graph_timer)
        self.get_logger().info(f"tron collector forwarding to {g('platform_url')} for {self.robot_id}")

    def _ordered(self, names, values):
        idx = {n: i for i, n in enumerate(names)}
        return [float(values[idx[j]]) if j in idx and idx[j] < len(values) else 0.0 for j in self.joints]

    def on_joints(self, m: JointState):
        self.msg_counts["/joint_states"] += 1
        pos = self._ordered(m.name, m.position)
        self.last_joint_positions = pos
        self.client.emit("PositionObserved", self.SOURCE, self.robot_id, {
            "topic": "/joint_states", "joint_names": self.joints, "position": pos,
            "velocity": self._ordered(m.name, m.velocity) if m.velocity else [0.0] * len(self.joints),
            "effort": self._ordered(m.name, m.effort) if m.effort else [0.0] * len(self.joints)})

    def on_trajectory(self, m: JointTrajectory):
        self.msg_counts[CMD_TOPIC] += 1
        if not m.points:
            return
        last = m.points[-1]
        pubs = [f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in self.get_publishers_info_by_topic(CMD_TOPIC)]
        # rclpy does not expose the sender of an individual message: unambiguous with one publisher,
        # otherwise all candidates are reported and the graph-level rule flags the unexpected one.
        src = pubs[0] if len(pubs) == 1 else None
        dur = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9
        payload = {"topic": CMD_TOPIC, "source_node": src, "candidate_publishers": pubs, "goal_name": m.header.frame_id or "trajectory",
                   "positions": self._ordered(m.joint_names, last.positions), "duration_s": dur,
                   "start_positions": self.last_joint_positions or self._ordered(m.joint_names, m.points[0].positions), "joint_names": self.joints}
        if src == self.planner:
            self.client.emit("TrajectoryGoalReceived", self.SOURCE, self.robot_id,
                             {"goal_name": payload["goal_name"], "positions": payload["positions"], "duration_s": dur, "source_node": src})
        self.client.emit("CommandReceived", self.SOURCE, self.robot_id, payload, confidence=1.0 if src else 0.7)

    def on_ctrl_state(self, m):
        self.msg_counts["/joint_trajectory_controller/state"] += 1
        ref = getattr(m, "reference", None) or getattr(m, "desired", None)
        targets = self._ordered(m.joint_names, ref.positions) if ref is not None and ref.positions else None
        self.client.emit("CommandExecuted", self.SOURCE, self.robot_id, {
            "targets": targets, "state": "active", "controller": "joint_trajectory_controller"})

    def on_graph_timer(self):
        now = time.time()
        names = {f"{ns.rstrip('/')}/{n}" for n, ns in self.get_node_names_and_namespaces()}
        for n in names - self.known_nodes:
            self.client.emit("ROSNodeStarted", self.SOURCE, self.robot_id, {"node": n})
        for n in self.known_nodes - names:
            self.client.emit("ROSNodeStopped", self.SOURCE, self.robot_id, {"node": n})
        self.known_nodes = names
        elapsed = max(1e-3, now - self.rate_window_start)
        participants, seen = [], set()
        for t in self.watch:
            pubs = self.get_publishers_info_by_topic(t)
            subs = self.get_subscriptions_info_by_topic(t)
            ptype = pubs[0].topic_type if pubs else (subs[0].topic_type if subs else None)
            self.client.emit("ROSTopicObserved", self.SOURCE, self.robot_id, {
                "topic": t, "type": ptype,
                "publishers": [f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in pubs],
                "subscribers": [f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in subs],
                "rate_hz": self.msg_counts[t] / elapsed, "count": self.msg_counts[t]})
            for i in pubs + subs:
                if i.node_name not in seen:
                    seen.add(i.node_name)
                    participants.append({"name": i.node_name, "guid": bytes(i.endpoint_gid[:8]).hex(), "address": "dds"})
        self.client.emit("NetworkConnectionObserved", self.SOURCE, self.robot_id, {"participants": participants, "connections": []})
        self.msg_counts.clear()
        self.rate_window_start = now


def main(args=None):
    rclpy.init(args=args)
    node = Collector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
