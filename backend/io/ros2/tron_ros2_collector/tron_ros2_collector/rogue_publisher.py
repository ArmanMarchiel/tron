"""Scenario B helper for a real ROS 2 graph: an *unexpected* node publishing a joint trajectory.

Benign, simulation/lab-only demo node.  It publishes the platform's 'rogue' waypoint as a
JointTrajectory only while scenario B is active (polled from the platform), so it never runs
unattended.

  ros2 run tron_ros2_collector rogue_publisher --ros-args -p platform_url:=http://<host>:8000
"""
from __future__ import annotations

import json
import time
import urllib.request

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


class RoguePublisher(Node):
    def __init__(self):
        super().__init__("teleop_override")
        self.declare_parameter("platform_url", "http://127.0.0.1:8000")
        self.declare_parameter("rate_hz", 10.0)
        self.url = self.get_parameter("platform_url").value.rstrip("/")
        self.pub = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.tick)
        self.active = None
        self.target = None
        self.last_poll = 0.0

    def tick(self):
        now = time.time()
        if now - self.last_poll > 0.5:
            self.last_poll = now
            try:
                with urllib.request.urlopen(self.url + "/api/faults/active", timeout=1.0) as r:
                    self.active = json.loads(r.read() or b"null")
                if self.target is None:
                    with urllib.request.urlopen(self.url + "/api/sim/waypoints", timeout=1.0) as r:
                        self.target = json.loads(r.read())["scenarios"]["rogue"]
            except Exception:
                self.active = None
        if not self.active or self.active.get("id") != "B" or not self.target:
            return
        m = JointTrajectory()
        m.header.frame_id = "rogue"
        m.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in self.target["q"]]
        pt.time_from_start = Duration(sec=2)
        m.points = [pt]
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    n = RoguePublisher()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
