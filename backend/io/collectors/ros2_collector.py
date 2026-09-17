"""Pointer module: the ROS 2 collector lives in ``backend/ros2/tron_ros2_collector`` because it must run
inside a ROS 2 environment (rclpy).  It speaks to the platform only through ``POST /api/ingest``
with the unified event schema.  See backend/ros2/tron_ros2_collector/tron_ros2_collector/collector.py
"""
