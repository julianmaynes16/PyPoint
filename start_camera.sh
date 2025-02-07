#!/bin/bash
echo "Sourcing ROS..."
source /opt/ros/humble/setup.bash
echo "Starting camera..."
ros2 launch realsense2_camera rs_pointcloud_launch.py