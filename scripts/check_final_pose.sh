#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
printf '%s\n' '=== Ground-truth odometry (ROS 2) ==='
ros2 topic echo /ground_truth/odom --once --field pose.pose.position
printf '%s\n' '=== Actual model pose (Gazebo) ==='
gz model -m ackerman --pose
