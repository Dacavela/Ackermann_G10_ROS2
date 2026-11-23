#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
ros2 run ackermann_trajectory_controller trajectory_node --ros-args \
  -p use_sim_time:=true \
  -p odom_topic:=/ground_truth/odom \
  -p radius:=2.6 \
  -p lookahead:=0.45 \
  -p speed:=0.30 \
  -p path_rotation_deg:=0.0 \
  -p goal_tolerance:=0.02 \
  -p slowdown_distance:=0.60 \
  -p csv_path:="$HOME/resultado_ackermann_final.csv"
