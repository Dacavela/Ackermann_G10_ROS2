#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
printf 'Build complete. Run in two terminals:\n'
printf '  bash scripts/run_gazebo.sh\n'
printf '  bash scripts/run_controller.sh\n'
