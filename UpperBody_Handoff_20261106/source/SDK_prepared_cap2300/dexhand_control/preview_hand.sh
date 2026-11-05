#!/usr/bin/env bash
set -eo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source /opt/ros/humble/setup.bash
source "$task_root/../humanoid_sdk-main/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${BHL_CAMERA_PREVIEW_DOMAIN:-93}"
exec ros2 launch bhl_hand_sdk estimated_hand.launch.py "$@"
