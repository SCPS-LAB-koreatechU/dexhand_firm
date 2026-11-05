#!/bin/bash
set -e
source /home/scps-ubuntu1/teleop/env.sh
sdk_src="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/humanoid_sdk-main/ros2_ws/src"
export PYTHONPATH="$sdk_src/bhl_arm_sdk:$sdk_src/bhl_upper_body_sdk:${PYTHONPATH:-}"
exec /usr/bin/python3 -m bhl_arm_sdk.slider_gui
