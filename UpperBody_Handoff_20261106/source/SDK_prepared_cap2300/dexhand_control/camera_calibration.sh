#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
task_package="$task_root/../humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk"
PYTHONPATH="$task_package${PYTHONPATH:+:$PYTHONPATH}" exec "$task_root/.venv/bin/python" -m bhl_hand_sdk.hand_tracking_gui --project "$task_root/.." --config "$task_package/config/dexhand_camera.yaml" "$@"
