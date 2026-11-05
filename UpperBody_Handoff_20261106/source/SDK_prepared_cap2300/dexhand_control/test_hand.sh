#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_python="$task_root/.venv/bin/python"
task_package="$task_root/../humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk"
if [[ ! -x "$task_python" ]]; then
  echo "First run: bash setup_hand_test.sh   (camera: add --camera)" >&2
  exit 2
fi
if [[ ! -f "$task_package/bhl_hand_sdk/demo.py" ]]; then
  echo "Keep dexhand_control beside the current humanoid_sdk-main folder." >&2
  exit 2
fi
task_hand="${1:-right}"
task_mode="${2:-ping}"
if (( $# > 0 )); then shift; fi
if (( $# > 0 )); then shift; fi
PYTHONPATH="$task_package${PYTHONPATH:+:$PYTHONPATH}" exec "$task_python" -m bhl_hand_sdk.demo \
  "$task_hand" "$task_mode" \
  --config "$task_package/config/dexhand_oneway.yaml" \
  --model "$task_root/.cache/hand_landmarker.task" "$@"
