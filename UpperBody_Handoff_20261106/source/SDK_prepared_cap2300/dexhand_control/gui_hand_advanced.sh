#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_python="$task_root/.venv/bin/python"
task_package="$task_root/../humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk"
if [[ ! -x "$task_python" ]]; then
  echo "First run: bash setup_hand_test.sh --gui --camera" >&2
  exit 2
fi
if ! "$task_python" -c 'import tkinter' >/dev/null 2>&1; then
  echo "Tk missing. On Ubuntu: sudo apt install python3-tk" >&2
  exit 2
fi
PYTHONPATH="$task_package${PYTHONPATH:+:$PYTHONPATH}" exec "$task_python" -m bhl_hand_sdk.gui \
  --project "$task_root/.." --config "$task_package/config/dexhand.yaml" \
  --model "$task_root/.cache/hand_landmarker.task" "$@"
