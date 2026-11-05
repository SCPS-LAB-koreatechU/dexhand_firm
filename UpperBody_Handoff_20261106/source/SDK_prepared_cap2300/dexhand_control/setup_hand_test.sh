#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_camera=false
task_gui=false
for task_option in "$@"; do
  case "$task_option" in
    --camera) task_camera=true ;;
    --gui) task_gui=true ;;
    *) echo "Usage: bash setup_hand_test.sh [--camera] [--gui]" >&2; exit 2 ;;
  esac
done
python3 -c 'import sys; assert sys.version_info >= (3,10), "Python 3.10+ required"'
if [[ "$task_gui" == true ]]; then
  if ! python3 -c 'import tkinter' >/dev/null 2>&1; then
    echo "Tk missing. On Ubuntu: sudo apt install python3-tk python3-venv" >&2
    exit 2
  fi
fi
if [[ ! -x "$task_root/.venv/bin/python" ]]; then
  python3 -m venv "$task_root/.venv"
fi
task_python="$task_root/.venv/bin/python"
if [[ "$task_camera" == true ]]; then
  "$task_python" -m pip install --only-binary=:all: -r "$task_root/hand-camera-requirements.txt"
  "$task_python" "$task_root/tools/fetch_hand_model.py" --output "$task_root/.cache/hand_landmarker.task"
  "$task_python" "$task_root/tools/check_camera_runtime.py" --model "$task_root/.cache/hand_landmarker.task"
else
  "$task_python" -m pip install --only-binary=:all: -r "$task_root/hand-test-requirements.txt"
fi
echo "Installed in $task_root/.venv; ROS environment was not changed."
echo "Next: bash test_hand.sh right ping"
if [[ "$task_gui" == true ]]; then echo "GUI: bash gui_hand.sh --dry-run"; fi
