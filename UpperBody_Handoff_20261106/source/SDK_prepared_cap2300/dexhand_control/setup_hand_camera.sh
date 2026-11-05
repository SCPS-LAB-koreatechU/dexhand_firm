#!/usr/bin/env bash
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
if [[ ! -x "$task_root/.venv/bin/python" ]]; then
  python3 -m venv --without-pip --system-site-packages "$task_root/.venv"
fi
if ! "$task_root/.venv/bin/python" -c 'import cv2,mediapipe,can,yaml,tkinter' >/dev/null 2>&1; then
  echo "카메라 의존성이 필요합니다. 기존 setup_hand_test.sh --camera --gui 설치 안내를 따라주세요." >&2
  exit 2
fi
"$task_root/.venv/bin/python" "$task_root/tools/fetch_hand_model.py" --output "$task_root/.cache/hand_landmarker.task"

# The X5 CPU does not expose AES; current MediaPipe 1.2.2 aborts at model creation.
if [[ "$(uname -m)" == "aarch64" ]]; then
  if ! "$task_root/.venv/bin/python" -c 'import importlib.metadata as m; assert m.version("mediapipe")=="0.10.18"' >/dev/null 2>&1; then
    "$task_root/.venv/bin/python" -m pip install --only-binary=:all: -r "$task_root/requirements_camera_rdk_x5.txt"
  fi
fi
"$task_root/.venv/bin/python" "$task_root/tools/check_hand_model.py" --model "$task_root/.cache/hand_landmarker.task"
