#!/bin/bash
set -e
hand_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg="$hand_root/../humanoid_sdk-main/ros2_ws/src/bhl_hand_sdk"
export PYTHONPATH="$pkg:${PYTHONPATH:-}"
exec "$hand_root/.venv/bin/python" -m bhl_hand_sdk.simple_gui --project "$hand_root/.." --config "$pkg/config/dexhand_oneway.yaml" "$@"
