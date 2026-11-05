#!/bin/bash
set -e
hand_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$hand_root/simple_hand.sh" "$@"
