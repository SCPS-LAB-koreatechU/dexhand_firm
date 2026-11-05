#!/bin/bash
set -e
source /home/scps-ubuntu1/teleop/env.sh
exec /usr/bin/python3 /home/scps-ubuntu1/teleop/tools/motor_voltage_monitor.py
