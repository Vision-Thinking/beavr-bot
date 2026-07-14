#!/usr/bin/env bash
set -euo pipefail

# Defaults match the right-arm jump host from ~/.ssh/config:
# dobot-cr5a-right-computer -> CR5 controller at 192.168.5.2.
export DOBOT_ROBOT_HOST="${DOBOT_ROBOT_HOST:-192.168.5.2}"
export DOBOT_ROBOT_PORT="${DOBOT_ROBOT_PORT:-29999}"
export DOBOT_DRY_RUN="${DOBOT_DRY_RUN:-1}"
export DOBOT_SPEED_PERCENT="${DOBOT_SPEED_PERCENT:-10}"
export DOBOT_ACCEL_PERCENT="${DOBOT_ACCEL_PERCENT:-10}"
export DOBOT_MAX_STEP_MM="${DOBOT_MAX_STEP_MM:-5.0}"
export DOBOT_WORKSPACE_RADIUS_MM="${DOBOT_WORKSPACE_RADIUS_MM:-100.0}"
export DOBOT_DEADBAND_MM="${DOBOT_DEADBAND_MM:-0.5}"

python teleop.py \
  --robot_name=dobot_cr5 \
  --laterality=right
