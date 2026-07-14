#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
cd "${REPO_ROOT}"
export PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
PYTHON="${PYTHON:-python}"

export DOBOT_ROBOT_HOST="${DOBOT_ROBOT_HOST:-192.168.5.2}"
export DOBOT_ROBOT_PORT="${DOBOT_ROBOT_PORT:-29999}"
export DOBOT_DRY_RUN="${DOBOT_DRY_RUN:-1}"
export DOBOT_SPEED_PERCENT="${DOBOT_SPEED_PERCENT:-10}"
export DOBOT_ACCEL_PERCENT="${DOBOT_ACCEL_PERCENT:-10}"
export DOBOT_MAX_STEP_MM="${DOBOT_MAX_STEP_MM:-5.0}"
export DOBOT_WORKSPACE_RADIUS_MM="${DOBOT_WORKSPACE_RADIUS_MM:-100.0}"
export DOBOT_DEADBAND_MM="${DOBOT_DEADBAND_MM:-0.5}"

DATASET_REPO_ID="${DATASET_REPO_ID:-vision-thinking/dobot_cr5_quest}"
DATASET_ROOT="${DATASET_ROOT:-data/dobot_cr5_quest}"
TASK_NAME="${TASK_NAME:-Teleoperate Dobot CR5 with Quest and collect training data}"
NUM_EPISODES="${NUM_EPISODES:-10}"
FPS="${FPS:-30}"
WARMUP_TIME_S="${WARMUP_TIME_S:-5}"
EPISODE_TIME_S="${EPISODE_TIME_S:-30}"
RESET_TIME_S="${RESET_TIME_S:-5}"
PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
VIDEO="${VIDEO:-false}"

"${PYTHON}" src/beavr/scripts/control_robot.py \
  --robot.type=dobot_cr5_only_adapter \
  --teleop.robot_name=dobot_cr5 \
  --teleop.operate=true \
  --control.type=record \
  --control.fps="${FPS}" \
  --control.num_episodes="${NUM_EPISODES}" \
  --control.warmup_time_s="${WARMUP_TIME_S}" \
  --control.episode_time_s="${EPISODE_TIME_S}" \
  --control.reset_time_s="${RESET_TIME_S}" \
  --control.repo_id="${DATASET_REPO_ID}" \
  --control.root="${DATASET_ROOT}" \
  --control.single_task="${TASK_NAME}" \
  --control.push_to_hub="${PUSH_TO_HUB}" \
  --control.video="${VIDEO}" \
  --control.resume=false
