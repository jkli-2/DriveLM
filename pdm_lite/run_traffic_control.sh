#!/bin/bash

# carla
export CARLA_ROOT=/home/ste/Documents/carla
export WORK_DIR=/home/ste/Documents/DriveLM/pdm_lite
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export SCENARIO_RUNNER_ROOT=${WORK_DIR}/scenario_runner
export LEADERBOARD_ROOT=${WORK_DIR}/leaderboard
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}

export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh
export REPETITIONS=1
export DEBUG_CHALLENGE=0

export PTH_ROUTE=${WORK_DIR}/leaderboard/data/routes_custom1
export SCENARIO_NAME="custom_1_1"

export SCRIPT="/home/ste/Documents/DriveLM/pdm_lite/traffic_control.py"
export ROUTE_ID=0
poetry run python ${SCRIPT}
