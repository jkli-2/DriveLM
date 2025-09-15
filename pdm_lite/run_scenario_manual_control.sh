#!/bin/bash

# carla
export CARLA_ROOT=/ext/carla
export WORK_DIR=/ext/DriveLM/pdm_lite
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export SCENARIO_RUNNER_ROOT=${WORK_DIR}/scenario_runner
export LEADERBOARD_ROOT=${WORK_DIR}/leaderboard
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}

export SCRIPT="${WORK_DIR}/scenario_runner/manual_control.py"
poetry run python ${SCRIPT}
