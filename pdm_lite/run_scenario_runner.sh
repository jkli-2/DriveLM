#!/bin/bash

# This script starts PDM-Lite and the CARLA simulator on a local machine

# Make sure any previously started Carla simulator instance is stopped
# Sometimes calling pkill Carla only once is not enough.
pkill Carla
pkill Carla
pkill Carla

term() {
  echo "Terminated Carla"
  pkill Carla
  pkill Carla
  pkill Carla
  exit 1
}
trap term SIGINT

# carla
export CARLA_ROOT=/home/ste/Documents/carla
export WORK_DIR=/home/ste/Documents/DriveLM/pdm_lite
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI
export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla
export SCENARIO_RUNNER_ROOT=${WORK_DIR}/scenario_runner
export LEADERBOARD_ROOT=${WORK_DIR}/leaderboard
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":${PYTHONPATH}

export CARLA_SERVER=${CARLA_ROOT}/CarlaUE4.sh

export SCENARIO_NAME="custom_1_1"

# Function to handle errors
handle_error() {
  pkill Carla
  exit 1
}

# Set up trap to call handle_error on ERR signal
trap 'handle_error' ERR

# Start the carla server
export PORT=$((RANDOM % (40000 - 2000 + 1) + 2000)) # use a random port
# sh ${CARLA_SERVER} -carla-streaming-port=0 -carla-rpc-port=${PORT} &
sh ${CARLA_SERVER} &
sleep 10 # on a fast computer this can be reduced (e.g., to 6 seconds)

echo 'Port' $PORT

export SCRIPT="/home/ste/Documents/DriveLM/pdm_lite/scenario_runner/scenario_runner.py"
poetry run python ${SCRIPT} --scenario ${SCENARIO_NAME} --reloadWorld

# Kill the Carla server afterwards
pkill Carla
