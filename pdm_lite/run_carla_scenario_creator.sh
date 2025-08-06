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
export REPETITIONS=1
export DEBUG_CHALLENGE=0

export PTH_ROUTE=${WORK_DIR}/leaderboard/data/routes_custom1

# Function to handle errors
handle_error() {
  pkill Carla
  exit 1
}

# Set up trap to call handle_error on ERR signal
trap 'handle_error' ERR

# Start the carla server
export PORT=$((RANDOM % (40000 - 2000 + 1) + 2000)) # use a random port
sh ${CARLA_SERVER} -carla-streaming-port=0 -carla-rpc-port=${PORT} &
sleep 20 # on a fast computer this can be reduced (e.g., to 6 seconds)

echo 'Port' $PORT

export ROUTES=${PTH_ROUTE}.xml
export SCRIPT="${WORK_DIR}/leaderboard/scripts/scenario_creator.py"
export ROUTE_ID=0
poetry run python ${SCRIPT} --host='localhost' --port=${PORT} --f ${ROUTES} ${ROUTE_ID}

# Kill the Carla server afterwards
pkill Carla
