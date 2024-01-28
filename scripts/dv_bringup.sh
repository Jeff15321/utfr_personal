#!/bin/bash

# Author: Daniel Asadi
# Description: Run DV stack and record all topics
# Usage: bash scripts/p_sensors_bag.sh <bag_name>

cleanup() {
    echo "Killing all ros2 processes"
    kill $PID_RECORD
    sleep 1
    kill $PID_LAUNCH
    exit
}

trap cleanup SIGINT

cd ~/dv24
source install/setup.bash
ros2 launch launcher interface.launch.py &
PID_LAUNCH=$!

sleep 30

cd # TODO: ssd location
ros2 bag record -a -o "$1" &
PID_RECORD=$!

wait