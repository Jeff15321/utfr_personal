#!/bin/bash

# Author: Daniel Asadi
# Description: Record cameras + LIDAR data
# Usage: bash scripts/p_sensors_bag.sh <bag_name>

cleanup() {
    echo "Killing all ros2 processes"
    kill $PID_RECORD
    kill $PID_LAUNCH
    exit
}

trap cleanup SIGINT

cd ~/dv24
source install/setup.bash
ros2 launch launcher p_sensors.launch.py &
PID_LAUNCH=$!

sleep 30

cd "/media/utfr-dv/1tb ssd/rosbags"
ros2 bag record -s mcap -a -o "$1" &
PID_RECORD=$!

wait