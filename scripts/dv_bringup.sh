#!/bin/bash

# Author: Daniel Asadi
# Description: Run DV stack and record all topics
# Usage: sudo bash scripts/dv_bringup.sh <bag_name>

cleanup() {
    echo "Killing all ros2 processes"
    kill $PID_RECORD
    # kill $PID_LAUNCH
    exit
}

trap cleanup SIGINT

cd ~/dv24
source install/setup.bash
bash scripts/enable_can.sh
ros2 launch launcher interface.launch.py &
PID_LAUNCH=$!

# sleep 30

# cd "/media/utfr-dv/1tb ssd/rosbags"
# ros2 bag record -a -o "$1" &
# PID_RECORD=$!

wait