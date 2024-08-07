#!/bin/bash

# Author: Daniel Asadi
# Description: Record cameras + LIDAR + GNSS/INS data
# Usage: bash scripts/sensors_bag.sh <bag_name>

cleanup() {
    echo "Killing all ros2 processes"
    kill $PID_RECORD
    kill $PID_LAUNCH
    exit
}

trap cleanup SIGINT

sleep 30 # Wait for sensors to boot up

cd ~/dv24
source install/setup.bash
ros2 launch launcher sensors.launch.py &
PID_LAUNCH=$!

sleep 30 # Wait for nodes to boot up

cd "/media/utfr-dv/1tb ssd/rosbags"
ros2 bag record -s mcap -a &
PID_RECORD=$!

sleep 120 # Record for 2 minutes

cleanup # Stop recording after 2 minutes