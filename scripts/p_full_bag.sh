#!/bin/bash

# Author: Daniel Asadi
# Description: Record full perception + sensors
# Usage: bash scripts/p_full_bag.sh <bag_name>

cleanup() {
    echo "Killing all ros2 processes"
    kill $PID_RECORD
    kill $PID_LAUNCH
    exit
}

trap cleanup SIGINT

cd ~/dv24
source install/setup.bash
ros2 launch launcher perception.launch.py &
PID_LAUNCH=$!

sleep 30

cd "/media/utfr-dv/1tb ssd/rosbags"
ros2 bag record -s mcap /ouster/points /right_image /right_camera_node/ready /left_image /left_camera_node/ready -o "$1" &
PID_RECORD=$!

wait