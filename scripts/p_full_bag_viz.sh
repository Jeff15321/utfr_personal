#!/bin/bash

# Author: Daniel Asadi
# Description: Record full perception + sensors with viz
# Usage: bash scripts/p_full_bag_viz.sh <bag_name>

cleanup() {
    echo "Killing all ros2 processes"
    kill $PID_RECORD
    kill $PID_BRIDGE
    kill $PID_VISUALIZATION
    kill $PID_LAUNCH
    exit
}

trap cleanup SIGINT

cd ~/dv24
source install/setup.bash
ros2 launch launcher perception.launch.py &
PID_LAUNCH=$!
ros2 launch visualization visualization.launch.py &
PID_VISUALIZATION=$!
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
PID_BRIDGE=$!

sleep 30

cd "/media/utfr-dv/1tb ssd/rosbags"
ros2 bag record -s mcap /ouster/points /right_camera_node/images /right_camera_node/ready /left_camera_node/images /left_camera_node/ready -o "$1" &
PID_RECORD=$!

wait