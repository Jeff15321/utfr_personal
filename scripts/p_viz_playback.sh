#!/bin/bash

# Author: Daniel Asadi
# Description: Playback sensor data rosbag against perception node and viz
# Usage: bash scripts/p_viz_playback.sh <bag_name>

cleanup() {
    echo "Killing all ros2 processes"
    kill $PID_PLAYBACK
    kill $PID_BRIDGE
    kill $PID_VISUALIZATION
    kill $PID_LAUNCH
    exit
}

trap cleanup SIGINT

cd ~/dv24
source install/setup.bash
ros2 launch launcher p_stack.launch.py &
PID_LAUNCH=$!
ros2 launch visualization visualization.launch.py &
PID_VISUALIZATION=$!
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
PID_BRIDGE=$!

sleep 30

cd "/media/utfr-dv/1tb ssd/rosbags"
ros2 bag play -l "$1" &
PID_PLAYBACK=$!

wait