#!/bin/bash

# Start mapping.launch.py in a new terminal and get its PID
gnome-terminal -- bash -c "ros2 launch mapping mapping.launch.py; exec bash" &

# Start utfr_sim_bridge.launch.py in a new terminal and get its PID
gnome-terminal -- bash -c "ros2 bag play mess exec bash" &

# Start eval.py in a new terminal and get its PID
gnome-terminal -- bash -c "python3 /home/arthurxu/Documents/Coding/dv24/src/mapping/eval/test.py; exec bash" &
