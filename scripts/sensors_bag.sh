#!/bin/bash

# Author: Daniel Asadi
# Description: Record cameras + LIDAR + GNSS/INS data
# Usage: bash scripts/sensors_bag.sh <bag_name>
# Service located in /etc/systemd/system/ros_sensors_launch.service

# How to create a service to launch this script on boot:

# Create a service file:
# sudo nano /etc/systemd/system/my_script.service

# Add the following lines:
# [Unit]
# Description=My Startup Script
# After=network.target

# [Service]
# ExecStart=/usr/local/bin/my_script.sh
# Type=simple
# Restart=on-failure

# [Install]
# WantedBy=multi-user.target

# Make the script executable:
# sudo chmod +x /usr/local/bin/my_script.sh

# Reload systemd to recognize the new service and enable it so it runs at boot:
# sudo systemctl daemon-reload
# sudo systemctl enable my_script.service

# Start the service immediately and check its status:
# sudo systemctl start my_script.service
# sudo systemctl status my_script.service

# Stop/disable the service:
# sudo systemctl stop my_script.service
# sudo systemctl disable my_script.service

# Remove service file:
# sudo rm /etc/systemd/system/my_script.service
# sudo systemctl daemon-reload

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
ros2 bag record -s mcap \
/car_interface/sensor_can \
/left_image/compressed \
/right_image/compressed \
/ouster/points \
/tf \
/tf_static &
PID_RECORD=$!

sleep 120 # Record for 2 minutes

cleanup # Stop recording after 2 minutes