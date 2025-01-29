#!/bin/bash

# Author: Daniel Asadi
# Description: Run DV stack and record all topics
# Usage: sudo bash scripts/dv_bringup.sh <bag_name>

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
# Restart=no

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
    sudo ip link set can0 down
    pkill -f "ros2"
    exit
}

trap cleanup SIGINT

cd /home/utfr-dv/utfr_dv/
source install/setup.bash
sudo ip link set can0 down
bash scripts/enable_can.sh
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
ros2 launch car_interface car_interface.launch.py &
PID_LAUNCH=$!

# sleep 30 # Wait for nodes to boot up
cd /home/utfr-dv/utfr_dv/rosbags
# ros2 bag record -s mcap \
# /tf \
# /tf_static \
# /car_interface/system_status \
# /car_interface/sensor_can \
# /controls/control_cmd \
# /left_camera/images \
# /right_camera/images \
# /ouster/points \
# /perception/cone_detections \
# /lidar_pipeline/detected \
# /lidar_pipeline/clustered \
# /lidar_pipeline/filtered \
# /lidar_pipeline/no_ground \
# /perception_ego_state \
# /perception/cone_detections \
# /perception/debug \
# /perception/debug_undistorted \
# /perception/lidar_projection \
# /perception/lidar_projection_matched \
# /synced_ego_state \
# /mapping/cone_map \
# /IM_GONNA_KMS \
# /GPS_STATE \
# /ekf/ego_state \
# /planning/accel_path \
# /planning/delaunay_midpoints \
# /planning/delaunay_waypoint \
# /planning/center_path \
# /planning/pure_pursuit_point \
# /planning/target_state \
# /planning/controller/path \
# filter/positionlla \
# filter/quaternion \
# filter/euler \
# filter/twist \
# filter/velocity \
# filter/free_acceleration \
# -s mcap &

PID_RECORD=$!

sleep 500 # Record for x seconds

cleanup # Stop recording 