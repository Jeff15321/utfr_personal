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
bash scripts/enable_can.sh
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
ros2 launch launcher interface.launch.py &
PID_LAUNCH=$!

sleep 30 # Wait for nodes to boot up

cd "/media/utfr-dv/1tb ssd/rosbags"
ros2 bag record -s mcap \
/car_interface/sensor_can \
/left_camera/images \
/right_camera/images \
/ouster/points \
/tf \
/tf_static \
/perception/cone_detections \
/lidar_pipeline/detected \
/perception/lidar_projection_left \
/perception/lidar_projection_right \
/perception/lidar_projection_matched_left \
/perception/lidar_projection_matched_right \
/perception/left_processed \
/perception/right_processed \
/perception/debug_left \
/perception/debug_right \
/mapping/cone_map \
/ekf/ego_state \
/planning/accel_path \
/planning/delaunay_midpoints \
/planning/delaunay_waypoint \
/planning/pure_pursuit_point \
/planning/target_state \
controls/control_cmd \
car_interface/system_status \
filter/positionlla \
filter/quaternion \
filter/euler \
filter/twist \
filter/velocity \
filter/free_acceleration \ 
-s mcap &

PID_RECORD=$!

sleep 120 # Record for 2 minutes

cleanup # Stop recording after 2 minutes