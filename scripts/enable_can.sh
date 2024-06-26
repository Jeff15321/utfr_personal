#!/bin/bash

# Define the CAN interface and bitrate
CAN_INTERFACE="can0"
BITRATE="1000000"

# Load the necessary CAN modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Check if the interface exists
if ! ip link show $CAN_INTERFACE > /dev/null 2>&1; then
    echo "Error: CAN interface $CAN_INTERFACE does not exist."
    exit 1
fi

# Bring down the CAN interface if it is up
sudo ip link set $CAN_INTERFACE down

# Set up the CAN interface with the specified bitrate
sudo ip link set $CAN_INTERFACE type can bitrate $BITRATE

# Bring up the CAN interface
sudo ip link set $CAN_INTERFACE up

# Verify the CAN interface status
ip link show $CAN_INTERFACE

# Print success message
echo "CAN interface $CAN_INTERFACE is up with bitrate $BITRATE."

# Optionally, start candump to see incoming CAN messages (uncomment to use)
# candump $CAN_INTERFACE

exit 0

#if there's issues, the pins might be not configured. then try: `sudo /opt/nvidia/jetson-io/jetson-io.py`