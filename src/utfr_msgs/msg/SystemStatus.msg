# Header for timestamp
std_msgs/Header header

# Autonomous System status based on enum below
uint8 status

# Selected Autonomous Mission
uint8 mission

# Autonomous System Status:
uint8 OFF=0
uint8 READY=1
uint8 DRIVING=2
uint8 EMERGENCY=3
uint8 FINISHED=4

uint8 MANUAL=0
uint8 INSPECTION=1
uint8 EBS=2
uint8 ACCEL=3
uint8 SKIDPAD=4