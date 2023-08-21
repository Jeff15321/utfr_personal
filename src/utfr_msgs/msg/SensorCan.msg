# Header for timestamp
std_msgs/Header header

# Steering sensor data 
std_msgs/Header steering_header
float64 steering_angle

# Motor speed data 
std_msgs/Header steering_header
float64 steering_angle

# Service brake data 
std_msgs/Header asb_header
float64 asb_pressure_1
float64 asb_pressure_2

# EBS pressure data 
std_msgs/Header ebs_header
float64 ebs_pressure_1
float64 ebs_pressure_2

# Wheelspeed sensor data 
std_msgs/Header sensor_header
float64 wheelspeed_fl
float64 wheelspeed_fr
float64 wheelspeed_rl
float64 wheelspeed_rr

# IMU data
sensor_msgs/Imu imu_data
