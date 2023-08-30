# Header for timestamp
std_msgs/Header header

# Steering sensor data 
std_msgs/Header steering_header
int16 steering_angle

# Motor speed data 
std_msgs/Header motor_header
float64 motor_speed

# Service brake data (weighted average front and rear)
std_msgs/Header asb_header
uint16 asb_pressure

# Wheel speed sensor data 
std_msgs/Header sensor_header
float64 wheelspeed_fl
float64 wheelspeed_fr
float64 wheelspeed_rl
float64 wheelspeed_rr

# IMU data
std_msgs/Header imu_header
sensor_msgs/Imu imu_data
