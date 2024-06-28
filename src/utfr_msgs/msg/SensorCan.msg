# Header for timestamp
std_msgs/Header header

# Steering sensor data 
std_msgs/Header steering_header
int16 steering_angle

# Motor speed data 
std_msgs/Header motor_header
float64 motor_speed

#brake pressure data
uint16 rear_pressure
uint16 front_pressure 

# Wheel speed sensor data 
std_msgs/Header sensor_header
float64 wheelspeed_fl
float64 wheelspeed_fr
float64 wheelspeed_rl
float64 wheelspeed_rr

# IMU data
std_msgs/Header imu_header
sensor_msgs/Imu imu_data

# GPS data 
std_msgs/Header gps_header 
geometry_msgs/Vector3Stamped gps_imu_accel 
geometry_msgs/Vector3Stamped gps_imu_ang_vel 
sensor_msgs/NavSatFix gps_gnss 
sensor_msgs/Imu gps_imu_data 

geometry_msgs/Accel gps_accel
geometry_msgs/Vector3Stamped gps_rpy
geometry_msgs/QuaternionStamped gps_orientation 
geometry_msgs/Pose gps_position
geometry_msgs/TwistStamped gps_twist 
geometry_msgs/TwistStamped gps_velocity 
std_msgs/NavSatFix latlong

std_msgs/Bool gps_error 

float64 commanded_torque