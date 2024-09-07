# Header for timestamp
std_msgs/Header header

# Steering sensor data 
int16 steering_angle

# Motor speed data 
float64 motor_speed

#brake pressure data
uint16 rear_pressure
uint16 front_pressure 

# Wheel speed sensor data 
float64 wheelspeed_fl
float64 wheelspeed_fr
float64 wheelspeed_rl
float64 wheelspeed_rr

# MTI680G GNSS/INS data 
sensor_msgs/NavSatFix position
sensor_msgs/Imu imu
geometry_msgs/Vector3 rpy
geometry_msgs/Twist velocity
std_msgs/Bool error

float64 commanded_torque