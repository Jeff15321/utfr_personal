#DV system status following FSG logging format

# Header for timestamp
std_msgs/Header header

# Current DV state
uint8 AS_state # 3 bits used for CANbus
uint8 EBS_state # 3 bits used for CANbus
uint8 AMI_state # 3 bits used for CANbus
uint8 Service_brake_state # 3 bits used for CANbus

# DV driving dynamics 1
uint8 Speed_actual # km/h
uint8 Speed_target # km/h

int8 Steering_angle_actual # deg, scale: 0.5
int8 Steering_angle_target # deg, scale: 0.5

uint8 Brake_hydr_actual # %
uint8 Brake_hydr_target # %

int8 Motor_moment_actual # %
int8 Motor_moment_target # %

# DV driving dynamics 2
int16 Acceleration_longitudinal # m/s^2, scale: 1/512
int16 Acceleration_lateral # m/s^2, scale: 1/512
int16 Yaw_rate # deg/s, scale: 1/128

# DV system status
uint8 AS_state_off=1
uint8 AS_state_ready=2
uint8 AS_state_driving=3
uint8 AS_state_emergency_brake=4
uint8 AS_state_finish=5

uint8 EBS_state_unavailable=1
uint8 EBS_state_armed=2
uint8 EBS_state_activated=3

uint8 AMI_state_acceleration=1
uint8 AMI_state_skidpad=2
uint8 AMI_state_trackdrive=3
uint8 AMI_state_braketest=4
uint8 AMI_state_inspection=5
uint8 AMI_state_autocross=6

uint8 Steering_state # bool, 1 bit used for CANbus

uint8 Service_brake_state_disengaged=1
uint8 Service_brake_state_engaged=2
uint8 Service_brake_state_available=3

uint8 Lap_counter # 3 bits used for CANbus
uint8 Cones_count_actual # 3 bits used for CANbus
uint8 Cones_count_all # 3 bits used for CANbus
