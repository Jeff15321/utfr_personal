#dv system status following fsg logging format

# header for timestamp
std_msgs/Header header

# dv driving dynamics 1
uint8 speed_actual # km/h
uint8 speed_target # km/h

int8 steering_angle_actual # deg, scale: 0.5
int8 steering_angle_target # deg, scale: 0.5

uint8 brake_hydr_actual # %
uint8 brake_hydr_target # %

int8 motor_moment_actual # %
int8 motor_moment_target # %

# dv driving dynamics 2
int16 acceleration_longitudinal # m/s^2, scale: 1/512
int16 acceleration_lateral # m/s^2, scale: 1/512
int16 yaw_rate # deg/s, scale: 1/128

# dv system status
uint8 as_state # 3 bits used for canbus
uint8 MANUAL=0
uint8 AS_STATE_OFF=1
uint8 AS_STATE_READY=2
uint8 AS_STATE_DRIVING=3
uint8 AS_STATE_FINISH=4
uint8 AS_STATE_EMERGENCY=5

uint8 ebs_state # 2 bits used for canbus
uint8 EBS_STATE_UNAVAILABLE=1
uint8 EBS_STATE_ARMED=2
uint8 EBS_STATE_ACTIVATED=3

uint8 ami_state # 3 bits used for canbus
uint8 AMI_STATE_TESTING=0
uint8 AMI_STATE_ACCELERATION=1
uint8 AMI_STATE_SKIDPAD=2
uint8 AMI_STATE_AUTOCROSS=3
uint8 AMI_STATE_TRACKDRIVE=4
uint8 AMI_STATE_EBSTEST=5
uint8 AMI_STATE_INSPECTION=6
uint8 AMI_STATE_MANUAL=7

uint8 service_brake_state # 2 bits used for canbus
uint8 SERVICE_BRAKE_STATE_DISENGAGED=1
uint8 SERVICE_BRAKE_STATE_ENGAGED=2
uint8 SERVICE_BRAKE_STATE_AVAILABLE=3

uint8 steering_state # bool, 1 bit used for canbus

uint8 lap_counter # 4 bits used for canbus
uint8 cones_count_actual # 8 bits used for canbus
uint32 cones_count_all # 17 bits used for canbus
