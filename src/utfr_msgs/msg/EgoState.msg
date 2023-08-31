# Car Pose, Acceleration, Velocity

# Header for timestamp
std_msgs/Header header

geometry_msgs/PoseWithCovariance pose # World frame
geometry_msgs/AccelWithCovariance accel # Body frame
geometry_msgs/TwistWithCovariance vel # Body frame

# Frame:
# x axis: +ve north, +ve cw roll axis
# y axis: +ve east, +ve cw pitch axis
# z axis: +ve up, +ve cw yaw axis
# Units: m, °, m/s, m/s^2, °/s, 

float64 steering_angle # +ve cw about north

uint8 lap_count # lap count from planning

bool finished # finish signal from planning