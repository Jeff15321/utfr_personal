# Car Pose, Acceleration, Velocity

# Header for timestamp
std_msgs/Header header

geometry_msgs/PoseWithCovariance pose # World frame
geometry_msgs/AccelWithCovariance accel # Body frame
geometry_msgs/TwistWithCovariance vel # Body frame

# rotation about z axis relative to north, clockwise in [rad]
float64 yaw
# rotational rate about z axis relative to north, clockwise to [rad]
float64 yaw_rate

float64 steering_angle