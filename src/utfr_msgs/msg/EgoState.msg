# Car Pose, Velocities and Acceleration

# Header for timestamp
std_msgs/Header header

geometry_msgs/PoseWithCovarianceStamped pose # World frame
geometry_msgs/AccelWithCovarianceStamped accel # Body frame
geometry_msgs/TwistWithCovarianceStamped vel # Body frame