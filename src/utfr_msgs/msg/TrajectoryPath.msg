# Header for timestamp
std_msgs/Header header

TrajectoryPoint[] trajectories

# True if this is the optimal trajectory in a set of paths
std_msgs/Bool optimal

# True if this path collides with occupancy map at some trajectory point
std_msgs/Bool collision

float64 velocity

float64 steering_angle