# Header for timestamp
std_msgs/Header header

# Type of target state, based on planner
uint8 type

# For Global Waypoints
TrajectoryPoint trajectory
float64 radius_of_validity

# For Trajectory Rollout
float64 velocity
float64 steering_angle

# target state types:
uint8 GLOBAL=0
uint8 ROLLOUT=1