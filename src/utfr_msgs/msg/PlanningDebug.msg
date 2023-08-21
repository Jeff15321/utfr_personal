#Planning debug

# Header for timestamp
std_msgs/Header header

# Crosstrack error as a double
float64 crosstrack_error

## Waypoint Path Specfic Navigation Messages:
TrajectoryPoint[] waypoints

## Trajectory Rollout Specific Debug Messages:
TrajectoryPath[] trajectories
uint8 sector

