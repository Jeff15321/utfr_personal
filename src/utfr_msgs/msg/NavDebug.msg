#Navigational Debug 
std_msgs/Header header

# Crosstrack error as a double
float64 crosstrack_error

# Navigation Planner Type
uint8 type # planner type based on enum below

## Waypoint Path Specfic Navigation Messages:
TrajectoryPoint[] waypoints

## Trajectory Rollout Specific Debug Messages:
TrajectoryPath[] trajectories
uint8 sector

# cone types:
uint8 WAYPOINT=0
uint8 ROLLOUT=1
