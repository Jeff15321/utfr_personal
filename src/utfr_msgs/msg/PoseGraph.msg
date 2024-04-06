# Pose graph for GraphSLAM

# Header for timestamp
std_msgs/Header header

PoseGraphData[] cones
PoseGraphData[] states
PoseGraphData[] measurement_edges
PoseGraphData[] motion_edge

float64[] color
float64[] cone_id
bool      run_slam