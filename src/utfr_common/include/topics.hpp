/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: topics.hpp
* auth: Kelvin Cui, Daniel Asadi
* desc: ros2 topics list
*/
#pragma once

// System Requirements
#include <string>

namespace utfr_dv {
namespace topics {

// Example
const std::string kExample{"example/topic"};

// Perception
const std::string kConeDetections{"perception/cone_detections"};
const std::string kPerceptionHeartbeat{"perception/heartbeat"};
const std::string kLidarProcHeartbeat{"lidar_proc/heartbeat"};

const std::string kFiltered{"/lidar_pipeline/filtered"};
const std::string kNoGround{"/lidar_pipeline/no_ground"};
const std::string kClustered{"/lidar_pipeline/clustered"};
const std::string kDetected{"/lidar_pipeline/detected"};

// EKF
const std::string kEgoState{"ekf/ego_state"};
const std::string kPose{"ekf/pose"};
const std::string kEKFHeartbeat{"ekf/heartbeat"};

// Mapping
const std::string kLoopClosed{"mapping/loop_closed"};
const std::string kConeMap{"mapping/cone_map"};
const std::string kMappingBuildHeartbeat{"mapping/build/heartbeat"};
const std::string kPoseGraph{"mapping/pose_graph"};
const std::string kMappingComputeHeartbeat{"mapping/compute/heartbeat"};

// Planning
const std::string kTargetState{"planning/target_state"};
const std::string kWaypointPath{"planning/waypoint_path"};
const std::string kPlanningDebug{"planning/debug"};
const std::string kCenterPath{"planning/center_path"};
const std::string kOptimizedCenterPath{"planning/optimized_center_path"};
const std::string kVelocityProfile{"planning/velocity_profile"};
const std::string kCenterPathHeartbeat{"planning/center_path/heartbeat"};
const std::string kControllerHeartbeat{"planning/controller/heartbeat"};
const std::string kAccelPath{"planning/accel_path"};
const std::string kDelaunayWaypoints{"planning/delaunay_waypoints"};
const std::string kControllerPath{"planning/controller/path"};
const std::string kPurePursuitPoint{"planning/pure_pursuit_point"};
const std::string kSkidpadFitting{"planning/skidpad_fitting"};
const std::string kSkidpadFitting2{"planning/skidpad_fitting_2"};
const std::string kSkidpadFittingavg{"planning/skidpad_fitting_avg"};

// Controls
const std::string kControlCmd{"controls/control_cmd"};
const std::string kControlsHeartbeat{"controls/heartbeat"};

// Sim
const std::string kEUFSControlCmd{"cmd"};
const std::string kEUFSMissionServer{"/ros_can/set_mission"};
const std::string kEUFSEgoState{"ground_truth/state"};
const std::string kEUFSConeDetection{"ground_truth/cones"};
const std::string kEUFSConeMap{"ground_truth/cones_map"};

// Car interface
const std::string kSystemStatus("car_interface/system_status");
const std::string kSensorCan{"car_interface/sensor_can"};

} // namespace topics
} // namespace utfr_dv