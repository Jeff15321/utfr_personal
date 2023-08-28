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

// Drivers
// TODO

// Perception
const std::string kConeDetections{"perception/cone_detections"};
const std::string kPerceptionHeartbeat{"perception/heartbeat"};

// EKF
const std::string kEgoState{"ekf/ego_state"};
const std::string kEKFHeartbeat{"ekf/heartbeat"};

// Mapping
const std::string kConeMap{"mapping/cone_map"};
const std::string kMappingBuildHeartbeat{"mapping/build/heartbeat"};
const std::string kPoseGraph{"mapping/pose_graph"};
const std::string kMappingComputeHeartbeat{"mapping/compute/heartbeat"};

// Planning
const std::string kTargetState{"planning/target_state"};
const std::string kWaypointPath{"planning/waypoint_path"};
const std::string kPlanningDebug{"planning/debug"};
const std::string kPlanningHeartbeat{"planning/heartbeat"};

// Controls
const std::string kControlCmd{"controls/control_cmd"};
const std::string kControlsHeartbeat{"controls/heartbeat"};

// Sim
const std::string kEUFSControlCmd{"cmd"};
const std::string kEUFSMissionServer{"/ros_can/set_mission"};
const std::string kEUFSEgoState{"ground_truth/state"};
const std::string kEUFSConeDetection{"ground_truth/cones"};

// Car interface
const std::string kSystemStatus("car_interface/system_status");
const std::string kSensorCan{"interface/sensor_can"};

} // namespace topics
} // namespace utfr_dv