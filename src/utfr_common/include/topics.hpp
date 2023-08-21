/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: topics.hpp
* auth: Kelvin Cui
* desc: ros2 topics list
*/
#pragma once

// System Requirements
#include <string>

namespace utfr_dv {
namespace topics {

const std::string kExample{"example/topic"};

// Drivers
const std::string kGPS{"sgb/gps"};
const std::string kLidarRaw{"velodyne_points"};

// Perception
const std::string kConeDetections{"perception/cone_detections"};
const std::string kPerceptionHeartbeat{"perception/heartbeat"};

// ConeDetection
const std::string kLidarDebug{"cone_det/lidar_debug"};

// StateEstimation
const std::string kEgoState{"state_estimation/ego_state"};
const std::string kStateEstimationHeartbeat{"state_estimation/heartbeat"};
const std::string kDatumLLA{"state_estimation/datum_lla"};
const std::string kGPSData{"state_estimation/gps"};

// Mapping
const std::string kConeMap{"mapping/cone_map"};
const std::string kMapOccupancy{"mapping/occupancy_grid"};
const std::string kMappingHeartbeat{"mapping/heartbeat"};

// Navigation
const std::string kTargetState{"navigation/target_state"};
const std::string kWaypointPath{"navigation/waypoint_path"};
const std::string kNavDebug{"navigation/debug"};
const std::string kNavigationHeartbeat{"navigation/heartbeat"};

// Control Systems
const std::string kControlCmd{"control_systems/control_cmd"};
const std::string kThrottleCmd{"drive_by_wire/thr_cmd"};
const std::string kControlSystemsHeartbeat{"control_systems/heartbeat"};

// Sim
const std::string kEUFSControlCmd{"cmd"};
const std::string kEUFSMissionServer{"/ros_can/set_mission"};
const std::string kEUFSEgoState{"ground_truth/state"};
const std::string kEUFSConeDetection{"ground_truth/cones"};

// Misc
const std::string kSystemStatus("mission_manager/system_status");
const std::string kJetson{"interface/jetson"};

} // namespace topics
} // namespace utfr_dv