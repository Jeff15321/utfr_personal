/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: pure_pursuit_controller.hpp
* auth: Youssef Elhadad
* desc: pure pursuit lateral controller header
*/
#pragma once

// System Requirements
#include <math.h>
// System Requirements
#include <chrono>
#include <functional>
#include <string>

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// UTFR Common Requirements
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Message Requirements
#include <std_msgs/msg/string.hpp>
#include <utfr_msgs/msg/control_cmd.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/jetson.hpp>
#include <utfr_msgs/msg/target_state.hpp>

#define PI 3.1415926535

namespace utfr_dv {
namespace controls {

class PurePursuitController {
public:
  /*! Initialize controller with car parameters.
   *
   *  @param[in] wheelbase double of vehicle wheelbase
   *  @param[in] ld_sf double of lookhead distance scaling factor
   */
  void initController(double wheelbase, double ld_sf);

  /*! Calculate Steering angle using pure pursuit algorithim.
   *
   *  @param[in] target TargetState of closest Trajectory Point
   *  @param[in] ego EgoState of latest ego state
   *  @returns double of steering angle in [deg].
   *
   */
  double getSteeringAngle(utfr_msgs::msg::TargetState::SharedPtr target,
                          utfr_msgs::msg::EgoState::SharedPtr ego);

private:
  double wheelbase_;
  double lookahead_distance_scaling_factor_;
};

using PurePursuitControllerUPtr = std::unique_ptr<PurePursuitController>;
using PurePursuitControllerSPtr = std::shared_ptr<PurePursuitController>;

} // namespace controls
} // namespace utfr_dv