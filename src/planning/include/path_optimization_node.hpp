/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: path_optimization_node.hpp
* auth: Justin
* desc: path optimization node header
*/
#pragma once

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// System Requirements
#include <chrono>
#include <fstream>
#include <functional>
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string>
#include <vector>

// Message Requirements
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/system_status.hpp>
#include <utfr_msgs/msg/target_state.hpp>
#include <utfr_msgs/msg/trajectory_point.hpp>
#include <utfr_msgs/msg/parametric_spline.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace path_optimization {

class PathOptimizationNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, initPublishers and initTimers.
   */
  PathOptimizationNode();

  /*! Calculate Velocities:
  * This function calculates the max velocity for a parametric spline at n 
    equidistant points for a lateral acceleration.
  * @param spline the spline
  * @param L the look ahead distance
  * @param n the number of points to calculate curvature for (n > 1)
  * @param a_lateral the lateral acceleration
  */
  std::vector<double> calculateVelocities(
                      utfr_msgs::msg::ParametricSpline &spline,
                      double L, int n, double a_lateral);

private:
  /*! Initialize and load params from config.yaml:
   */
  void initParams();

  /*! Initialize Subscribers:
   */
  void initSubscribers();

  /*! Initialize Publishers:
   */
  void initPublishers();

  /*! Initialize Timers:
   */
  void initTimers();

  /*! Initialize Heartbeat:
   */
  void initHeartbeat();
};
} // namespace path_optimization
} // namespace utfr_dv
