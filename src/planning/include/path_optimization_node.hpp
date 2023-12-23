/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: path_optimization_node.hpp
* auth: Justin, Richard
* desc: path optimization node header
*/
#pragma once

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// System Requirements
#include <chrono>
#include <fstream>
#include <functional>
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string>
#include <vector>
#include <unordered_map>
#include <set>

// Message Requirements
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/parametric_spline.hpp>
#include <utfr_msgs/msg/system_status.hpp>
#include <utfr_msgs/msg/target_state.hpp>
#include <utfr_msgs/msg/trajectory_point.hpp>
#include <utfr_msgs/msg/velocity_profile.hpp>

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
  std::vector<double>
  calculateVelocities(utfr_msgs::msg::ParametricSpline &spline, double L, int n,
                      double a_lateral);
  /*! Filter Velocities:
  * This function passes through all the velocities and makes sure that all
    accelerations are possible, and that the velocities are under the max
    allowed velocity. If possible, all velocities will be under the maxes,
    otherwise they will be minimized until they fall below the maxes.
  * @param max_velocities the max velocities on the current path (size>1)
  * @param distance the distance the velocities span
  * @param max_velocity the max velocity that the car can travel
  * @param max_acceleration the max acceleration (positive)
  * @param min_acceleration the max deceleration (negative)
  */
  std::vector<double> filterVelocities(std::vector<double> &max_velocities,
                                       double current_velocity, double distance,
                                       double max_velocity,
                                       double max_acceleration,
                                       double min_acceleration);
  
  /*! Calculate Longitudinal Acceleration:
   * This function calculates the maximum (positive) longitudinal acceleration possible
   * for a given velocity and lateral acceleration.
   * 
   * @param 
   */
  double getMaxA_longit(double velocity, double a_lateral);

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

  /*! Initialize GGV data:
  */
  void initGGV(std::string filename);

  /*! Initialize Heartbeat:
   */
  void initHeartbeat();

  /*! Publish Heartbeat:
   */
  void publishHeartbeat(const int status);

  /*! EgoState Subscriber Callback:
   */
  void egoStateCB(const utfr_msgs::msg::EgoState &msg);

  /*! ConeMap Subscriber Callback:
   */
  void coneMapCB(const utfr_msgs::msg::ConeMap &msg);

  /*! CenterPath Subscriber Callback:
   */
  void centerPathCB(const utfr_msgs::msg::ParametricSpline &msg);

  /*! Accel Timer Callback:
   */
  void timerCBAccel();

  /*! Skidpad Timer Callback:
   */
  void timerCBSkidpad();

  /*! Autocross Timer Callback:
   */
  void timerCBAutocross();

  /*! Trackdrive Timer Callback:
   */
  void timerCBTrackdrive();

  /*! 
   */

  /*! Initialize global variables:
   */
  double update_rate_;
  std::string event_;
  bool skip_path_opt_;
  double lookahead_distance_;
  double num_points_;
  double a_lateral_max_;
  double max_velocity_;

  utfr_msgs::msg::EgoState::SharedPtr ego_state_{nullptr};
  utfr_msgs::msg::ConeMap::SharedPtr cone_map_{nullptr};
  utfr_msgs::msg::ParametricSpline::SharedPtr center_path_{nullptr};

  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      ego_state_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ConeMap>::SharedPtr cone_map_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ParametricSpline>::SharedPtr
      center_path_subscriber_;

  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::ParametricSpline>::SharedPtr
      center_path_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::VelocityProfile>::SharedPtr
      velocity_profile_publisher_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Time ros_time_;

  utfr_msgs::msg::TargetState target_;
  utfr_msgs::msg::SystemStatus::SharedPtr status_{nullptr};
  utfr_msgs::msg::Heartbeat heartbeat_;

  
  // map of GGV data. keys are velocity, values are array of lat. accel
  std::unordered_map<double, std::vector<double>> GGV_vel_to_lat_accel;
  // map of GGV data. keys are velocity, values are array of long. accel
  std::unordered_map<double, std::vector<double>> GGV_vel_to_long_accel;
  std::set<double> GGV_velocities;
};
} // namespace path_optimization
} // namespace utfr_dv
