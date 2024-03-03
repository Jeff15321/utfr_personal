/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: controller_node.hpp
* auth: Justin Lim
* desc: controller node header
*/
#pragma once

// ROS2 Requirements
#include <ament_index_cpp/get_package_share_directory.hpp>
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
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/parametric_spline.hpp>
#include <utfr_msgs/msg/system_status.hpp>
#include <utfr_msgs/msg/target_state.hpp>
#include <utfr_msgs/msg/trajectory_point.hpp>
#include <utfr_msgs/msg/velocity_profile.hpp>
#include <utfr_msgs/msg/waypoint_path.hpp>
#include <visualization_msgs/msg/marker.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace controller {

class ControllerNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, initPublishers and initTimers.
   */
  ControllerNode();

  double k(std::vector<double> c, double s);

  std::vector<double>
  calculateSkidpadVelocities(utfr_msgs::msg::ParametricSpline &spline, double L,
                             int n, double a_lateral);

  std::vector<geometry_msgs::msg::Pose>
  discretizeCircle(const utfr_msgs::msg::ParametricSpline &spline_params,
                   double cur_s, double ds, int num_points);

  std::vector<double>
  calculateVelocities(utfr_msgs::msg::ParametricSpline &spline, double L, int n,
                      double a_lateral);

  std::vector<double> filterVelocities(std::vector<double> &max_velocities,
                                       double current_velocity, double distance,
                                       double max_velocity,
                                       double max_acceleration,
                                       double min_acceleration);

  double getMaxLongAccelGGV(double velocity, double a_lateral);

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

  void initEvent();

  void missionCB(const utfr_msgs::msg::SystemStatus &msg);

  /*! Initialize GGV data:
   * NOTE: assumes that the lateral acceleration data is in
   * decreasing order.
   */
  void initGGV(std::string filename);

  /*! Setup Heartbeat message with appropriate module name and update rate.
   */
  void initHeartbeat();

  /*! Send Heartbeat on every timer loop.
   *
   *  @param[in] status current module status, using Heartbeat status enum.
   */
  void publishHeartbeat(const int status);

  /*! EgoState Subscriber Callback:
   */
  void egoStateCB(const utfr_msgs::msg::EgoState &msg);

  /*! ConeMap Subscriber Callback:
   */
  void coneMapCB(const utfr_msgs::msg::ConeMap &msg);

  /*! Path Subscriber Callback:
   */
  void pathCB(const utfr_msgs::msg::ParametricSpline &msg);

  /*! VelocityProfile Subscriber Callback:
   */
  void velocityProfileCB(const utfr_msgs::msg::VelocityProfile &msg);

  /*! Lap counter callback:
   */
  void lapCounterCB(const utfr_msgs::msg::Heartbeat &msg);

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

  void timerCBEBS();

  void timerCBAS();

  /*! Discretize point on a path
   */
  geometry_msgs::msg::Pose
  discretizePoint(const utfr_msgs::msg::ParametricSpline &spline_params,
                  const double s, const double delta);

  /*! Discretize Path from Parametric
   */
  std::vector<geometry_msgs::msg::Pose>
  discretizeParametric(const utfr_msgs::msg::ParametricSpline &spline_params,
                       double cur_s, double ds, int num_points);

  /*! Pure Pursuit Controller
   */
  utfr_msgs::msg::TargetState purePursuitController(
      double max_steering_angle, utfr_msgs::msg::ParametricSpline spline_params,
      double cur_s, double ds, utfr_msgs::msg::VelocityProfile velocity_profile,
      double baselink_location, utfr_msgs::msg::EgoState ego_state,
      double base_lookahead_distance, double lookahead_distance_scaling_factor);

  /*! Initialize global variables:
   */
  double update_rate_;
  std::string event_;
  std::string controller_;
  int discretized_points_;
  double cte_error_;
  double cte_angle_error_;
  double ds_;
  double max_velocity_;
  double max_steering_angle_;
  double max_steering_rate_;
  double max_tire_;
  double baselink_location_;
  double wheel_base_;
  int num_points_;
  double base_lookahead_distance_;
  double lookahead_distance_scaling_factor_;
  int lap_count_;
  bool finished_event_ = false;
  bool finished_and_stopped_ = false;
  rclcpp::Time start_time_;
  bool start_finish_time = true;
  int last_lap_count_;

  bool skip_path_opt_;
  double lookahead_distance_;
  double a_lateral_max_;

  utfr_msgs::msg::EgoState::SharedPtr ego_state_{nullptr};
  utfr_msgs::msg::ConeMap::SharedPtr cone_map_{nullptr};
  utfr_msgs::msg::ParametricSpline::SharedPtr path_{nullptr};
  utfr_msgs::msg::VelocityProfile::SharedPtr velocity_profile_{nullptr};

  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      ego_state_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ConeMap>::SharedPtr cone_map_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ParametricSpline>::SharedPtr
      path_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::Heartbeat>::SharedPtr
      center_path_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::Heartbeat>::SharedPtr
      lap_counter_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::SystemStatus>::SharedPtr
      mission_subscriber_;

  rclcpp::Publisher<utfr_msgs::msg::TargetState>::SharedPtr
      target_state_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::EgoState>::SharedPtr ego_state_publisher__;
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      pure_pursuit_point_publisher_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Time ros_time_;

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      path_publisher_;

  utfr_msgs::msg::TargetState target_;
  utfr_msgs::msg::SystemStatus::SharedPtr status_{nullptr};
  utfr_msgs::msg::Heartbeat heartbeat_;

  // map of GGV data. keys are velocity, values are array of lat. accel
  std::unordered_map<double, std::vector<double>> GGV_vel_to_lat_accel_;
  // map of GGV data. keys are velocity, values are array of long. accel
  std::unordered_map<double, std::vector<double>> GGV_vel_to_long_accel_;
  std::set<double> GGV_velocities_;
};
} // namespace controller
} // namespace utfr_dv
