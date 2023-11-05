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
#include <visualization_msgs/msg/marker.hpp>
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

  /*! Publish Heartbeat:
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

  /*! Return Closest Point
   */
  geometry_msgs::msg::Pose
  closestPoint(utfr_msgs::msg::EgoState ego_state,
               std::vector<geometry_msgs::msg::Pose> waypoints);

  /*! Sign
   */
  double sign(double x);

  /*! Stanley Controller
   */
  utfr_msgs::msg::TargetState stanleyController(
      double k, double max_speed, double max_steering_angle,
      double max_steering_rate, utfr_msgs::msg::ParametricSpline spline_params,
      double cur_s, double ds, utfr_msgs::msg::VelocityProfile velocity_profile,
      double baselink_location, utfr_msgs::msg::EgoState ego_state);

  /*! Pure Pursuit Controller
   */
  utfr_msgs::msg::TargetState purePursuitController(
      double max_speed, double max_steering_angle,
      utfr_msgs::msg::ParametricSpline spline_params, double cur_s, double ds,
      utfr_msgs::msg::VelocityProfile velocity_profile,
      double baselink_location, utfr_msgs::msg::EgoState ego_state,
      double base_lookahead_distance, double lookahead_distance_scaling_factor);

  /*! Initialize global variables:
   */
  double update_rate_;
  std::string event_;
  std::string controller_;
  double stanley_gain_;
  double softening_constant_;
  double k_yaw_rate_;
  double k_damp_steer_;
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

  utfr_msgs::msg::EgoState::SharedPtr ego_state_{nullptr};
  utfr_msgs::msg::ConeMap::SharedPtr cone_map_{nullptr};
  utfr_msgs::msg::ParametricSpline::SharedPtr path_{nullptr};
  utfr_msgs::msg::VelocityProfile::SharedPtr velocity_profile_{nullptr};

  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      ego_state_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ConeMap>::SharedPtr cone_map_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ParametricSpline>::SharedPtr
      path_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::VelocityProfile>::SharedPtr
      velocity_profile_subscriber_;

  rclcpp::Publisher<utfr_msgs::msg::TargetState>::SharedPtr
      target_state_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::EgoState>::SharedPtr
      ego_state_publisher_;
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
};
} // namespace controller
} // namespace utfr_dv
