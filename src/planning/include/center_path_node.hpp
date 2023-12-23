/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: center_path_node.hpp
* auth: Justin Lim
* desc: center path node header
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
#include <deque>

// Message Requirements
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/parametric_spline.hpp>
#include <utfr_msgs/msg/system_status.hpp>
#include <utfr_msgs/msg/target_state.hpp>
#include <utfr_msgs/msg/trajectory_point.hpp>
#include <utfr_msgs/msg/cone_detections.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Delaunay Requirements:
#ifdef EPS
#undef EPS
#endif
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::Point_2 Point;

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace center_path {

class CenterPathNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, initPublishers and initTimers.
   */
  CenterPathNode();

  void GlobalWaypoints();

  std::vector<std::pair<double,double>> getWaypoints();

  void nextWaypoint(std::deque<std::pair<double,double>> &waypoints);

  std::vector<std::pair<double,double>> transform(std::vector<std::pair<double,double>> &points);

  std::tuple<double,double,double,double> getCentres();

  std::tuple<double,double,double,double> skidpadCircleCentres();

  std::pair<double,double> circleCentre(std::vector<utfr_msgs::msg::Cone> &cones, double radius, int inlier_count);

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

  /*! ConeDetections Subscriber Callback:
   */
  void coneDetectionsCB(const utfr_msgs::msg::ConeDetections &msg);

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

  /*! Delaunay Triangulation:
   */
  void delaunayTriangulation();

  /*! Initialize global variables:
   */
  double update_rate_;
  std::string event_;
  const double smallRadius_ = 7.625; // radius of small circle for skidpad
  const double largeRadius_ = 10.625; // radius of large circle for skidpad
  const double centreDistance_ = 9.125; // skidpad centres to track centre dist
  const int smallCircleCones_ = 16; // number of cones in small circle
  const int largeCircleCones_ = 13; // number of cones in large circle

  std::deque<std::pair<double,double>> *path_{nullptr};

  utfr_msgs::msg::EgoState::SharedPtr ego_state_{nullptr};
  utfr_msgs::msg::ConeMap::SharedPtr cone_map_{nullptr};
  utfr_msgs::msg::ConeDetections::SharedPtr cone_detections_{nullptr};

  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      ego_state_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ConeMap>::SharedPtr cone_map_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ConeDetections>::SharedPtr
      cone_detection_subscriber_;

  rclcpp::Publisher<utfr_msgs::msg::ParametricSpline>::SharedPtr
      center_path_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Time ros_time_;

  utfr_msgs::msg::TargetState target_;
  utfr_msgs::msg::SystemStatus::SharedPtr status_{nullptr};
  utfr_msgs::msg::Heartbeat heartbeat_;
};
} // namespace center_path
} // namespace utfr_dv
