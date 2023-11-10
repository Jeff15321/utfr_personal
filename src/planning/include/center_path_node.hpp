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

// Message Requirements
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/parametric_spline.hpp>
#include <utfr_msgs/msg/system_status.hpp>
#include <utfr_msgs/msg/target_state.hpp>
#include <utfr_msgs/msg/trajectory_point.hpp>
#include <utfr_msgs/msg/cone_detections.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

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
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
// #include <CGAL/draw_triangulation_2.h>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace center_path {

class CenterPathNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, initPublishers and initTimers.
   */
  CenterPathNode();
  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  // typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
  typedef K::Point_2 Point;
  

  typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, K>   Vb;
  typedef CGAL::Triangulation_data_structure_2<Vb>                       Tds;
  typedef CGAL::Delaunay_triangulation_2<K, Tds>                    Delaunay;
  typedef Delaunay::Vertex_handle Vertex_handle;

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

  /*! Function for sorting cones
   */
  bool coneDistComparitor(const utfr_msgs::msg::Cone& a, 
      const utfr_msgs::msg::Cone& b);
  
  /*! Accel line fitting
   * This function returns m and c for the line y=mx+c using cone detections 
     completely colorblind
   */
  std::vector<double> getAccelPath();

  /*! Midpoints using Delaunay Triangulation:
   */
  std::vector<CGAL::Point_2<CGAL::Epick> > Midpoints(utfr_msgs::msg::ConeDetections_<std::allocator<void> >::SharedPtr cone_detections_);

  /*! Bezier Points:
   */
  std::tuple<std::vector<CGAL::Point_2<CGAL::Epick> >, std::vector<double>, std::vector<double>> BezierPoints(std::vector<CGAL::Point_2<CGAL::Epick> > midpoints);


  /*! Initialize global variables:
   */
  double update_rate_;
  std::string event_;

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
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      accel_path_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      delaunay_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      first_midpoint_path_publisher_;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Time ros_time_;

  utfr_msgs::msg::TargetState target_;
  utfr_msgs::msg::SystemStatus::SharedPtr status_{nullptr};
  utfr_msgs::msg::Heartbeat heartbeat_;
};
} // namespace center_path
} // namespace utfr_dv
