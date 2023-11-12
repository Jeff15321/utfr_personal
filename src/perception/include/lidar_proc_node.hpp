/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: lidar_proc_node.hpp
* auth: Kareem Elsawah
* desc: lidar processing node header
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

#include <clusterer.hpp>
#include <cone_filter.hpp>
#include <filter.hpp>

// Message Requirements
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace lidar_proc {

class LidarProcNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, initPublishers and initTimers.
   */
  LidarProcNode();

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

  /*! Initialize global variables:
   */
  double update_rate_;

  utfr_msgs::msg::EgoState::SharedPtr ego_state_{nullptr};
  utfr_msgs::msg::ConeMap::SharedPtr cone_map_{nullptr};
  utfr_msgs::msg::ConeDetections::SharedPtr cone_detections_{nullptr};

  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      ego_state_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ConeMap>::SharedPtr cone_map_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::ConeDetections>::SharedPtr
      cone_detection_subscriber_;

  rclcpp::Publisher<utfr_msgs::msg::ParametricSpline>::SharedPtr
      lidar_proc_publisher_;
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

  // todo cleanup
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_no_ground;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clustered;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_clustered_center;
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr pub_heartbeat;
  Filter filter;
  Clusterer clusterer;
  ConeFilter cone_filter;
  utfr_msgs::msg::Heartbeat heartbeat;
};
} // namespace lidar_proc
} // namespace utfr_dv
