/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: compute_graph_node.hpp
* auth: Arthur Xu
* desc: compute graph node header
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
#include <utfr_msgs/msg/pose_graph.hpp>
#include <utfr_msgs/msg/system_status.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace compute_graph {

class ComputeGraphNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, initPublishers and initTimers.
   */
  ComputeGraphNode();

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

  /*! Pose Graph callback function
   */
  void poseGraphCB(const utfr_msgs::msg::PoseGraph msg);

  /*! Graph SLAM function
   *   @param[in] pose_graph utfr_msgs::msg::PoseGraph Pose graph message
   *   @param[out] cone_map utfr_msgs::msg::ConeMap Cone map message
   */
  void graphSLAM();

  // Publisher
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::ConeMap>::SharedPtr cone_map_publisher_;
  rclcpp::TimerBase::SharedPtr slam_timer_;

  // Subscribers
  rclcpp::Subscription<utfr_msgs::msg::PoseGraph>::SharedPtr
      pose_graph_subscriber_;

  utfr_msgs::msg::PoseGraph pose_graph_;
  double slam_rate_;
};
} // namespace compute_graph
} // namespace utfr_dv
