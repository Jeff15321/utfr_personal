/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: build_graph_node.hpp
* auth: Arthur Xu
* desc: build graph node header
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
#include <utfr_msgs/msg/pose_graph.hpp>
#include <utfr_msgs/msg/cone_detections.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace build_graph {

class BuildGraphNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, initPublishers and initTimers.
   */
  BuildGraphNode();

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

  /*! Cone detection callback function
  */
  void coneDetectionCB(const utfr_msgs::msg::ConeDetections msg);

  /*! State Estimation callback function
  */
  void stateEstimationCB(const utfr_msgs::msg::EgoState msg);

  /*! Implement a KNN algorithm to match cones to previous detections
  *  @param[in] cones utfr_msgs::msg::ConeDetecions&, cone detections
  *  @param[in] cone_map utfr_msgs::msg::ConeMap&, current cone map estimate
  *  @param[out] cone_map utfr_msgs::msg::ConeMap&, updated cone map
  */
  void KNN(const utfr_msgs::msg::ConeDetections& cones);

  /*! Compose a graph for G2O to optimize.
  *  @param[in] states std::vector<utfr_msgs::msg::EgoState>&, past states
  *  @param[in] cones std::vector<std::pair<float, utfr_msgs::msg:Cone>>&, list of cones, with their associated ID's
  *  @param[out] cone_map utfr_msgs::msg::ConeMap&, updated cone map
  */
  void buildGraph();

  // Publisher
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::PoseGraph>::SharedPtr pose_graph_publisher_;

  // Subscribers
  rclcpp::Subscription<utfr_msgs::msg::ConeDetections>::SharedPtr 
      cone_detection_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr 
      state_estimation_subscriber_;

  // Global variables
  std::vector<utfr_msgs::msg::EgoState> past_states_; // Previous states of vehicle
  std::vector<std::pair<float, utfr_msgs::msg::Cone>> past_detections_; // Previous cone detections
  utfr_msgs::msg::ConeMap current_cone_map_; // Current cone map estimate
};
} // namespace build_graph
} // namespace utfr_dv
