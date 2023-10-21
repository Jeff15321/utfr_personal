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
#include <utfr_msgs/msg/cone_detections.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/pose_graph.hpp>
#include <utfr_msgs/msg/system_status.hpp>

// Import G2O 2D Slam types
// #include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/types_slam2d.h>
// #include <g2o/types/slam2d/vertex_point_xy.h>

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
   *  @param[in] past_detections_ std::vector<std::pair<float,
   * utfr_msgs::msg::Cone>>&, current cone id mapping
   *  @param[out] past_detections_ std::vector<std::pair<float,
   * utfr_msgs::msg::Cone>>&, updated cone id mapping
   *  @param[out] detected_cone_ids std::vector<int>&, ids of cones detected
   */

  std::vector<int> KNN(const utfr_msgs::msg::ConeDetections &cones);

  /*! Implement functionalty to detect loop closures
   *  @param[in] cones std::vector<int>&, ids of detected cones
   *  @param[out] loop_closed boolean&, true if loop is closed
   */
  void loopClosure(const std::vector<int> &cones);

  /*! Create a pose node for G2O
    * @param[in] id int, id of the pose node
    * @param[in] x double, x position of the pose node
    * @param[in] y double, y position of the pose node
    * @param[in] theta double, rotation of the pose node (radians)
    * @param[out] vertex g2o::VertexSE2*, pose node pointer
   */
  g2o::VertexSE2* createPoseNode(int id, double x, double y, double theta);

  /*! Create a cone node for G2O
    * @param[in] id int, id of the cone node
    * @param[in] x double, x position of the cone node
    * @param[in] y double, y position of the cone node
    * @param[out] vertex g2o::VertexSE2, cone node pointer
   */
  g2o::VertexPointXY* createConeVertex(int id, double x, double y);

  /*! Compose a graph for G2O to optimize.
   *  @param[in] states std::vector<utfr_msgs::msg::EgoState>&, past states
   *  @param[in] cones std::vector<std::pair<float, utfr_msgs::msg:Cone>>&, list
   * of cones, with their associated ID's
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
  std::vector<utfr_msgs::msg::EgoState>
      past_states_; // Previous states of vehicle
  std::vector<std::pair<float, utfr_msgs::msg::Cone>>
      past_detections_;                      // Previous cone detections
  utfr_msgs::msg::ConeMap current_cone_map_; // Current cone map estimate
  utfr_msgs::msg::EgoState current_state_;   // Current state estimate
  bool loop_closed_;                         // True if loop is closed
  bool landmarked_;
  int landmarkedID_;
  bool out_of_frame_;
  int cones_found_;
};
} // namespace build_graph
} // namespace utfr_dv
