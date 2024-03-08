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
#include <std_msgs/msg/bool.hpp>

// Message Requirements
#include "tf2_ros/transform_broadcaster.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <utfr_msgs/msg/cone_detections.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/pose_graph_data.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/pose_graph.hpp>
#include <utfr_msgs/msg/system_status.hpp>

// Import G2O 2D Slam types
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/types/slam2d/vertex_se2.h>

// KD Tree Requirement
#include <kd_tree_knn.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

#include <g2o/core/sparse_optimizer.h>

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

  /*! Setup Heartbeat message with appropriate module name and update rate.
   */
  void initHeartbeat();

  /*! Send Heartbeat on every timer loop.
   *
   *  @param[in] status current module status, using Heartbeat status enum.
   */
  void publishHeartbeat(const int status);

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
   *  @param[out] cones_id_list std::vector<int> of cones found
   */
  void loopClosure(const std::vector<int> &cones);

  /*! Primary callback function
   */
  void timerCB();

  /*! States for hearbeat publisher */
  enum class HeartBeatState {
    NOT_READY = 1,
    READY = 2,
    ACTIVE = 3,
    ERROR = 4,
    FINISH = 5
  };

  HeartBeatState heartbeat_state_;

  // Publisher
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::PoseGraph>::SharedPtr pose_graph_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::ConeMap>::SharedPtr cone_map_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr loop_closure_publisher_;

  // Subscribers
  rclcpp::Subscription<utfr_msgs::msg::ConeDetections>::SharedPtr
      cone_detection_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      state_estimation_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      state_estimation_subscriber_2_;

  // Global variables
  std::vector<std::pair<float, utfr_msgs::msg::Cone>>
      past_detections_; // Previous cone detections
  std::map<int, utfr_msgs::msg::PoseGraphData> cone_id_to_vertex_map_;
  utfr_msgs::msg::ConeMap current_cone_map_; // Current cone map estimate
  utfr_msgs::msg::EgoState current_state_;   // Current state estimate
  std::map<int, utfr_msgs::msg::Cone>
      id_to_cone_map_; // Maps cone detection to id
  std::map<int, utfr_msgs::msg::EgoState>
      id_to_ego_map_;                       // Maps state estimate to id
  std::map<int, int> cone_id_to_color_map_; // Maps cone id to color
  std::map<int, utfr_msgs::msg::PoseGraphData>
      id_to_pose_map_; // Maps state estimate to pose node
  std::map<int, std::tuple<double, double>> potential_cones_;
  std::map<int, std::vector<double>> average_position_;
  int cones_potential_;
  int count_;
  bool loop_closed_; // True if loop is closed
  bool landmarked_;
  int landmarkedID_;
  int out_of_frame_;
  int cones_found_;
  int current_pose_id_;
  int first_detection_pose_id_;
  std::unique_ptr<kd_tree_knn::KDTree> globalKDTreePtr_;
  double heartbeat_rate_;
  bool do_graph_slam_;
  std_msgs::msg::Bool closed_loop_once;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  utfr_msgs::msg::ConeDetections current_cone_detections_;
  utfr_msgs::msg::EgoState current_ego_state_;

  // Lists for poses, cones, and edges
  std::vector<utfr_msgs::msg::PoseGraphData> pose_nodes_;
  std::vector<utfr_msgs::msg::PoseGraphData> cone_nodes_;
  std::vector<utfr_msgs::msg::PoseGraphData> pose_to_pose_edges_;
  std::vector<utfr_msgs::msg::PoseGraphData> pose_to_cone_edges_;

  utfr_msgs::msg::ConeMap cone_map_;
  utfr_msgs::msg::Heartbeat heartbeat_;
  double update_rate_;
  rclcpp::TimerBase::SharedPtr main_timer_;
};
} // namespace build_graph
} // namespace utfr_dv