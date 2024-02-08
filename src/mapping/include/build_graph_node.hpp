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
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/types_slam2d.h>

//KD Tree Requirement
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

  /*! Initialize Heartbeat:
   */
  void initHeartbeat();
  
  /*! Publish Heartbeat:
   */
  void publishHeartbeat();

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

  /*! Add a pose to pose edge to G2O
    * @param[in] pose1 g2o::VertexSE2*, pointer to the first pose node
    * @param[in] pose2 g2o::VertexSE2*, pointer to the second pose node
    * @param[in] dx double, x distance between poses
    * @param[in] dy double, y distance between poses
    * @param[in] dtheta double, rotation between poses
    * @param[in] loop_closure bool, true if loop closure
    * @param[out] edge g2o::EdgeSE2PointXY*, pose to pose edge pointer
   */
  g2o::EdgeSE2* addPoseToPoseEdge(g2o::VertexSE2* pose1, g2o::VertexSE2* pose2,
                         double dx, double dy, double dtheta, bool loop_closure);

  /*! Add a pose to cone edge to G2O
    * @param[in] pose g2o::VertexSE2*, pointer to the pose node
    * @param[in] cone g2o::VertexPointXY*, pointer to the cone node
    * @param[in] dx double, x distance between pose and cone
    * @param[in] dy double, y distance between pose and cone
    * @param[out] edge g2o::EdgeSE2PointXY*, pose to cone edge pointer
   */
  g2o::EdgeSE2PointXY* addPoseToConeEdge(g2o::VertexSE2* pose, g2o::VertexPointXY* cone,
                               double dx, double dy);

  /*! Compose a graph for G2O to optimize.
   *  @param[in] states std::vector<utfr_msgs::msg::EgoState>&, past states
   *  @param[in] cones std::vector<std::pair<float, utfr_msgs::msg:Cone>>&, list
   * of cones, with their associated ID's
   *  @param[out] cone_map utfr_msgs::msg::ConeMap&, updated cone map
   */
  void buildGraph();

  /*! Graph SLAM function
   *   @param[in] pose_graph utfr_msgs::msg::PoseGraph Pose graph message
   *   @param[out] cone_map utfr_msgs::msg::ConeMap Cone map message
   */
  void graphSLAM();
  
  /*! States for hearbeat publisher */
  enum class HeartBeatState{ 
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
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Subscribers
  rclcpp::Subscription<utfr_msgs::msg::ConeDetections>::SharedPtr
      cone_detection_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      state_estimation_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      state_estimation_subscriber_2_;

  // Global variables
  std::vector<std::pair<float, utfr_msgs::msg::Cone>>
      past_detections_;                      // Previous cone detections
  std::map<int, g2o::VertexPointXY*> cone_id_to_vertex_map_;
  utfr_msgs::msg::ConeMap current_cone_map_; // Current cone map estimate
  utfr_msgs::msg::EgoState current_state_;   // Current state estimate
  std::map<int, utfr_msgs::msg::Cone> id_to_cone_map_; // Maps cone detection to id
  std::map<int, utfr_msgs::msg::EgoState> id_to_ego_map_; // Maps state estimate to id
  std::map<int, int> cone_id_to_color_map_; // Maps cone id to color
  std::map<int, g2o::VertexSE2*> id_to_pose_map_; // Maps state estimate to pose node
  std::map<int, std::tuple<double, double>> potential_cones_;
  int cones_potential_;
  int count_;
  bool loop_closed_;                         // True if loop is closed
  bool landmarked_;
  int landmarkedID_;
  int out_of_frame_;
  int cones_found_;
  int current_pose_id_;
  int first_detection_pose_id_;
  std::unique_ptr<kd_tree_knn::KDTree> globalKDTreePtr_;

  double heartbeat_rate_;

  // Lists for poses, cones, and edges
  std::vector<g2o::VertexSE2*> pose_nodes_;
  std::vector<g2o::VertexPointXY*> cone_nodes_;
  std::vector<g2o::EdgeSE2*> pose_to_pose_edges_;
  std::vector<g2o::EdgeSE2PointXY*> pose_to_cone_edges_;

  // G2O Information matricies
  Eigen::Matrix3d P2PInformationMatrix_;
  Eigen::Matrix2d P2CInformationMatrix_;
  Eigen::Matrix3d LoopClosureInformationMatrix_;

  g2o::SparseOptimizer optimizer_;
  utfr_msgs::msg::ConeMap cone_map_;
};
} // namespace build_graph
} // namespace utfr_dv