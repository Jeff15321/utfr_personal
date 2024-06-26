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
#include <utfr_msgs/msg/pose_graph_data.hpp>
#include <utfr_msgs/msg/system_status.hpp>

// Import G2O 2D Slam types
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/types/slam2d/vertex_se2.h>

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

  /*! Setup Heartbeat message with appropriate module name and update rate.
   */
  void initHeartbeat();

  /*! Send Heartbeat on every timer loop.
   *
   *  @param[in] status current module status, using Heartbeat status enum.
   */
  void publishHeartbeat(const int status);

  /*! Pose Graph callback function
   */
  void poseGraphCB(const utfr_msgs::msg::PoseGraph msg);

  /*! Graph SLAM function
   *   @param[in] pose_graph utfr_msgs::msg::PoseGraph Pose graph message
   *   @param[out] cone_map utfr_msgs::msg::ConeMap Cone map message
   */
  void graphSLAM();

  /*! Create a pose node for G2O
   * @param[in] id int, id of the pose node
   * @param[in] x double, x position of the pose node
   * @param[in] y double, y position of the pose node
   * @param[in] theta double, rotation of the pose node (radians)
   * @param[out] vertex g2o::VertexSE2*, pose node pointer
   */
  g2o::VertexSE2 *createPoseNode(int id, double x, double y, double theta);

  /*! Create a cone node for G2O
   * @param[in] id int, id of the cone node
   * @param[in] x double, x position of the cone node
   * @param[in] y double, y position of the cone node
   * @param[out] vertex g2o::VertexSE2, cone node pointer
   */
  g2o::VertexPointXY *createConeVertex(int id, double x, double y);

  /*! Add a pose to pose edge to G2O
   * @param[in] pose1 g2o::VertexSE2*, pointer to the first pose node
   * @param[in] pose2 g2o::VertexSE2*, pointer to the second pose node
   * @param[in] dx double, x distance between poses
   * @param[in] dy double, y distance between poses
   * @param[in] dtheta double, rotation between poses
   * @param[in] loop_closure bool, true if loop closure
   * @param[out] edge g2o::EdgeSE2PointXY*, pose to pose edge pointer
   */
  g2o::EdgeSE2 *addPoseToPoseEdge(g2o::VertexSE2 *pose1, g2o::VertexSE2 *pose2,
                                  double dx, double dy, double dtheta,
                                  bool loop_closure);

  /*! Add a pose to cone edge to G2O
   * @param[in] pose g2o::VertexSE2*, pointer to the pose node
   * @param[in] cone g2o::VertexPointXY*, pointer to the cone node
   * @param[in] dx double, x distance between pose and cone
   * @param[in] dy double, y distance between pose and cone
   * @param[out] edge g2o::EdgeSE2PointXY*, pose to cone edge pointer
   */
  g2o::EdgeSE2PointXY *addPoseToConeEdge(g2o::VertexSE2 *pose,
                                         g2o::VertexPointXY *cone, double dx,
                                         double dy);

  /*! Primary callback function
   */
  void timerCB();

  // Publisher
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::ConeMap>::SharedPtr cone_map_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::PoseGraphData>::SharedPtr
      slam_state_publisher_;

  // Subscribers
  rclcpp::Subscription<utfr_msgs::msg::PoseGraph>::SharedPtr
      pose_graph_subscriber_;

  // G2O Information matricies
  Eigen::Matrix3d P2PInformationMatrix_;
  Eigen::Matrix2d P2CInformationMatrix_;
  Eigen::Matrix3d LoopClosureInformationMatrix_;

  std::vector<g2o::VertexSE2 *> pose_nodes_;
  std::vector<g2o::VertexPointXY *> cone_nodes_;
  std::vector<g2o::EdgeSE2 *> pose_to_pose_edges_;
  std::vector<g2o::EdgeSE2PointXY *> pose_to_cone_edges_;

  // Create a data structure to store pose to pose edges between optimizations
  std::map<std::string, g2o::EdgeSE2 *> pose_to_pose_edge_map_;
  std::map<std::string, g2o::EdgeSE2PointXY *> pose_to_cone_edge_map_;

  std::map<int, g2o::VertexPointXY *> cone_id_to_vertex_map_;
  std::map<int, g2o::VertexSE2 *> pose_id_to_vertex_map_;

  std::map<int, utfr_msgs::msg::Cone> fixed_cone_map_;
  std::map<int, g2o::VertexSE2 *> fixed_pose_map_;
  std::unordered_set<int> fixed_pose_ids_;
  std::unordered_set<int> fixed_cone_ids_;

  int lastPose;
  int lastCone;
  int lastPoseToPoseEdge;
  int lastPoseToConeEdge;

  g2o::SparseOptimizer optimizer_;

  utfr_msgs::msg::PoseGraph data;
  bool do_graph_slam_;
  std::map<int, int> cone_id_to_color_map_; // Maps cone id to color
  utfr_msgs::msg::ConeMap cone_map_;

  utfr_msgs::msg::PoseGraph pose_graph_;
  double slam_rate_;
  utfr_msgs::msg::Heartbeat heartbeat_;
  double update_rate_;
  int mapping_mode_;
  rclcpp::TimerBase::SharedPtr main_timer_;
};
} // namespace compute_graph
} // namespace utfr_dv
