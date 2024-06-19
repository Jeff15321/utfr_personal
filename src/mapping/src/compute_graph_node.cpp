/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: compute_graph_node.cpp
* auth: Arthur Xu
* desc: compute graph node class
*/

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include <Eigen/Core>
#include <build_graph_node.hpp>
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <kd_tree_knn.hpp>

G2O_USE_TYPE_GROUP(slam2d);
#include <compute_graph_node.hpp>

namespace utfr_dv {
namespace compute_graph {

ComputeGraphNode::ComputeGraphNode() : Node("compute_graph_node") {
  this->initParams();
  this->initHeartbeat();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::NOT_READY);
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
}

using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
using SlamLinearSolver =
    g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;

void ComputeGraphNode::initParams() {
  this->declare_parameter("update_rate", 33.33);
  this->declare_parameter("slam_timer_", 33.33);

  this->declare_parameter("mapping_mode", 0);
  mapping_mode_ = this->get_parameter("mapping_mode").as_int();

  // Will have to tune these later depending on the accuracy of our sensors
  Eigen::DiagonalMatrix<double, 3> P2P;
  Eigen::DiagonalMatrix<double, 2> P2C;
  Eigen::DiagonalMatrix<double, 3> LoopClosure;
  P2P.diagonal() << 300, 300, 3000;
  P2C.diagonal() << 100, 1000;
  LoopClosure.diagonal() << 500, 500, 5000;

  P2PInformationMatrix_ = P2P;
  P2CInformationMatrix_ = P2C;
  LoopClosureInformationMatrix_ = LoopClosure;

  auto linearSolverLM = std::make_unique<SlamLinearSolver>();
  linearSolverLM->setBlockOrdering(false);
  g2o::OptimizationAlgorithm *optimizer =
      new g2o::OptimizationAlgorithmLevenberg(
          std::make_unique<SlamBlockSolver>(std::move(linearSolverLM)));
  optimizer_.setAlgorithm(optimizer);

  // int lastPose = 0;
  // int lastCone = 0;
  // int lastPosetoPoseEdge = 0;
  // int lastPosetoConeEdge = 0;

  pose_id_to_vertex_map_.clear();
  cone_id_to_vertex_map_.clear();

  slam_rate_ =
      this->get_parameter("slam_timer_").get_parameter_value().get<double>();
  update_rate_ = this->get_parameter("update_rate").as_double();
}

void ComputeGraphNode::initSubscribers() {
  if (mapping_mode_ == 0) {
    pose_graph_subscriber_ = this->create_subscription<utfr_msgs::msg::PoseGraph>(
      topics::kPoseGraph, 1,
      std::bind(&ComputeGraphNode::poseGraphCB, this, std::placeholders::_1));
  }
}

void ComputeGraphNode::initPublishers() {
  if (mapping_mode_ == 0) {
    cone_map_publisher_ =
      this->create_publisher<utfr_msgs::msg::ConeMap>(topics::kConeMap, 10);

    slam_state_publisher_ = 
      this->create_publisher<utfr_msgs::msg::PoseGraphData>(topics::kSlamPose, 10);
  }
}

void ComputeGraphNode::initTimers() {
  if (mapping_mode_ == 0) {
    main_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(this->slam_rate_),
      std::bind(&ComputeGraphNode::timerCB, this));
  }
}

void ComputeGraphNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kMappingComputeHeartbeat, 10);
  heartbeat_.module.data = "mapping_compute";
  heartbeat_.update_rate = update_rate_;
}

void ComputeGraphNode::publishHeartbeat(const int status) {
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_publisher_->publish(heartbeat_);
}

g2o::VertexSE2 *ComputeGraphNode::createPoseNode(int id, double x, double y,
                                                 double theta) {
  g2o::VertexSE2 *poseVertex = new g2o::VertexSE2();
  poseVertex->setId(id);

  g2o::SE2 poseEstimate;
  poseEstimate.fromVector(Eigen::Vector3d(x, y, theta));
  poseVertex->setEstimate(poseEstimate);

  return poseVertex;
}

g2o::VertexPointXY *ComputeGraphNode::createConeVertex(int id, double x,
                                                       double y) {

  g2o::VertexPointXY *coneVertex = new g2o::VertexPointXY();
  coneVertex->setId(id);

  Eigen::Vector2d position(x, y);
  coneVertex->setEstimate(position);
  return coneVertex;
}

g2o::EdgeSE2 *ComputeGraphNode::addPoseToPoseEdge(g2o::VertexSE2 *pose1,
                                                  g2o::VertexSE2 *pose2,
                                                  double dx, double dy,
                                                  double dtheta,
                                                  bool loop_closure) {
  g2o::EdgeSE2 *edge = new g2o::EdgeSE2();

  g2o::SE2 measurement(dx, dy, dtheta);
  edge->setMeasurement(measurement);
  if (loop_closure) {
    edge->setInformation(LoopClosureInformationMatrix_);
  } else {
    edge->setInformation(P2PInformationMatrix_);
  }

  edge->setVertex(0, pose1);
  edge->setVertex(1, pose2);

  return edge;
}

g2o::EdgeSE2PointXY *ComputeGraphNode::addPoseToConeEdge(
    g2o::VertexSE2 *pose, g2o::VertexPointXY *cone, double dx, double dy) {
  g2o::EdgeSE2PointXY *edge = new g2o::EdgeSE2PointXY();

  Eigen::Vector2d measurement(dx, dy);
  edge->setMeasurement(measurement);
  edge->setInformation(P2CInformationMatrix_);

  edge->setVertex(0, pose);
  edge->setVertex(1, cone);

  return edge;
}

void ComputeGraphNode::poseGraphCB(const utfr_msgs::msg::PoseGraph msg) {
  data = msg;
}

void ComputeGraphNode::graphSLAM() {
  // Set the fixed node as 1001

  int POSE_WINDOW = do_graph_slam_ ? 1500 : 500;
  int CONE_WINDOW = do_graph_slam_ ? 4500 : 1500;

  for (size_t i = 0; i < pose_nodes_.size(); i++) {
    if (fixed_pose_ids_.find(pose_nodes_[i]->id()) == fixed_pose_ids_.end()) {
      optimizer_.addVertex(pose_nodes_[i]);
    } else {
      optimizer_.addVertex(fixed_pose_map_[pose_nodes_[i]->id()]);
    }
  }

  for (g2o::VertexPointXY *cone : cone_nodes_) {
    // int id = cone->id();
    // if (fixed_cone_ids_.find(id) == fixed_cone_ids_.end()) {
    optimizer_.addVertex(cone);
    // } else {
    //   utfr_msgs::msg::Cone coneData = fixed_cone_map_[id];
    //   optimizer_.addVertex(createConeVertex(id, coneData.pos.x,
    //   coneData.pos.y));
    // }
  }

  // for (g2o::EdgeSE2 *edge : pose_to_pose_edges_) {
  //   optimizer_.addEdge(edge);
  // }
  for (size_t i = 0; i < pose_to_pose_edges_.size(); i++) {
    // Check if the edge exists already
    // if (pose_to_pose_edge_map_.find(
    //         std::make_pair(pose_to_pose_edges_[i]->vertex(0)->id(),
    //                        pose_to_pose_edges_[i]->vertex(1)->id())) ==
    //     pose_to_pose_edge_map_.end()) {
    //   optimizer_.addEdge(pose_to_pose_edges_[i]);
    // }

    optimizer_.addEdge(pose_to_pose_edges_[i]);
  }

  for (size_t i = 0; i < pose_to_cone_edges_.size(); i++) {
    optimizer_.addEdge(pose_to_cone_edges_[i]);
  }

  // for (g2o::EdgeSE2PointXY *edge : pose_to_cone_edges_) {
  //   optimizer_.addEdge(edge);
  // }

  optimizer_.initializeOptimization();
  optimizer_.optimize(1);
  do_graph_slam_ = false;

  for (auto v : optimizer_.vertices()) {
    g2o::VertexPointXY *vertexPointXY =
        dynamic_cast<g2o::VertexPointXY *>(v.second);
    if (vertexPointXY == nullptr) {
      g2o::VertexSE2 *vertexSE2 = dynamic_cast<g2o::VertexSE2 *>(v.second);
      fixed_pose_map_[v.first] =
          createPoseNode(v.first, vertexSE2->estimate().translation().x(),
                         vertexSE2->estimate().translation().y(),
                         vertexSE2->estimate().rotation().angle());
      fixed_pose_ids_.insert(v.first);
    }
    if (vertexPointXY) {

      // cone_id_to_vertex_map_[vertexPointXY->id()] = vertexPointXY;
      // Extract the x and y coordinates
      double x = vertexPointXY->estimate()(0);
      double y = vertexPointXY->estimate()(1);

      // Get the ID
      int id = v.first;
      int color = cone_id_to_color_map_[id];

      // Create a cone object then add it to the cone map
      utfr_msgs::msg::Cone cone;
      cone.pos.x = x;
      cone.pos.y = y;

      if (color == 1) {
        cone.type = utfr_msgs::msg::Cone::BLUE;
        cone_map_.left_cones.push_back(cone);
      } else if (color == 2) {
        cone.type = utfr_msgs::msg::Cone::YELLOW;
        cone_map_.right_cones.push_back(cone);
      } else if (color == 3) {
        cone.type = utfr_msgs::msg::Cone::SMALL_ORANGE;
        cone_map_.small_orange_cones.push_back(cone);
      } else if (color == 4) {
        cone.type = utfr_msgs::msg::Cone::LARGE_ORANGE;
        cone_map_.large_orange_cones.push_back(cone);
      }

      // if (fixed_cone_ids_.find(id) == fixed_cone_ids_.end()) {
      //   fixed_cone_map_[id] = new_cone;
      //   fixed_cone_ids_.insert(id);
      // }
    }
  }

  // Loop through all the edges and save them
  for (auto e : optimizer_.edges()) {
    g2o::EdgeSE2 *edgeSE2 = dynamic_cast<g2o::EdgeSE2 *>(e);
    if (edgeSE2) {
      int id1 = edgeSE2->vertex(0)->id();
      int id2 = edgeSE2->vertex(1)->id();
      std::string key = std::to_string(id1) + "_" + std::to_string(id2);
      pose_to_pose_edge_map_[key] = edgeSE2;
    }

    g2o::EdgeSE2PointXY *edgeSE2PointXY =
        dynamic_cast<g2o::EdgeSE2PointXY *>(e);
    if (edgeSE2PointXY) {
      int id1 = edgeSE2PointXY->vertex(0)->id();
      int id2 = edgeSE2PointXY->vertex(1)->id();
      std::string key = std::to_string(id1) + "_" + std::to_string(id2);
      pose_to_cone_edge_map_[key] = edgeSE2PointXY;
    }
  }

  cone_map_publisher_->publish(cone_map_);

  // Get the last pose from the optimizer then publish it
  g2o::VertexSE2 *lastPose = dynamic_cast<g2o::VertexSE2 *>(
      optimizer_.vertex(pose_nodes_.back()->id()));
  utfr_msgs::msg::PoseGraphData pose;
  pose.id = lastPose->id();
  pose.x = lastPose->estimate().translation().x();
  pose.y = lastPose->estimate().translation().y();
  pose.theta = lastPose->estimate().rotation().angle();
  slam_state_publisher_->publish(pose);

  optimizer_.clear();

  // // Loop through all the pose nodes from pose_nodes_.size() - POSE_WINDOW to
  // // the end and delete them
  // for (int i = 0; i < std::max(0, (int)pose_nodes_.size() - POSE_WINDOW - 1);
  //      i++) {
  //   delete pose_nodes_[i];
  // }

  // for (int i = 0;
  //      i < std::max(0, (int)pose_to_pose_edges_.size() - POSE_WINDOW - 1);
  //      i++) {
  //   delete pose_to_pose_edges_[i];
  // }

  // for (int i = 0;
  //      i < std::max(0, (int)pose_to_cone_edges_.size() - CONE_WINDOW - 1);
  //      i++) {
  //   delete pose_to_cone_edges_[i];
  // }
}

void ComputeGraphNode::timerCB() {

  // Check if data has been received
  if (data.states.size() == 0) {
    return;
  }

  int POSE_WINDOW = do_graph_slam_ ? 1500 : 500;
  int CONE_WINDOW = do_graph_slam_ ? 4500 : 1500;

  utfr_msgs::msg::PoseGraph local_data = data;
  pose_nodes_.clear();
  cone_nodes_.clear();
  pose_to_pose_edges_.clear();
  pose_to_cone_edges_.clear();

  do_graph_slam_ = local_data.run_slam;
  cone_map_.left_cones.clear();
  cone_map_.right_cones.clear();
  cone_map_.small_orange_cones.clear();
  cone_map_.large_orange_cones.clear();
  cone_map_.header.frame_id = "map";

  int startPoseIndex = std::max(0, (int)data.states.size() - POSE_WINDOW);
  for (int i = startPoseIndex; i < local_data.states.size(); i++) {
    utfr_msgs::msg::PoseGraphData pose = local_data.states[i];
    g2o::VertexSE2 *poseVertex =
        createPoseNode(pose.id, pose.x, pose.y, pose.theta);
    if (pose.id == 1001) {
      poseVertex->setFixed(true);
    }
    pose_id_to_vertex_map_[pose.id] = poseVertex;
    pose_nodes_.push_back(poseVertex);
  }

  for (utfr_msgs::msg::PoseGraphData cone : local_data.cones) {
    g2o::VertexPointXY *coneVertex = createConeVertex(cone.id, cone.x, cone.y);
    cone_id_to_vertex_map_[cone.id] = coneVertex;
    cone_nodes_.push_back(coneVertex);
  }

  for (int i = startPoseIndex; i < local_data.motion_edge.size(); i++) {

    utfr_msgs::msg::PoseGraphData edge = local_data.motion_edge[i];
    std::string key = std::to_string(edge.id) + "_" + std::to_string(edge.id2);

    if (pose_to_pose_edge_map_.find(key) != pose_to_pose_edge_map_.end()) {
      pose_to_pose_edges_.push_back(pose_to_pose_edge_map_[key]);
    } else {
      g2o::EdgeSE2 *res = addPoseToPoseEdge(
          pose_id_to_vertex_map_[edge.id], pose_id_to_vertex_map_[edge.id2],
          edge.dx, edge.dy, edge.dtheta, edge.loop_closure);
      pose_to_pose_edges_.push_back(res);
    }
  }

  int startConeIndex = std::max(0, (int)local_data.measurement_edges.size() - CONE_WINDOW);
  for (int i = startConeIndex; i < local_data.measurement_edges.size(); i++) {
    utfr_msgs::msg::PoseGraphData edge = local_data.measurement_edges[i];
    std::string key = std::to_string(edge.id) + "_" + std::to_string(edge.id2);
    if (pose_to_cone_edge_map_.find(key) != pose_to_cone_edge_map_.end()) {
      pose_to_cone_edges_.push_back(pose_to_cone_edge_map_[key]);
    } else {
      g2o::EdgeSE2PointXY *res =
          addPoseToConeEdge(pose_id_to_vertex_map_[edge.id],
                            cone_id_to_vertex_map_[edge.id2], edge.dx, edge.dy);
      pose_to_cone_edges_.push_back(res);
    }
  }

  for (size_t i = 0; i < local_data.color.size(); i++) {
    cone_id_to_color_map_[local_data.cone_id[i]] = local_data.color[i];
  }

  graphSLAM();
}

} // namespace compute_graph
} // namespace utfr_dv
