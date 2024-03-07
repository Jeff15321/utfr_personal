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

  // Will have to tune these later depending on the accuracy of our sensors
  Eigen::DiagonalMatrix<double, 3> P2P;
  Eigen::DiagonalMatrix<double, 2> P2C;
  Eigen::DiagonalMatrix<double, 3> LoopClosure;
  P2P.diagonal() << 200, 200, 2000;
  P2C.diagonal() << 100, 1000;
  LoopClosure.diagonal() << 100, 100, 1000;

  P2PInformationMatrix_ = P2P;
  P2CInformationMatrix_ = P2C;
  LoopClosureInformationMatrix_ = LoopClosure;

  auto linearSolverLM = std::make_unique<SlamLinearSolver>();
  linearSolverLM->setBlockOrdering(false);
  g2o::OptimizationAlgorithm *optimizer =
      new g2o::OptimizationAlgorithmLevenberg(
          std::make_unique<SlamBlockSolver>(std::move(linearSolverLM)));
  optimizer_.setAlgorithm(optimizer);

  slam_rate_ =
      this->get_parameter("slam_timer_").get_parameter_value().get<double>();
  update_rate_ = this->get_parameter("update_rate").as_double();
}

void ComputeGraphNode::initSubscribers() {
  pose_graph_subscriber_ = this->create_subscription<utfr_msgs::msg::PoseGraph>(
      topics::kPoseGraph, 1,
      std::bind(&ComputeGraphNode::poseGraphCB, this, std::placeholders::_1));
}

void ComputeGraphNode::initPublishers() {
  cone_map_publisher_ =
      this->create_publisher<utfr_msgs::msg::ConeMap>(topics::kConeMap, 10);
}

void ComputeGraphNode::initTimers() {
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(this->slam_rate_),
      std::bind(&ComputeGraphNode::timerCB, this));
}

void ComputeGraphNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kMappingComputeHeartbeat, 10);
  heartbeat_.module.data = "compute_graph_node";
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

g2o::EdgeSE2 *ComputeGraphNode::addPoseToPoseEdge(g2o::VertexSE2* pose1, g2o::VertexSE2* pose2,
                                                  double dx, double dy, double dtheta,
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

g2o::EdgeSE2PointXY *ComputeGraphNode::addPoseToConeEdge(g2o::VertexSE2* pose, g2o::VertexPointXY* cone, 
                                                          double dx, double dy) {
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
  pose_nodes_[0]->setFixed(true);

  // Add all the nodes and egdes to the optimizer
  for (g2o::VertexSE2 *pose : pose_nodes_) {
    optimizer_.addVertex(pose);
  }

  for (g2o::VertexPointXY *cone : cone_nodes_) {
    optimizer_.addVertex(cone);
  }

  for (g2o::EdgeSE2 *edge : pose_to_pose_edges_) {
    optimizer_.addEdge(edge);
  }

  for (int i = std::max(0, (int)pose_to_cone_edges_.size() - 10000);
       i < pose_to_cone_edges_.size(); i++) {
    optimizer_.addEdge(pose_to_cone_edges_[i]);
  }

  optimizer_.initializeOptimization();
  optimizer_.optimize(1);
  do_graph_slam_ = false;

  for (auto v : optimizer_.vertices()) {
    g2o::VertexPointXY *vertexPointXY =
        dynamic_cast<g2o::VertexPointXY *>(v.second);
    if (vertexPointXY) {
      // Extract the x and y coordinates
      double x = vertexPointXY->estimate()(0);
      double y = vertexPointXY->estimate()(1);

      // Get the ID
      int id = vertexPointXY->id();
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
    }
  }

  cone_map_publisher_->publish(cone_map_);
  optimizer_.save("pose_graph.g2o");
  optimizer_.clear();
}

void ComputeGraphNode::timerCB() {  

  // Check if data has been received
  if (data.states.size() == 0) {
    return;
  }

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

  if (!do_graph_slam_) {
    for (int i = 0; i < local_data.color.size(); i++) {
      utfr_msgs::msg::Cone cone;
      cone.pos.x = local_data.cones[i].x;
      cone.pos.y = local_data.cones[i].y;
      if (local_data.color[i] == 1) {
        cone.type = utfr_msgs::msg::Cone::BLUE;
        cone_map_.left_cones.push_back(cone);
      } else if (local_data.color[i] == 2) {
        cone.type = utfr_msgs::msg::Cone::YELLOW;
        cone_map_.right_cones.push_back(cone);
      } else if (local_data.color[i] == 3) {
        cone.type = utfr_msgs::msg::Cone::SMALL_ORANGE;
        cone_map_.small_orange_cones.push_back(cone);
      } else if (local_data.color[i] == 4) {
        cone.type = utfr_msgs::msg::Cone::LARGE_ORANGE;
        cone_map_.large_orange_cones.push_back(cone);
      }
    }

    cone_map_publisher_->publish(cone_map_);
    return;
  }

  for (utfr_msgs::msg::PoseGraphData pose : local_data.states) {
    g2o::VertexSE2 *poseVertex = createPoseNode(
        pose.id, pose.x, pose.y, pose.theta);
    pose_id_to_vertex_map_[pose.id] = poseVertex;
    pose_nodes_.push_back(poseVertex);
  }

  for (utfr_msgs::msg::PoseGraphData cone : local_data.cones) {
    g2o::VertexPointXY *coneVertex = createConeVertex(
        cone.id, cone.x, cone.y);
    cone_id_to_vertex_map_[cone.id] = coneVertex;
    cone_nodes_.push_back(coneVertex);
  }

  for (utfr_msgs::msg::PoseGraphData edge : local_data.motion_edge) {
    g2o::EdgeSE2 *res = addPoseToPoseEdge(
        pose_id_to_vertex_map_[edge.id],
        pose_id_to_vertex_map_[edge.id2], edge.dx, edge.dy, edge.dtheta,
        edge.loop_closure);
    pose_to_pose_edges_.push_back(res);
  }

  for (utfr_msgs::msg::PoseGraphData edge : local_data.measurement_edges) {
    g2o::EdgeSE2PointXY *res = addPoseToConeEdge(
        pose_id_to_vertex_map_[edge.id],
        cone_id_to_vertex_map_[edge.id2], edge.dx, edge.dy);
    pose_to_cone_edges_.push_back(res);
  }

  for (int i = 0; i < local_data.color.size(); i++) {
    cone_id_to_color_map_[local_data.cone_id[i]] = local_data.color[i];
  }

  graphSLAM();
}

} // namespace compute_graph
} // namespace utfr_dv
