/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: build_graph_node.cpp
* auth: Arthur Xu
* desc: build graph node class
*/

#include <build_graph_node.hpp>
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/sparse_optimizer.h"
#include <Eigen/Core>
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

G2O_USE_TYPE_GROUP(slam2d);

namespace utfr_dv {
namespace build_graph {

BuildGraphNode::BuildGraphNode() : Node("build_graph_node") {
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >;
using SlamLinearSolver =
    g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;

void BuildGraphNode::initParams() {
  loop_closed_ = false;
  landmarked_ = false;
  landmarkedID_ = -1;
  out_of_frame_ = false;
  cones_found_ = 0;
  current_pose_id_ = 0;
  first_detection_pose_id_ = 0;
  

  // Will have to tune these later depending on the accuracy of our sensors
  P2PInformationMatrix_ = Eigen::Matrix3d::Identity();
  P2CInformationMatrix_ = Eigen::Matrix2d::Identity();
  LoopClosureInformationMatrix_ = Eigen::Matrix3d::Identity();

  auto linearSolverLM = std::make_unique<SlamLinearSolver>();
  linearSolverLM->setBlockOrdering(false);
  g2o::OptimizationAlgorithm* optimizer = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<SlamBlockSolver>(std::move(linearSolverLM)));
  optimizer_.setAlgorithm(optimizer);

}

void BuildGraphNode::initSubscribers() {
  cone_detection_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ConeDetections>(
          topics::kConeDetections, 1,
          std::bind(&BuildGraphNode::coneDetectionCB, this,
                    std::placeholders::_1));

  state_estimation_subscriber_ =
      this->create_subscription<utfr_msgs::msg::EgoState>(
          topics::kEgoState, 1,
          std::bind(&BuildGraphNode::stateEstimationCB, this,
                    std::placeholders::_1));
}

void BuildGraphNode::initPublishers() {
  pose_graph_publisher_ =
      this->create_publisher<utfr_msgs::msg::PoseGraph>(topics::kPoseGraph, 10);
}

void BuildGraphNode::initTimers() {
}

void BuildGraphNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kMappingBuildHeartbeat, 10);
}

void BuildGraphNode::coneDetectionCB(const utfr_msgs::msg::ConeDetections msg) {
  std::vector<int> cones = KNN(msg);
  // Print out the cones
  for (int cone : cones) {
    std::cout << cone << std::endl;
  }
  loopClosure(cones);
}

void BuildGraphNode::stateEstimationCB(const utfr_msgs::msg::EgoState msg) {
  current_state_ = msg;

  current_pose_id_ += 1;
  id_to_ego_map_[current_pose_id_] = current_state_;
  
  g2o::VertexSE2* poseVertex = createPoseNode(current_pose_id_, msg.pose.pose.position.x, msg.pose.pose.position.y, utfr_dv::util::quaternionToYaw(msg.pose.pose.orientation));
  pose_nodes_.push_back(poseVertex);
  id_to_pose_map_[current_pose_id_] = poseVertex;

  utfr_msgs::msg::EgoState prevPoseVertex = id_to_ego_map_[current_pose_id_ - 1];

  double dx = current_state_.pose.pose.position.x - prevPoseVertex.pose.pose.position.x;
  double dy = current_state_.pose.pose.position.y - prevPoseVertex.pose.pose.position.y;
  double dtheta = utfr_dv::util::quaternionToYaw(current_state_.pose.pose.orientation) - utfr_dv::util::quaternionToYaw(prevPoseVertex.pose.pose.orientation);

  g2o::EdgeSE2* edge = addPoseToPoseEdge(id_to_pose_map_[current_pose_id_ - 1], poseVertex, dx, dy, dtheta, false);
}

std::vector<int> BuildGraphNode::KNN(const utfr_msgs::msg::ConeDetections &cones){
    std::vector<int> cones_id_list_;  
    std::vector<utfr_msgs::msg::Cone> all_cones;
    //adding all cones to one vector
    all_cones.insert(all_cones.end(), cones.left_cones.begin(), cones.left_cones.end());
    all_cones.insert(all_cones.end(), cones.right_cones.begin(), cones.right_cones.end());
    all_cones.insert(all_cones.end(), cones.large_orange_cones.begin(), cones.large_orange_cones.end());
    all_cones.insert(all_cones.end(), cones.small_orange_cones.begin(), cones.small_orange_cones.end());
    //iterating through all detected cones
    for (const auto& pastCone : all_cones){
      utfr_msgs::msg::Cone newCone = pastCone;
      //updating detected position to global frame
      double ego_x = current_state_.pose.pose.position.x;
      double ego_y = current_state_.pose.pose.position.y;
      newCone.pos.x += ego_x;
      newCone.pos.y += ego_y;
      bool adding_to_past = true;      
      //iterating through old cones
      for (size_t i=0; i <past_detections_.size(); ++i){
        const utfr_msgs::msg::Cone&pastDetectionsCone=past_detections_[i].second;    
        //finding displacement between detected cone and past cone    
        double displacement = utfr_dv::util::euclidianDistance2D(newCone.pos.x, pastDetectionsCone.pos.x, newCone.pos.y, pastDetectionsCone.pos.y);    
        //not adding if already detected (within error)
        if (displacement <= 0.3){
          adding_to_past=false;

          // Maps the cone detection to the cone id
          utfr_msgs::msg::Cone detection = newCone;
          detection.pos.x -= current_state_.pose.pose.position.x;
          detection.pos.y -= current_state_.pose.pose.position.y;
          id_to_cone_map_[past_detections_[i].first] = detection;

          // Create edges between pose and cone
          g2o::EdgeSE2PointXY* edge = addPoseToConeEdge(id_to_pose_map_[current_pose_id_], cone_id_to_vertex_map_[past_detections_[i].first], detection.pos.x, detection.pos.y);

          break;
        }
      }
      if (adding_to_past){
        // adding to id_list_ and past_detections_ if past error for all previously detected cones
        cones_id_list_.push_back(cones_found_);
        past_detections_.emplace_back(cones_found_,newCone);

        // Maps the cone detection to the cone id
        utfr_msgs::msg::Cone detection = newCone;
        detection.pos.x -= current_state_.pose.pose.position.x;
        detection.pos.y -= current_state_.pose.pose.position.y;
        id_to_cone_map_[cones_found_] = detection;

        cones_found_+=1;
        
        // creates vertex for new cone detections and add to vertices array
        g2o::VertexPointXY* vertex = createConeVertex(cones_found_, detection.pos.x, detection.pos.y);
        cone_nodes_.push_back(vertex);

        // maps cone id to vertex
        cone_id_to_vertex_map_[cones_found_] = vertex;

        // Create edges between pose and cone
        g2o::EdgeSE2PointXY* edge = addPoseToConeEdge(id_to_pose_map_[current_pose_id_], vertex, detection.pos.x, detection.pos.y);
      }
    }
    return cones_id_list_;  }


void BuildGraphNode::loopClosure(const std::vector<int> &cones) {
  // Loop hasn't been completed yet
  if (!loop_closed_) {
    // Go through cone list
    for (int coneID : cones) {
      // No landmark cone yet
      // We gotta fix this later when Mark finishes his thing.
      // IDK why I chose past_detections_ to be a vector of pairs
      // but it should probably be a map :(
      for (auto cone : past_detections_) {
        if (cone.first == coneID && utfr_dv::util::isLargeOrangeCone(cone.second.type) && !landmarked_) {
          // Set first large orange cone listed to be landmark cone
          landmarkedID_ = coneID;
          landmarked_ = true;
          first_detection_pose_id_ = current_pose_id_;
          std::cout << "Landmark Set" << std::endl;
        }
      }

      // if (utfr_dv::util::isLargeOrangeCone(coneID) && !landmarked_) {
      //   // Set first large orange cone listed to be landmark cone
      //   landmarkedID_ = coneID;
      //   landmarked_ = true;
      // }
      // Cone has been landmarked and still in frame for the first time
      if (landmarked_ == true && out_of_frame_ == false) {
        auto seen_status = std::find(cones.begin(), cones.end(), landmarkedID_);
        // If cone not in frame anymore
        if (seen_status == cones.end()) {
          // Set out of frame to be true
          out_of_frame_ = true;
        }
      }
      // Cone has been landmarked and is no longer in frame
      if (landmarked_ == true && out_of_frame_ == true) {
        // Check if cone appears in frame again
        auto seen_status = std::find(cones.begin(), cones.end(), landmarkedID_);
        // If cone in frame again
        if (seen_status != cones.end()) {
          // Car has returned back to landmark cone position, made full loop
          double dx = id_to_ego_map_[first_detection_pose_id_].pose.pose.position.x - 
          	      id_to_ego_map_[current_pose_id_].pose.pose.position.x;
          double dy = id_to_ego_map_[first_detection_pose_id_].pose.pose.position.y-
          	      id_to_ego_map_[current_pose_id_].pose.pose.position.y;
          double dtheta = current_state_.pose.pose.orientation.z;
          loop_closed_ = true;
          // Create node objects for each pose
          // Get the state estimate at a pose using id_to_state_map_[pose_id_]

          g2o::VertexSE2* first_pose_node = id_to_pose_map_[first_detection_pose_id_];
          g2o::VertexSE2* second_pose_node = id_to_pose_map_[current_pose_id_];

          // add an edge using the pose ids at initial detection and loop closure detection
          g2o::EdgeSE2* edge = addPoseToPoseEdge(first_pose_node, second_pose_node, dx, dy, dtheta, loop_closed_);
          pose_to_pose_edges_.push_back(edge);
          std::cout << "Loop closure edge added" << std::endl;
          graphSLAM();
        }
      }
    }
  }
}

g2o::VertexSE2* BuildGraphNode::createPoseNode(int id, double x, double y,
                                              double theta) {
  g2o::VertexSE2* poseVertex = new g2o::VertexSE2();
  poseVertex->setId(id);

  g2o::SE2 poseEstimate;
  poseEstimate.fromVector(Eigen::Vector3d(x, y, theta));
  poseVertex->setEstimate(poseEstimate);

  return poseVertex;
}

g2o::VertexPointXY* BuildGraphNode::createConeVertex(int id, double x, double y) {

  g2o::VertexPointXY* coneVertex = new g2o::VertexPointXY();
  coneVertex->setId(id);

  Eigen::Vector2d position(x, y);
  coneVertex->setEstimate(position);
  return coneVertex;
}

g2o::EdgeSE2* BuildGraphNode::addPoseToPoseEdge(g2o::VertexSE2* pose1, g2o::VertexSE2* pose2,
                                               double dx, double dy,
                                               double dtheta, bool loop_closure) {
  g2o::EdgeSE2* edge = new g2o::EdgeSE2();

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

g2o::EdgeSE2PointXY* BuildGraphNode::addPoseToConeEdge(g2o::VertexSE2* pose,
                                                      g2o::VertexPointXY* cone,
                                                      double dx, double dy) {
  g2o::EdgeSE2PointXY* edge = new g2o::EdgeSE2PointXY();

  Eigen::Vector2d measurement(dx, dy);
  edge->setMeasurement(measurement);
  edge->setInformation(P2CInformationMatrix_);

  edge->setVertex(0, pose);
  edge->setVertex(1, cone);

  return edge;
}

void BuildGraphNode::graphSLAM() {

  std::cout << "Edges and nodes cleared" << std::endl;

  // Add all the nodes and egdes to the optimizer
  for (g2o::VertexSE2* pose : pose_nodes_) {
    optimizer_.addVertex(pose);
  }

  for (g2o::VertexPointXY* cone : cone_nodes_) {
    optimizer_.addVertex(cone);
  }

  for (g2o::EdgeSE2* edge : pose_to_pose_edges_) {
    optimizer_.addEdge(edge);
  }

  for (g2o::EdgeSE2PointXY* edge : pose_to_cone_edges_) {
    optimizer_.addEdge(edge);
  }
  
  optimizer_.initializeOptimization();
  optimizer_.optimize(10);

  // for (g2o::SparseOptimizer::VertexIDMap::const_iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it) {
  //       g2o::SparseOptimizer::Vertex* vertex = dynamic_cast<g2o::SparseOptimizer::Vertex*>(it->second);
  //       if (vertex) {
  //           g2o::VertexSE2* se2Vertex = dynamic_cast<g2o::VertexSE2*>(vertex);
  //           if (se2Vertex) {
  //               const g2o::SE2& se2 = se2Vertex->estimate();
  //               double x = se2.toVector()[0];  // Extract the x component
  //               double y = se2.toVector()[1];  // Extract the y component

  //               utfr_msgs::msg::Cone cone;
  //               cone.pos.x = x;
  //               cone.pos.y = y;
  //               cone_map_.left_cones.push_back(cone);
  //           }
  //       }
  //   }
  // Save the optimized pose graph
  optimizer_.save("pose_graph.g2o");
  std::cout << "Optimized pose graph saved" << std::endl;
}

void BuildGraphNode::buildGraph() {}

} // namespace build_graph
} // namespace utfr_dv
