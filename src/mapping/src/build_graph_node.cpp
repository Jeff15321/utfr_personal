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
#include <kd_tree_knn.hpp>
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

  // set heartbeat state to not ready
  HeartBeatState heartbeat_state_ = HeartBeatState::NOT_READY;
  heartbeat_rate_ = 1;
  this->initTimers();
  
  this->initHeartbeat();
  this->publishHeartbeat(); 
  this->initSubscribers();
  this->initPublishers();
  this->initParams();

  // set heartbeat state to active
  heartbeat_state_ = HeartBeatState::ACTIVE;
}

using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >;
using SlamLinearSolver =
    g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;

void BuildGraphNode::initParams() {
  loop_closed_ = false;
  landmarked_ = false;
  landmarkedID_ = -1;
  out_of_frame_ = -1;
  cones_found_ = 0;
  current_pose_id_ = 1000;
  first_detection_pose_id_ = 0;
  count_ = 0;
  cones_potential_= 0;
  globalKDTreePtr_ = nullptr;



  // Will have to tune these later depending on the accuracy of our sensors
  Eigen::DiagonalMatrix<double, 3> P2P;
  Eigen::DiagonalMatrix<double, 2> P2C;
  Eigen::DiagonalMatrix<double, 3> LoopClosure;
  P2P.diagonal() << 20, 20, 200;
  P2C.diagonal() << 10, 100;
  LoopClosure.diagonal() << 100, 100, 1000;


  P2PInformationMatrix_ = P2P;
  P2CInformationMatrix_ = P2C;
  LoopClosureInformationMatrix_ = LoopClosure;

  auto linearSolverLM = std::make_unique<SlamLinearSolver>();
  linearSolverLM->setBlockOrdering(false);
  g2o::OptimizationAlgorithm* optimizer = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<SlamBlockSolver>(std::move(linearSolverLM)));
  optimizer_.setAlgorithm(optimizer);

  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = this->get_clock()->now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "map";

  // Set the translation and rotation of the transform
  transformStamped.transform.translation.x = 0;
  transformStamped.transform.translation.y = 0;
  transformStamped.transform.translation.z = 0;

  transformStamped.transform.rotation.x = 0;
  transformStamped.transform.rotation.y = 0;
  transformStamped.transform.rotation.z = 0;
  transformStamped.transform.rotation.w = 0;

  // Broadcast the transform
  broadcaster_->sendTransform(transformStamped);
}

void BuildGraphNode::initSubscribers() {
  cone_detection_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ConeDetections>(
          topics::kConeDetections, 1,
          std::bind(&BuildGraphNode::coneDetectionCB, this,
                    std::placeholders::_1));

  state_estimation_subscriber_ =
      this->create_subscription<utfr_msgs::msg::EgoState>(
          topics::kPose, 1,
          std::bind(&BuildGraphNode::stateEstimationCB, this,
                    std::placeholders::_1));
}

void BuildGraphNode::initPublishers() {
  pose_graph_publisher_ =
      this->create_publisher<utfr_msgs::msg::PoseGraph>(topics::kPoseGraph, 10);

  cone_map_publisher_ =
      this->create_publisher<utfr_msgs::msg::ConeMap>(topics::kConeMap, 10);

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void BuildGraphNode::initTimers() {
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(heartbeat_rate_),
        std::bind(&BuildGraphNode::publishHeartbeat, this));
  }

void BuildGraphNode::initHeartbeat() {
    heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
        topics::kMappingBuildHeartbeat, 10);
  }


void BuildGraphNode::publishHeartbeat() {
    utfr_msgs::msg::Heartbeat heartbeat_msg;
    heartbeat_msg.status = static_cast<uint8_t>(heartbeat_state_);  
    heartbeat_msg.header.stamp = this->now();  

    heartbeat_publisher_->publish(heartbeat_msg);
  }


void BuildGraphNode::coneDetectionCB(const utfr_msgs::msg::ConeDetections msg) {
  std::vector<int> cones = KNN(msg);
  loopClosure(cones);
  if (cones_found_ > 10) {
    graphSLAM();
  }
}


void BuildGraphNode::stateEstimationCB(const utfr_msgs::msg::EgoState msg) {

  //Print the x and y position of the car
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
  pose_to_pose_edges_.push_back(edge);
}


kd_tree_knn::KDTree generateKDTree(std::vector < std::tuple < double, double, double >> points_tuple) {
  // Convert tuples to points
  std::vector < kd_tree_knn::Point > points;
  for (const auto & tuple: points_tuple) {
    points.emplace_back(std::get<0>(tuple), std::get<1>(tuple), std::get<2>(tuple));
  }
  // Create a KD tree
  kd_tree_knn::KDTree globalKDTree(points);

  return globalKDTree;
}


std::vector<int> BuildGraphNode::KNN(const utfr_msgs::msg::ConeDetections &cones){
    std::vector<int> cones_id_list_;  
    std::vector<utfr_msgs::msg::Cone> all_cones;
    std::vector<std::tuple<double, double>> current_round_cones_;


    // Create a temp current state so current state doesn't get modified while we're using it
    utfr_msgs::msg::EgoState temp_current_state_ = current_state_;
    float yaw = utfr_dv::util::quaternionToYaw(temp_current_state_.pose.pose.orientation);
    int temp_current_pose_id_ = current_pose_id_;
    //adding all cones to one vector
    all_cones.insert(all_cones.end(), cones.left_cones.begin(), cones.left_cones.end());
    all_cones.insert(all_cones.end(), cones.right_cones.begin(), cones.right_cones.end());
    all_cones.insert(all_cones.end(), cones.large_orange_cones.begin(), cones.large_orange_cones.end());
    all_cones.insert(all_cones.end(), cones.small_orange_cones.begin(), cones.small_orange_cones.end());
    
    // Iterating through all detected cones
    for (const auto& newCone : all_cones) {
        // Updating detected position to global frame
        double ego_x = temp_current_state_.pose.pose.position.x;
        double ego_y = temp_current_state_.pose.pose.position.y;
        double position_x_ = ego_x + newCone.pos.x * cos(yaw) - newCone.pos.y * sin(yaw);
        double position_y_ = ego_y + newCone.pos.x * sin(yaw) + newCone.pos.y * cos(yaw);

        // Check if the KD tree is not created, and create it
        if (globalKDTreePtr_ == nullptr) {
        // Update vars
          past_detections_.emplace_back(cones_found_,newCone);
          cones_id_list_.push_back(cones_found_);
          cone_id_to_color_map_[cones_found_] = newCone.type;
          g2o::VertexPointXY* vertex = createConeVertex(cones_found_, position_x_, position_y_);
          cone_nodes_.push_back(vertex);
          g2o::EdgeSE2PointXY* edge = addPoseToConeEdge(id_to_pose_map_[temp_current_pose_id_], vertex, newCone.pos.x, newCone.pos.y);
          pose_to_cone_edges_.push_back(edge);        
          globalKDTreePtr_ = std::make_unique<kd_tree_knn::KDTree>(generateKDTree({std::make_tuple(position_x_, position_y_, cones_found_)}));
          cones_found_ += 1;
          continue;
        }

        int number_cones = globalKDTreePtr_ -> getNumberOfCones();

        if (number_cones == 1){
          past_detections_.emplace_back(cones_found_,newCone);
          cones_id_list_.push_back(cones_found_);
          cone_id_to_color_map_[cones_found_] = newCone.type;
          g2o::VertexPointXY* vertex = createConeVertex(cones_found_, position_x_, position_y_);
          cone_nodes_.push_back(vertex);
          g2o::EdgeSE2PointXY* edge = addPoseToConeEdge(id_to_pose_map_[temp_current_pose_id_], vertex, newCone.pos.x, newCone.pos.y);
          pose_to_cone_edges_.push_back(edge);
          kd_tree_knn::Point newPoint(position_x_,position_y_, cones_found_);
          globalKDTreePtr_ -> insert(newPoint);
          cones_found_ += 1;
          continue;
        }

        // Use KNN search to find the nearest cone
        kd_tree_knn::Point knnResult = globalKDTreePtr_ -> KNN(kd_tree_knn::Point(position_x_, position_y_, 0));

        const kd_tree_knn::Point& nearestCone = knnResult;

        // Check the result of the nearest neighbour search and calculate displacement
        if (knnResult != kd_tree_knn::Point(0.0, 0.0, 0.0)){
        
          // Use the nearest point
            double displacement = utfr_dv::util::euclidianDistance2D(position_x_, nearestCone.x, position_y_, nearestCone.y);

            // Do not add if its within 0.3 of an already seen cone
            if (displacement <= 0.3) {
                // Add the ID to the list
                cones_id_list_.push_back(nearestCone.id);
                continue;
            }

            // Do not add if its within 0.3 of another cone deteced within the current call of the function (rejecting duplicate detections)
            double comparative_displacement = 0.0;
            bool is_duplicate_ = false;
            for (const auto& duplicates_potential : current_round_cones_){
              comparative_displacement = utfr_dv::util::euclidianDistance2D(position_x_, std::get<0>(duplicates_potential), position_y_, std::get<1>(duplicates_potential));
              if (comparative_displacement <= 0.3){
                is_duplicate_ = true;
                break;
              }
            }
            if (is_duplicate_){
              continue;
            }
        }
        // Add to duplicate checker
        current_round_cones_.emplace_back(std::make_tuple(position_x_, position_y_));

        // Add to potential_cones (used for checking if cone can be added to past_detections)
        potential_cones_.insert(std::make_pair(cones_potential_, std::make_tuple(position_x_, position_y_)));

        std::vector<int> keys{};

        // Check if same cone detected in three different time instances
        for (const auto& pair : potential_cones_) {
            int key_ = pair.first;
            const std::tuple<double,double> potentialPoint = pair.second;
            double temp_displacement_ = utfr_dv::util::euclidianDistance2D(position_x_, std::get<0>(potentialPoint), position_y_, std::get<1>(potentialPoint));

            // Check if cone within 0.3 of any other new detected cone
            if (temp_displacement_ <= 0.3) {
                    count_ += 1;
                    keys.push_back(key_);
                    // Check if three of same detected
                    if (count_ == 3) {
                        // FIX: Delete all of the same cone from potential_cones_ if three detected
                        for (int key_number_ : keys) {
                            potential_cones_.erase(key_number_);
                        }
                        keys.clear();
                      
                        // Update cone_id_list_ and past_detections_ and KD tree
                        cones_id_list_.push_back(cones_found_);
                        cone_id_to_color_map_[cones_found_] = newCone.type;
                        g2o::VertexPointXY* vertex = createConeVertex(cones_found_, position_x_, position_y_);
                        cone_nodes_.push_back(vertex);
                        g2o::EdgeSE2PointXY* edge = addPoseToConeEdge(id_to_pose_map_[temp_current_pose_id_], vertex, newCone.pos.x, newCone.pos.y);
                        pose_to_cone_edges_.push_back(edge);
                        kd_tree_knn::Point newPoint(position_x_,position_y_, cones_found_);
                      
                        // std::cout << "Cone x: " << position_x_ << " Cone y: " << position_y_ << " Cone id: " << cones_found_ << std::endl;
                        past_detections_.emplace_back(cones_found_,newCone);
                        globalKDTreePtr_ -> insert(newPoint);
                        cones_found_ += 1;
                        break;
                        }
                    }
            }

        count_ = 0;
        cones_potential_ += 1;
      }
    return cones_id_list_;
  }


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
          out_of_frame_ = 0;
        }
      }

      // if (utfr_dv::util::isLargeOrangeCone(coneID) && !landmarked_) {
      //   // Set first large orange cone listed to be landmark cone
      //   landmarkedID_ = coneID;
      //   landmarked_ = true;
      // }
      // Cone has been landmarked and still in frame for the first time
      if (landmarked_ == true && out_of_frame_ > -1 && out_of_frame_ < 1000) {
        auto seen_status = std::find(cones.begin(), cones.end(), landmarkedID_);
        // If cone not in frame anymore
        if (seen_status == cones.end()) {
          // Set out of frame to be true
          out_of_frame_ += 1;
        } else {
          // Set out of frame to be false
          out_of_frame_ = 0;
        }
      }
      // Cone has been landmarked and is no longer in frame
      if (landmarked_ == true && out_of_frame_ >= 1000) {
        // Check if cone appears in frame again
        auto seen_status = std::find(cones.begin(), cones.end(), landmarkedID_);
        // If cone in frame again
        if (seen_status != cones.end()) {
          // Car has returned back to landmark cone position, made full loop

          double dx = id_to_ego_map_[current_pose_id_].pose.pose.position.x - 
                      id_to_ego_map_[first_detection_pose_id_].pose.pose.position.x;
          double dy = id_to_ego_map_[current_pose_id_].pose.pose.position.y-
                      id_to_ego_map_[first_detection_pose_id_].pose.pose.position.y;
          double dtheta = utfr_dv::util::quaternionToYaw(id_to_ego_map_[current_pose_id_].pose.pose.orientation) - 
                          utfr_dv::util::quaternionToYaw(id_to_ego_map_[first_detection_pose_id_].pose.pose.orientation);
          loop_closed_ = true;
          // Create node objects for each pose
          // Get the state estimate at a pose using id_to_state_map_[pose_id_]

          g2o::VertexSE2* first_pose_node = id_to_pose_map_[first_detection_pose_id_];
          g2o::VertexSE2* second_pose_node = id_to_pose_map_[current_pose_id_];

          // add an edge using the pose ids at initial detection and loop closure detection
          g2o::EdgeSE2* edge = addPoseToPoseEdge(first_pose_node, second_pose_node, dx, dy, dtheta, loop_closed_);
          pose_to_pose_edges_.push_back(edge);

          landmarked_ = false;
          landmarkedID_ = -1;
          out_of_frame_ = -1;
          first_detection_pose_id_ = 0;
          loop_closed_ = false;
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

  // Set the fixed node as 1001
  pose_nodes_[0]->setFixed(true);

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
  // optimizer_.optimize(10);

  cone_map_.left_cones.clear();
  cone_map_.right_cones.clear();
  cone_map_.small_orange_cones.clear();
  cone_map_.large_orange_cones.clear();
  cone_map_.header.frame_id = "map";

  for (auto v : optimizer_.vertices()) {
      g2o::VertexPointXY* vertexPointXY = dynamic_cast<g2o::VertexPointXY*>(v.second);
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
          cone.pos.y = -y;

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
  // Save the optimized pose graph
  // std::cout << "Optimized pose graph saved" << std::endl;
}

void BuildGraphNode::buildGraph() {}


} // namespace build_graph
} // namespace utfr_dv