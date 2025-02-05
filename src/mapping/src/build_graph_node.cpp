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

namespace utfr_dv {
namespace build_graph {

BuildGraphNode::BuildGraphNode() : Node("build_graph_node") {
  this->initParams();
  this->initHeartbeat();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::NOT_READY);
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
  cone_viz_publisher_->publish(marker_array);
}

using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
using SlamLinearSolver =
    g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;

void BuildGraphNode::initParams() {
  this->declare_parameter("update_rate", 100.00);
  update_rate_ = this->get_parameter("update_rate").as_double();
  this->declare_parameter("mapping_mode", 0);
  mapping_mode_ = this->get_parameter("mapping_mode").as_int();
  this->declare_parameter("displacement_radius", 0.0);
  displacement_radius_ = this->get_parameter("displacement_radius").as_double();
  this->declare_parameter("count_threshold", 0);
  count_threshold_ = this->get_parameter("count_threshold").as_int();
  
  this->declare_parameter("loop_closed", false);
  loop_closed_ = this->get_parameter("loop_closed").as_bool();
  landmarked_cone_id_ = std::nullopt;
  this->declare_parameter("out_of_frame", -1);
  out_of_frame_ = this->get_parameter("out_of_frame").as_int();
  this->declare_parameter("cones_found", 0);
  cones_found_ = this->get_parameter("cones_found").as_int();
  this->declare_parameter("current_pose_id", 1000);
  current_pose_id_ = this->get_parameter("current_pose_id").as_int();
  this->declare_parameter("first_detection_pose_id", 0);
  first_detection_pose_id_ = this->get_parameter("first_detection_pose_id").as_int();
  this->declare_parameter("count", 0);
  count_ = this->get_parameter("count").as_int();
  this->declare_parameter("cones_potential", 0);
  cones_potential_ = this->get_parameter("cones_potential").as_int();
  globalKDTreePtr_ = nullptr;
  this->declare_parameter("do_graph_slam", false);
  do_graph_slam_ = this->get_parameter("do_graph_slam").as_bool();
  this->declare_parameter("closed_loop_once_data", false);
  closed_loop_once.data = this->get_parameter("closed_loop_once_data").as_bool();

  geometry_msgs::msg::TransformStamped transformStamped;

  transformStamped.header.stamp = this->get_clock()->now();
  this->declare_parameter("transformStamped_header_frameid", "world");
  transformStamped.header.frame_id = this->get_parameter("transformStamped_header_frameid").as_string();
  this->declare_parameter("transformStamped_child_frameid", "map");
  transformStamped.child_frame_id = this->get_parameter("transformStamped_child_frameid").as_string();

  // Set the translation and rotation of the transform
  this->declare_parameter("transformStamped_translation_x", 0);
  transformStamped.transform.translation.x = this->get_parameter("transformStamped_translation_x").as_int();
  this->declare_parameter("transformStamped_translation_y", 0);
  transformStamped.transform.translation.y = this->get_parameter("transformStamped_translation_y").as_int();
  this->declare_parameter("transformStamped_translation_z", 0);
  transformStamped.transform.translation.z = this->get_parameter("transformStamped_translation_z").as_int();

  this->declare_parameter("transformStamped_rotation_x", 0);
  transformStamped.transform.rotation.x = this->get_parameter("transformStamped_rotation_x").as_int();
  this->declare_parameter("transformStamped_rotation_y", 0);
  transformStamped.transform.rotation.y = this->get_parameter("transformStamped_rotation_y").as_int();
  this->declare_parameter("transformStamped_rotation_z", 0);
  transformStamped.transform.rotation.z = this->get_parameter("transformStamped_rotation_z").as_int();
  this->declare_parameter("transformStamped_rotation_w", 1.0);
  transformStamped.transform.rotation.w = this->get_parameter("transformStamped_rotation_w").as_double();

  this->declare_parameter("comparative_displacement", 0.0);
  comparative_displacement = this->get_parameter("comparative_displacement").as_double();
  this->declare_parameter("is_duplicate", false);
  is_duplicate_ = this->get_parameter("is_duplicate").as_bool();
  this->declare_parameter("true_coordinate_x", 0.0);
  true_coordinate_x = this->get_parameter("true_coordinate_x").as_double();
  this->declare_parameter("true_coordinate_y", 0.0);
  true_coordinate_y = this->get_parameter("true_coordinate_y").as_double();

  // Broadcast the transform
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  broadcaster_->sendTransform(transformStamped);
}

void BuildGraphNode::initSubscribers() {
  if (mapping_mode_ == 0) {
    cone_detection_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ConeDetections>(
          topics::kConeDetections, 1,
          std::bind(&BuildGraphNode::coneDetectionCB, this,
                    std::placeholders::_1));

  state_estimation_subscriber_ =
      this->create_subscription<utfr_msgs::msg::EgoState>(
          "perception_ego_state", 1,
          std::bind(&BuildGraphNode::stateEstimationCB, this,
                    std::placeholders::_1));
  }
}

void BuildGraphNode::initPublishers() {
  if (mapping_mode_ == 0) {
    pose_graph_publisher_ =
      this->create_publisher<utfr_msgs::msg::PoseGraph>(topics::kPoseGraph, 10);

    // cone_map_publisher_ =
    //     this->create_publisher<utfr_msgs::msg::ConeMap>(topics::kConeMap, 10);

    loop_closure_publisher_ =
        this->create_publisher<std_msgs::msg::Bool>(topics::kLoopClosed, 10);
  }

  cone_viz_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("ConeViz", 10);

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
  cone_viz_publisher_->publish(marker_array);
}

void BuildGraphNode::initTimers() {
  if (mapping_mode_ == 0) {
    main_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(this->update_rate_),
      std::bind(&BuildGraphNode::timerCB, this));
  }
}

void BuildGraphNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kMappingBuildHeartbeat, 10);
  heartbeat_.module.data = "mapping_build";
  heartbeat_.update_rate = update_rate_;
}

void BuildGraphNode::publishHeartbeat(const int status) {
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_publisher_->publish(heartbeat_);
}

void BuildGraphNode::coneDetectionCB(const utfr_msgs::msg::ConeDetections msg) {
  current_cone_detections_ = msg;
}

void BuildGraphNode::stateEstimationCB(const utfr_msgs::msg::EgoState msg) {

  current_ego_state_ = msg;
}

kd_tree_knn::KDTree
generateKDTree(std::vector<std::tuple<double, double, double>> points_tuple) {
  // Convert tuples to points
  std::vector<kd_tree_knn::Point> points;
  for (const auto &tuple : points_tuple) {
    points.emplace_back(std::get<0>(tuple), std::get<1>(tuple),
                        std::get<2>(tuple));
  }
  // Create a KD tree
  kd_tree_knn::KDTree globalKDTree(points);

  return globalKDTree;
}

std::vector<int>
BuildGraphNode::KNN(const utfr_msgs::msg::ConeDetections &cones) {
  std::vector<int> cones_id_list_;
  std::vector<utfr_msgs::msg::Cone> all_cones;
  std::vector<std::tuple<double, double>> current_round_cones_;

  // Create a temp current state so current state doesn't get modified while
  // we're using it
  utfr_msgs::msg::EgoState temp_current_state_ = current_state_;
  float yaw =
      utfr_dv::util::quaternionToYaw(temp_current_state_.pose.pose.orientation);
  int temp_current_pose_id_ = current_pose_id_;
  // adding all cones to one vector
  all_cones.insert(all_cones.end(), cones.left_cones.begin(),
                   cones.left_cones.end());
  all_cones.insert(all_cones.end(), cones.right_cones.begin(),
                   cones.right_cones.end());
  all_cones.insert(all_cones.end(), cones.large_orange_cones.begin(),
                   cones.large_orange_cones.end());
  all_cones.insert(all_cones.end(), cones.small_orange_cones.begin(),
                   cones.small_orange_cones.end());

  // Iterating through all detected cones
  for (const auto &newCone : all_cones) {
    // Updating detected position to global frame
    double ego_x = temp_current_state_.pose.pose.position.x;
    double ego_y = temp_current_state_.pose.pose.position.y;
    double position_x_ =
        ego_x + newCone.pos.x * cos(yaw) - newCone.pos.y * sin(yaw);
    double position_y_ =
        ego_y + newCone.pos.x * sin(yaw) + newCone.pos.y * cos(yaw);

    int colour = newCone.type;
    // Check if the KD tree is not created, and create it
    if (globalKDTreePtr_ == nullptr) {
      // Update vars
      past_cone_detections_[cones_found_] = newCone;
      cones_id_list_.push_back(cones_found_);
      cone_id_to_color_map_[cones_found_] = newCone.type;

      utfr_msgs::msg::PoseGraphData cone_data;
      cone_data.id = cones_found_;
      cone_data.x = position_x_;
      cone_data.y = position_y_;
      cone_nodes_[cones_found_] = (cone_data);
      average_position_[cones_found_] = {position_x_, position_y_, 1};
      cone_id_to_vertex_map_[cones_found_] = cone_data;

      utfr_msgs::msg::PoseGraphData edge_data;
      edge_data.id = temp_current_pose_id_;
      edge_data.id2 = cones_found_;
      edge_data.dx = newCone.pos.x;
      edge_data.dy = newCone.pos.y;
      pose_to_cone_edges_.push_back(edge_data);
      globalKDTreePtr_ = std::make_unique<kd_tree_knn::KDTree>(generateKDTree(
          {std::make_tuple(position_x_, position_y_, cones_found_)}));
      cones_found_ += 1;
      detection_counts[cones_found_] = 1;
      continue;
    }

    int number_cones = globalKDTreePtr_->getNumberOfCones();

    if (number_cones == 1) {
      past_cone_detections_[cones_found_] = newCone;
      cones_id_list_.push_back(cones_found_);
      cone_id_to_color_map_[cones_found_] = newCone.type;

      utfr_msgs::msg::PoseGraphData cone_data;
      cone_data.id = cones_found_;
      cone_data.x = position_x_;
      cone_data.y = position_y_;
      cone_nodes_[cones_found_] = (cone_data);
      average_position_[cones_found_] = {position_x_, position_y_, 1};
      cone_id_to_vertex_map_[cones_found_] = cone_data;

      utfr_msgs::msg::PoseGraphData edge_data;
      edge_data.id = temp_current_pose_id_;
      edge_data.id2 = cones_found_;
      edge_data.dx = newCone.pos.x;
      edge_data.dy = newCone.pos.y;
      pose_to_cone_edges_.push_back(edge_data);
      kd_tree_knn::Point newPoint(position_x_, position_y_, cones_found_);

      detection_counts[cones_found_] = 1;
      globalKDTreePtr_->insert(newPoint);
      cones_found_ += 1;
      continue;
    }

    // Use KNN search to find the nearest cone
    kd_tree_knn::Point knnResult =
        globalKDTreePtr_->KNN(kd_tree_knn::Point(position_x_, position_y_, 0));

    const kd_tree_knn::Point &nearestCone = knnResult;

    // Check the result of the nearest neighbour search and calculate
    // displacement
    if (knnResult != kd_tree_knn::Point(0.0, 0.0, 0.0)) {

      // Use the nearest point
      double displacement = utfr_dv::util::euclidianDistance2D(
          position_x_, nearestCone.x, position_y_, nearestCone.y);

      // Do not add if its within 1 of an already seen cone
      if (displacement <= displacement_radius_) {
        // Add the ID to the list
        cones_id_list_.push_back(nearestCone.id);
        average_position_[nearestCone.id][0] += position_x_;
        average_position_[nearestCone.id][1] += position_y_;
        average_position_[nearestCone.id][2] += 1;

        cone_nodes_[nearestCone.id].x = average_position_[nearestCone.id][0] /
                                        average_position_[nearestCone.id][2];
        cone_nodes_[nearestCone.id].y = average_position_[nearestCone.id][1] /
                                        average_position_[nearestCone.id][2];

        utfr_msgs::msg::PoseGraphData edge_data;
        edge_data.id = temp_current_pose_id_;
        edge_data.id2 = nearestCone.id;
        edge_data.dx = newCone.pos.x;
        edge_data.dy = newCone.pos.y;
        pose_to_cone_edges_.push_back(edge_data);
        continue;
      }

      // Do not add if its within 0.5 of another cone deteced within the current
      // call of the function (rejecting duplicate detections)
      for (const auto &duplicates_potential : current_round_cones_) {
        comparative_displacement = utfr_dv::util::euclidianDistance2D(
            position_x_, std::get<0>(duplicates_potential), position_y_,
            std::get<1>(duplicates_potential));
        if (comparative_displacement <= 0.3) {
          is_duplicate_ = true;
          break;
        }
      }
      if (is_duplicate_) {
        continue;
      }
    }
    // Add to duplicate checker
    current_round_cones_.emplace_back(
        std::make_tuple(position_x_, position_y_));

    // Add to potential_cones (used for checking if cone can be added to
    // past_detections)
    potential_cones_.insert(std::make_pair(
        cones_potential_, std::make_tuple(position_x_, position_y_, colour)));

    std::vector<int> keys{};

    count_ = 0;
    // Check if same cone detected in three different time instances
    for (auto pointer = potential_cones_.begin();
         pointer != potential_cones_.end(); ++pointer) {
      std::pair<int, std::tuple<double, double, int>> pair = *pointer;
      int key_ = pair.first;
      const std::tuple<double, double, int> potentialPoint = pair.second;
      double temp_displacement_ = utfr_dv::util::euclidianDistance2D(
          position_x_, std::get<0>(potentialPoint), position_y_,
          std::get<1>(potentialPoint));

      // Check if cone within 1 of any other new detected cone
      if (temp_displacement_ <= displacement_radius_ && colour == std::get<2>(potentialPoint)) {
        count_ += 1;
        keys.push_back(key_);
        // Check if 30 of same detected
        true_coordinate_x += std::get<0>(potentialPoint);
        true_coordinate_y += std::get<1>(potentialPoint);
        //std::cout << "count: " << count_threshold_ << std::endl;
        if (count_ == count_threshold_) {

          double average_x = position_x_;
          double average_y = position_y_;
          double number_of_points = 1;
          for (auto it = potential_cones_.begin();
               it != potential_cones_.end();) {
            double temp_displacement_ = utfr_dv::util::euclidianDistance2D(
                average_x / number_of_points, std::get<0>(it->second),
                average_y / number_of_points, std::get<1>(it->second));
            if (temp_displacement_ <= displacement_radius_) {
              average_x += std::get<0>(it->second);
              average_y += std::get<1>(it->second);
              number_of_points += 1;
              it = potential_cones_.erase(it);
            } else {
              ++it;
            }
          }

          // Update cone_id_list_ and past_cone_detections_ and KD tree
          cones_id_list_.push_back(cones_found_);
          cone_id_to_color_map_[cones_found_] = newCone.type;
          utfr_msgs::msg::PoseGraphData cone_data;
          cone_data.id = cones_found_;
          cone_data.x = average_x / number_of_points;
          cone_data.y = average_y / number_of_points;
          cone_nodes_[cones_found_] = (cone_data);
          average_position_[cones_found_] = {position_x_, position_y_, 1};
          cone_id_to_vertex_map_[cones_found_] = cone_data;

          utfr_msgs::msg::PoseGraphData edge_data;
          edge_data.id = temp_current_pose_id_;
          edge_data.id2 = cones_found_;
          edge_data.dx = newCone.pos.x;
          edge_data.dy = newCone.pos.y;
          pose_to_cone_edges_.push_back(edge_data);
          kd_tree_knn::Point newPoint(average_x / number_of_points,
                                      average_y / number_of_points, cones_found_);
          detection_counts[cones_found_] = 3;
          // std::cout << "Cone x: " << position_x_ << " Cone y: " <<
          // position_y_ << " Cone id: " << cones_found_ << std::endl;
          past_cone_detections_[cones_found_] = newCone;
          globalKDTreePtr_->insert(newPoint);
          cones_found_ += 1;
          break;
        }
      }
    }

    cones_potential_ += 1;
    count_ = 0;
  }
  return cones_id_list_;
}

void BuildGraphNode::loopClosure(std::vector<int> const& current_frame_cones)
{
  if (loop_closed_)
    return;

  for (int current_frame_cone_id : current_frame_cones)
  {
    for (auto const& [past_cone_id, past_cone] : past_cone_detections_)
    {
      if (past_cone_id == current_frame_cone_id &&
          (past_cone_id == 0 || past_cone_id == 1) &&
          !landmarked_cone_id_.has_value())
      {
        // Set first large orange cone listed to be landmark cone
        landmarked_cone_id_ = current_frame_cone_id;
        first_detection_pose_id_ = current_pose_id_;
        out_of_frame_ = 0;
      }
    }

    // Cone has been landmarked and still in frame for the first time
    if (landmarked_cone_id_.has_value() && out_of_frame_ > -1 && out_of_frame_ < 1000) {
      auto seen_status = std::find(current_frame_cones.begin(), current_frame_cones.end(), *landmarked_cone_id_);
      // If cone not in frame anymore
      if (seen_status == current_frame_cones.end()) {
        out_of_frame_ += 1;
      }
      else {
        out_of_frame_ = 0;
      }
    }
    
    // Cone has been landmarked and is no longer in frame
    if (landmarked_cone_id_.has_value() && out_of_frame_ >= 1000) {
      // Check if cone appears in frame again
      auto seen_status = std::find(current_frame_cones.begin(), current_frame_cones.end(), *landmarked_cone_id_);
      // If cone in frame again
      if (seen_status != current_frame_cones.end()) {
        // Car has returned back to landmark cone position, made full loop

        double dx =
            id_to_ego_map_[current_pose_id_].pose.pose.position.x -
            id_to_ego_map_[first_detection_pose_id_].pose.pose.position.x;
        double dy =
            id_to_ego_map_[current_pose_id_].pose.pose.position.y -
            id_to_ego_map_[first_detection_pose_id_].pose.pose.position.y;
        double dtheta =
            utfr_dv::util::quaternionToYaw(
                id_to_ego_map_[current_pose_id_].pose.pose.orientation) -
            utfr_dv::util::quaternionToYaw(
                id_to_ego_map_[first_detection_pose_id_]
                    .pose.pose.orientation);

        utfr_msgs::msg::PoseGraphData edge_data;
        edge_data.id = first_detection_pose_id_;
        edge_data.id2 = current_pose_id_;
        edge_data.dx = dx;
        edge_data.dy = dy;
        edge_data.dtheta = dtheta;
        edge_data.loop_closure = true;
        pose_to_pose_edges_.push_back(edge_data);

        landmarked_cone_id_ = std::nullopt;
        out_of_frame_ = -1;
        first_detection_pose_id_ = 0;
        loop_closed_ = true;
        do_graph_slam_ = true;
        closed_loop_once.data = true;
      }
    }
  }
}

void BuildGraphNode::timerCB() {

  // Print the x and y position of the car
  current_state_ = current_ego_state_;

  visualization_msgs::msg::MarkerArray markerArray;

  int count = 0;

  for (utfr_msgs::msg::Cone cone : current_cone_detections_.left_cones) {
    count += 1;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "os_sensor";
    marker.header.stamp = this->get_clock()->now();
    marker.id = count;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = cone.pos.x;
    marker.pose.position.y = cone.pos.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    markerArray.markers.push_back(marker);
  }

  for (utfr_msgs::msg::Cone cone : current_cone_detections_.right_cones) {
    count += 1;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "os_sensor";
    marker.header.stamp = this->get_clock()->now();
    marker.id = count;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = cone.pos.x;
    marker.pose.position.y = cone.pos.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    markerArray.markers.push_back(marker);
  }

  for (int i = count+1; i < 15; i++) {
    // Delete the marker if it's not being used
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "os_sensor";
    marker.header.stamp = this->get_clock()->now();
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    markerArray.markers.push_back(marker);
  }

  // cone_viz_publisher_->publish(markerArray);

  if (current_pose_id_ == 1000) {
    current_state_.pose.pose.position.x = 0;
    current_state_.pose.pose.position.y = 0;
    current_state_.pose.pose.orientation = utfr_dv::util::yawToQuaternion(0);
  }

  current_pose_id_ += 1;
  id_to_ego_map_[current_pose_id_] = current_state_;

  utfr_msgs::msg::PoseGraphData pose_data;
  pose_data.id = current_pose_id_;
  pose_data.x = current_state_.pose.pose.position.x;
  pose_data.y = current_state_.pose.pose.position.y;
  pose_data.theta =
      utfr_dv::util::quaternionToYaw(current_state_.pose.pose.orientation);
  pose_nodes_[current_pose_id_] = (pose_data);
  id_to_pose_map_[current_pose_id_] = pose_data;

  if (current_pose_id_ == 1001) {
    return;
  }

  utfr_msgs::msg::EgoState prevPoseVertex =
      id_to_ego_map_[current_pose_id_ - 1];

  double dx =
      current_state_.pose.pose.position.x - prevPoseVertex.pose.pose.position.x;
  double dy =
      current_state_.pose.pose.position.y - prevPoseVertex.pose.pose.position.y;
  double dtheta =
      utfr_dv::util::quaternionToYaw(current_state_.pose.pose.orientation) -
      utfr_dv::util::quaternionToYaw(prevPoseVertex.pose.pose.orientation);

  utfr_msgs::msg::PoseGraphData edge_data;
  edge_data.id = current_pose_id_ - 1;
  edge_data.id2 = current_pose_id_;
  edge_data.dx = dx;
  edge_data.dy = dy;
  edge_data.dtheta = dtheta;
  edge_data.loop_closure = false;
  pose_to_pose_edges_.push_back(edge_data);

  std::vector<int> cones = KNN(current_cone_detections_);
  loopClosure(cones);

  std::vector<utfr_msgs::msg::PoseGraphData> cones_;
  std::vector<utfr_msgs::msg::PoseGraphData> states_;

  // Add all values from the map to the vector
  for (const auto& pair : cone_nodes_) {
      cones_.push_back(pair.second);
  }

  for (const auto& pair : pose_nodes_) {
      states_.push_back(pair.second);
  }

  // Make pose_to_cone_edges_ have a max size of 4501
  if (pose_to_cone_edges_.size() > 4500) {
    // Delete all the edges before 4501
    pose_to_cone_edges_.erase(pose_to_cone_edges_.begin(),
                              pose_to_cone_edges_.begin() + 4500);
  }

  if (cones_found_ > 3) {
    utfr_msgs::msg::PoseGraph poseGraph;
    poseGraph.header.stamp = this->get_clock()->now();

    poseGraph.cones = cones_;
    poseGraph.states = states_;
    poseGraph.motion_edge = pose_to_pose_edges_;
    poseGraph.measurement_edges = pose_to_cone_edges_;

    for (const auto &pair : cone_id_to_color_map_) {
      poseGraph.cone_id.push_back(pair.first);
      poseGraph.color.push_back(pair.second);
    }

    poseGraph.run_slam = do_graph_slam_;
    pose_graph_publisher_->publish(poseGraph);
    loop_closure_publisher_->publish(closed_loop_once);
    

    if (do_graph_slam_) {
      do_graph_slam_ = false;
    }
  }
  // cone_map_publisher_->publish(cone_map_);
  // Save the optimized pose graph
  // std::cout << "Optimized pose graph saved" << std::endl;
}

} // namespace build_graph
} // namespace utfr_dv