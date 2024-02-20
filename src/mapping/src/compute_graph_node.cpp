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

#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/command_args.h"
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

void ComputeGraphNode::initParams() {
  this->declare_parameter("slam_timer_", 100.0);

  slam_rate_ =
      this->get_parameter("slam_timer_").get_parameter_value().get<double>();
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
  slam_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(slam_rate_),
      std::bind(&ComputeGraphNode::graphSLAM, this));
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

void ComputeGraphNode::poseGraphCB(const utfr_msgs::msg::PoseGraph msg) {}

void ComputeGraphNode::graphSLAM() {}

} // namespace compute_graph
} // namespace utfr_dv
