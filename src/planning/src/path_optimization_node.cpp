/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: center_path_node.cpp
* auth: Justin Lim
* desc: path optimization node class
*/

#include <path_optimization_node.hpp>

namespace utfr_dv {
namespace path_optimization {

PathOptimizationNode::PathOptimizationNode() : Node("path_optimization_node") {
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

void PathOptimizationNode::initParams() {}

void PathOptimizationNode::initSubscribers() {}

void PathOptimizationNode::initPublishers() {}

void PathOptimizationNode::initTimers() {}

void PathOptimizationNode::initHeartbeat() {}

} // namespace path_optimization
} // namespace utfr_dv

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<utfr_dv::path_optimization::PathOptimizationNode>());
  rclcpp::shutdown();
  return 0;
}
