/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: center_path_node.cpp
* auth: Justin Lim
* desc: center path node class
*/

#include <center_path_node.hpp>

namespace utfr_dv {
namespace center_path {

CenterPathNode::CenterPathNode() : Node("center_path_node") {
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

void CenterPathNode::initParams() {}

void CenterPathNode::initSubscribers() {}

void CenterPathNode::initPublishers() {}

void CenterPathNode::initTimers() {}

void CenterPathNode::initHeartbeat() {}

} // namespace center_path
} // namespace utfr_dv

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::center_path::CenterPathNode>());
  rclcpp::shutdown();
  return 0;
}
