/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: controller_node.cpp
* auth: Justin Lim
* desc: controller node class
*/

#include <controller_node.hpp>

namespace utfr_dv {
namespace controller {

ControllerNode::ControllerNode() : Node("controller_node") {
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

void ControllerNode::initParams() {}

void ControllerNode::initSubscribers() {}

void ControllerNode::initPublishers() {}

void ControllerNode::initTimers() {}

void ControllerNode::initHeartbeat() {}

} // namespace controller
} // namespace utfr_dv

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::controller::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
