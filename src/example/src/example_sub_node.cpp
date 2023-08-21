/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: example_sub_node.cpp
* auth: Kelvin Cui
* desc: example subscriber node class for ros2 template
*/

#include <example_sub_node.hpp>

namespace utfr_dv {
namespace example {

ExampleSubNode::ExampleSubNode() : Node("example_sub_node") {
  this->loadParams();
  this->initSubscribers();
}

void ExampleSubNode::loadParams() {
  // Initialize Params with default values
  this->declare_parameter("echo_string", "");

  // Load params from parameter server
  echo_string_ = this->get_parameter("echo_string")
                     .get_parameter_value()
                     .get<std::string>();
}

void ExampleSubNode::initSubscribers() {
  example_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      topics::kExample, 10, std::bind(&ExampleSubNode::exampleCB, this, _1));
}

void ExampleSubNode::exampleCB(const std_msgs::msg::String &msg) {
  std::string print_statement = echo_string_ + msg.data;
  RCLCPP_INFO(this->get_logger(), "%s", print_statement.c_str());
}

} // namespace example
} // namespace utfr_dv