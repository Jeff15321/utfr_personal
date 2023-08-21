/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: example_pub_node.cpp
* auth: Kelvin Cui
* desc: example node class for ros2 template
*/

#include <example_pub_node.hpp>

namespace utfr_dv {
namespace example {

ExamplePubNode::ExamplePubNode() : Node("example_pub_node") {
  this->initParams();
  this->initPublishers();
  this->initTimers();
}

void ExamplePubNode::initParams() {
  // Initialize Params with default values
  this->declare_parameter("publish_rate", 1000.0);
  this->declare_parameter("publish_string", "Default Message");

  // Load params from parameter server
  publish_rate_ =
      this->get_parameter("publish_rate").get_parameter_value().get<double>();

  publish_string_ = this->get_parameter("publish_string")
                        .get_parameter_value()
                        .get<std::string>();
}

void ExamplePubNode::initPublishers() {
  example_publisher_ =
      this->create_publisher<std_msgs::msg::String>(topics::kExample, 10);
}

void ExamplePubNode::initTimers() {
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(publish_rate_),
      std::bind(&ExamplePubNode::timerCB, this));
}

void ExamplePubNode::timerCB() {
  auto curr_message = std_msgs::msg::String();
  curr_message.data = publish_string_ + std::to_string(counter_);
  example_publisher_->publish(curr_message);

  RCLCPP_INFO(this->get_logger(), "Published Message:  %s!",
              curr_message.data.c_str());

  auto counter_factorial = util::factorial(counter_);

  RCLCPP_INFO(this->get_logger(), "Counter Factorial::  %s!",
              std::to_string(counter_factorial).c_str());

  counter_++;
}

} // namespace example
} // namespace utfr_dv