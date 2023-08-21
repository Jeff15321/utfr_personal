/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: example_sub.cpp
* auth: Kelvin Cui
* desc: example main executable for ros2 template
*/
#include <example_sub_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::example::ExampleSubNode>());
  rclcpp::shutdown();
  return 0;
}
