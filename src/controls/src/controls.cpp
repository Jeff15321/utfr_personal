/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: controls.cpp
* auth: Youssef Elhadad
* desc: controllers main executable for ros2 sub
*/
#include <controls_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::controls::ControlsNode>());
  rclcpp::shutdown();
  return 0;
}
