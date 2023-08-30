#include <rclcpp/rclcpp.hpp>
#include <controller_node.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::controller::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}