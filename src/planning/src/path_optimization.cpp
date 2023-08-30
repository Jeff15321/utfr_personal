#include <rclcpp/rclcpp.hpp>
#include <path_optimization_node.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<utfr_dv::path_optimization::PathOptimizationNode>());
  rclcpp::shutdown();
  return 0;
}