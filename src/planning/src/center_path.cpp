#include <rclcpp/rclcpp.hpp>
#include <center_path_node.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::center_path::CenterPathNode>());
  rclcpp::shutdown();
  return 0;
}
