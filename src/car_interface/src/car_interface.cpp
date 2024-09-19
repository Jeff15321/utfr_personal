/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: car_interface.cpp
* auth: Youssef Elhadad
* desc: car_interface main executable
*/
#include <car_interface_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::car_interface::CarInterface>());
  std::system("pkill -f \"ros\"");
  rclcpp::shutdown();
  return 0;
}
