/*
******************************************************
|                                                    |
|           _     __                    _            |
|    _   _ | |_  / _| _ __            _| |__   __    |
|   | | | || __|| |_ | '__|         / _` |\ \ / /    |
|   | |_| || |_ |  _|| |           | (_| | \ V /     |
|    \__,_| \__||_|  |_|    _____   \__,_|  \_/      |
|                          |_____|                   |
|                                                    |
|                                                    |
******************************************************
*
* file: jetson_interface.cpp
* auth: Youssef Elhadad
* desc: jetson_interface main executable
*/
#include <rclcpp/rclcpp.hpp>
#include <jetson_interface_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::jetson_interface::JetsonInterface>());
  rclcpp::shutdown();
  return 0;
}

