/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: ekf_node.cpp
* auth: Justin Lim
* desc: ekf node class
*/

#include <ekf_node.hpp>

namespace utfr_dv {
namespace ekf {

EkfNode::EkfNode() : Node("ekf_node") {
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

void EkfNode::initParams() {}

void EkfNode::initSubscribers() {

  sensorcan_subscriber_ =     
      this->create_subscription<utfr_msgs::msg::SensorCan>(
          topics::kSensorCan, 1, 
          std::bind(&EkfNode::sensorCB, this, 
              std::placeholders::_1));

}

void EkfNode::initPublishers() {
  state_estimation_publisher_ = 
    this->create_publisher<utfr_msgs::msg::EgoState>(topics::kEgoState, 10);
}

void EkfNode::initTimers() {}

void EkfNode::initHeartbeat() {
  heartbeat_publisher_ = 
    this->create_publisher<utfr_msgs::msg::Heartbeat>(topics::kEKFHeartbeat, 10);
}

void EkfNode::sensorCB(const utfr_msgs::msg::SensorCan msg) {};

void EkfNode::vehicleModel(const float& throttle, 
                          const float& brake, 
                          const float& steering_angle) {};

void EkfNode::EKF() {};

} // namespace ekf
} // namespace utfr_dv

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::ekf::EkfNode>());
  rclcpp::shutdown();
  return 0;
}
