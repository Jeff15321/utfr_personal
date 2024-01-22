/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: ekf_node.cpp
* auth: Arthur Xu
* desc: ekf node class
*/

#include <ekf_node.hpp>

namespace utfr_dv {
namespace ekf {

enum class HeartBeatState{ NOT_READY, READY, ACTIVE, ERROR, FINISH };

EkfNode::EkfNode() : Node("ekf_node") {

  // set heartbeat state to not ready
  HeartBeatState heartbeat_state_ = HeartBeatState::NOT_READY;

  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
  
  // set heartbeat state to active
  heartbeat_state_ = HeartBeatState::ACTIVE;
  
  this->publishHeartbeat();
}

void EkfNode::initParams() {
   heartbeat_rate_ = 1;
}

void EkfNode::initSubscribers() {

  sensorcan_subscriber_ = this->create_subscription<utfr_msgs::msg::SensorCan>(
      topics::kSensorCan, 1,
      std::bind(&EkfNode::sensorCB, this, std::placeholders::_1));
}

void EkfNode::initPublishers() {
  state_estimation_publisher_ =
      this->create_publisher<utfr_msgs::msg::EgoState>(topics::kEgoState, 10);
}

void EkfNode::initTimers() {
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(heartbeat_rate_),
        std::bind(&EkfNode::publishHeartbeat, this));
}


void EkfNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kEKFHeartbeat, 10);
}

void EkfNode::publishHeartbeat() {
    utfr_msgs::msg::Heartbeat heartbeat_msg;
    heartbeat_msg.status = static_cast<uint8_t>(heartbeat_state_);  
    heartbeat_msg.header.stamp = this->now();  

    heartbeat_publisher_->publish(heartbeat_msg);
}

void EkfNode::sensorCB(const utfr_msgs::msg::SensorCan msg) {}

void EkfNode::vehicleModel(const float &throttle, const float &brake,
                           const float &steering_angle) {}

void EkfNode::EKF() {}

} // namespace ekf
} // namespace utfr_dv
