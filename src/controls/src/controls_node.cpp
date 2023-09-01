/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: controls_node.cpp
* auth: Youssef Elhadad
* desc: controls node class
*/
#include <controls_node.hpp>

namespace utfr_dv {
namespace controls {

ControlsNode::ControlsNode() : Node("controls_node") {
  this->initParams();
  this->initPublishers();
  this->initController();
  this->initSubscribers();
  this->initTimers();
  this->initHeartbeat();
}

void ControlsNode::initParams() {
  // steering_controller_params
  std::vector<double> default_pid;
  this->declare_parameter("steering_controller_params", default_pid);
  str_ctrl_params_ =
      this->get_parameter("steering_controller_params").as_double_array();

  // throttle_controller_params
  this->declare_parameter("throttle_controller_params", default_pid);
  thr_ctrl_params_ =
      this->get_parameter("throttle_controller_params").as_double_array();

  // braking_controller_params
  this->declare_parameter("braking_controller_params", default_pid);
  brk_ctrl_params_ =
      this->get_parameter("braking_controller_params").as_double_array();

  // update_rate_
  this->declare_parameter("update_rate", 1000.0);
  update_rate_ = this->get_parameter("update_rate").as_double();

  ros_time_ = this->now();
}

void ControlsNode::initPublishers() {
  control_cmd_publisher_ = this->create_publisher<utfr_msgs::msg::ControlCmd>(
      topics::kControlCmd, 1);
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kControlsHeartbeat, 1);
}

void ControlsNode::initSubscribers() {
  target_state_subscriber_ =
      this->create_subscription<utfr_msgs::msg::TargetState>(
          topics::kTargetState, 1,
          std::bind(&ControlsNode::targetStateCB, this, _1));

  ego_state_subscriber_ = this->create_subscription<utfr_msgs::msg::EgoState>(
      topics::kEgoState, 1, std::bind(&ControlsNode::egoStateCB, this, _1));

  sensor_can_subscriber_ = this->create_subscription<utfr_msgs::msg::SensorCan>(
      topics::kSensorCan, 1, std::bind(&ControlsNode::sensorCanCB, this, _1));
}

void ControlsNode::initTimers() {
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(this->update_rate_),
      std::bind(&ControlsNode::timerCB, this));
}

void ControlsNode::initController() {
  steering_pid_ = std::make_unique<PIDController>();
  throttle_pid_ = std::make_unique<PIDController>();
  braking_pid_ = std::make_unique<PIDController>();

  steering_pid_->initController(str_ctrl_params_, "steering controller");
  throttle_pid_->initController(thr_ctrl_params_, "throttle controller");
  braking_pid_->initController(brk_ctrl_params_, "braking controller");
}

void ControlsNode::initHeartbeat() {
  heartbeat_.module.data = "controls";
  heartbeat_.update_rate = update_rate_;
}

void ControlsNode::publishHeartbeat(const int status) {
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_publisher_->publish(heartbeat_);
}

void ControlsNode::targetStateCB(const utfr_msgs::msg::TargetState &msg) {
  target_state_ = std::make_shared<utfr_msgs::msg::TargetState>(msg);
}

void ControlsNode::egoStateCB(const utfr_msgs::msg::EgoState &msg) {
  ego_state_ = std::make_shared<utfr_msgs::msg::EgoState>(msg);
}

void ControlsNode::sensorCanCB(const utfr_msgs::msg::SensorCan &msg) {
  sensor_can_ = msg;
}

void ControlsNode::timerCB() {
  int status = utfr_msgs::msg::Heartbeat::ACTIVE;

  // Check if other nodes are sending correct messages
  if (ego_state_ == nullptr || target_state_ == nullptr) {
    status = utfr_msgs::msg::Heartbeat::NOT_READY;
    this->publishHeartbeat(status);
  }

  try {
    double target_velocity;
    double current_velocity;
    int16_t target_sa;
    int16_t current_sa;

    double dt = (this->now() - ros_time_).seconds();
    ros_time_ = this->now();

    //*****   Steering   *****
    current_sa = (ego_state_->steering_angle) / 4.5; // TODO: review
    target_sa = target_state_->steering_angle;       // TODO: review

    control_cmd_.str_cmd =
        (int16_t)steering_pid_->getCommand(target_sa, current_sa, dt);

    //*****   Throttle & Brake  *****
    current_velocity = ego_state_->vel.twist.linear.x; // TODO: review
    target_velocity = target_state_->speed;            // TODO: review
    RCLCPP_INFO(this->get_logger(), "Current speed: %f", current_velocity);

    control_cmd_.thr_cmd =
        throttle_pid_->getCommand(target_velocity, current_velocity, dt);

    control_cmd_.brk_cmd = (uint8_t)braking_pid_->getCommand(
        target_velocity, current_velocity, dt);

    if (current_velocity < target_velocity) {
      RCLCPP_INFO(this->get_logger(), "Accelerating to: %f", target_velocity);
      control_cmd_.brk_cmd = 0;
    } else {
      RCLCPP_INFO(this->get_logger(), "Braking to: %f", target_velocity);
      control_cmd_.thr_cmd = 0;
    }

    control_cmd_.header.stamp = this->get_clock()->now();

    // Publish message
    control_cmd_publisher_->publish(control_cmd_);

  } catch (int e) {
    RCLCPP_INFO(this->get_logger(), "timerCB: Error occured, error #%d", e);
    status = utfr_msgs::msg::Heartbeat::ERROR;
  }

  this->publishHeartbeat(status);
}

} // namespace controls
} // namespace utfr_dv
