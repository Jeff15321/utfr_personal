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
  this->initHeartbeat();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::NOT_READY);
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
}

void ControlsNode::initParams() {
  std::vector<double> default_pid = {0.0, 0.0, 0.0, 0.0, 0.0};
  // throttle_controller_params
  // this->declare_parameter("throttle_controller_params", default_pid);
  // thr_ctrl_params_ =
  //     this->get_parameter("throttle_controller_params").as_double_array();

  // braking_controller_params
  this->declare_parameter("braking_controller_params", default_pid);
  brk_ctrl_params_ =
      this->get_parameter("braking_controller_params").as_double_array();

  // update_rate_
  this->declare_parameter("update_rate", 33.33);
  update_rate_ = this->get_parameter("update_rate").as_double();

  // testing_
  this->declare_parameter("testing", 0);
  testing_ = this->get_parameter("testing").as_int();
  ros_time_ = this->now();
  brake_inc_ = 0;
  steer_inc_ = 0;
  throttle_inc_ = 0;
  setpoints_ = {-30, 0, 30};
}

void ControlsNode::initPublishers() {
  control_cmd_publisher_ = this->create_publisher<utfr_msgs::msg::ControlCmd>(
      topics::kControlCmd, 1);
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
  // steering_pid_ = std::make_unique<PIDController>();
  // throttle_pid_ = std::make_unique<PIDController>();
  braking_pid_ = std::make_unique<PIDController>();

  // steering_pid_->initController(str_ctrl_params_, "steering controller");
  // throttle_pid_->initController(thr_ctrl_params_, "throttle controller");
  braking_pid_->initController(brk_ctrl_params_, "braking controller");
}

void ControlsNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kControlsHeartbeat, 10);
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

bool max_vel = false;
float current_velocity = 0.0;
float target_velocity = 0.0;
void ControlsNode::brakeTesting() {

  // If targets are available get them
  if (ego_state_ == nullptr || target_state_ == nullptr) {
    control_cmd_.brk_cmd = 255 - (brake_inc_ / 10) * 15;
    brake_inc_++;
    if (brake_inc_ >= 179) {
      brake_inc_ = 0;
    }
  } else { // TODO: review
    if (ego_state_ != nullptr) {
      current_velocity = ego_state_->vel.twist.linear.x;

      target_velocity = 5.0;

      if (!max_vel && (ego_state_->vel.twist.linear.x > target_velocity)) {
        max_vel = true;
      }

      if (max_vel) {
        target_velocity = 0.0;
        if (ego_state_->vel.twist.linear.x == target_velocity) {
          max_vel = false;
        }
      }

      RCLCPP_INFO(this->get_logger(), "Current speed: %fm/s", current_velocity);
      RCLCPP_INFO(this->get_logger(), "Target speed: %fm/s", target_velocity);

      control_cmd_.brk_cmd = (target_velocity > current_velocity) ? 0 : 255;
    }
  }

  RCLCPP_INFO(this->get_logger(), "PWM: %f", control_cmd_.brk_cmd);
}

void ControlsNode::steerTesting() {
  steer_inc_++;
  if (steer_inc_ >= 20) {
    steer_inc_ = -20;
  } else if (steer_inc_ >= 0) {
    control_cmd_.str_cmd = 85;
  } else if (steer_inc_ < 0) {
    control_cmd_.str_cmd = -85;
  }

  RCLCPP_INFO(this->get_logger(), "STR Cmd: %d", control_cmd_.str_cmd);

  control_cmd_.thr_cmd = 0.2;
}

void ControlsNode::throttleTesting() {
  control_cmd_.thr_cmd = (throttle_inc_)*5;
  throttle_inc_++;
  if (throttle_inc_ >= 21) {
    throttle_inc_ = 0;
  }
}

void ControlsNode::timerCB() {
  // Check if testing is in place
  if (testing_) {
    // Init all params to zero
    control_cmd_.brk_cmd = 0;
    control_cmd_.thr_cmd = 0;
    control_cmd_.str_cmd = 0;

    if (testing_ & 0x1) {
      this->brakeTesting();
    }
    if (testing_ & 0x2) {
      this->steerTesting();
    }
    if (testing_ & 0x4) {
      this->throttleTesting();
    }

    control_cmd_.header.stamp = this->get_clock()->now();

    // Publish messages
    control_cmd_publisher_->publish(control_cmd_);
    this->publishHeartbeat(utfr_msgs::msg::Heartbeat::NOT_READY);
    return;
  }

  // Check if other nodes are sending correct messages
  if (target_state_ == nullptr) {
    if (target_state_)
      RCLCPP_INFO(this->get_logger(), "No EGO states");
    if (ego_state_)
      RCLCPP_INFO(this->get_logger(), "No Target states");
    this->publishHeartbeat(utfr_msgs::msg::Heartbeat::NOT_READY);
    return;
  }

  try {
    double dt = (this->now() - ros_time_).seconds();
    ros_time_ = this->now();

    // control_cmd_.str_cmd =
    //     std::clamp((int)utfr_dv::util::radToDeg(target_state_->steering_angle),
    //                MAX_STR, -MAX_STR);
    control_cmd_.str_cmd = (int)(target_state_->steering_angle * 180 / 3.1415);
    RCLCPP_WARN(this->get_logger(), "Controls converted str cmd: %d",
                control_cmd_.str_cmd);

    // //*****   Throttle & Brake  *****
    // current_velocity = ego_state_->vel.twist.linear.x; // TODO: review
    // target_velocity = target_state_->speed;            // TODO: review

    // if (current_velocity < target_velocity) {
    //   // RCLCPP_INFO(this->get_logger(), "Accelerating to reach: %fm/s",
    //   // target_velocity);
    //   // control_cmd_.thr_cmd =
    //   //     throttle_pid_->getCommand(target_velocity, current_velocity,
    //   dt);
    //   // TODO: add RPM cap
    //   control_cmd_.thr_cmd =
    //       target_velocity * 60 / (2 * M_PI * WHEEL_RADIUS) * GEAR_RATIO;
    //   control_cmd_.brk_cmd = 0;
    // } else {
    //   // RCLCPP_INFO(this->get_logger(), "Braking to reach: %fm/s",
    //   // target_velocity);
    //   control_cmd_.thr_cmd = 0;
    //   control_cmd_.brk_cmd =
    //       braking_pid_->getCommand(target_velocity, current_velocity, dt);
    // }

    control_cmd_.thr_cmd = 0;
    control_cmd_.brk_cmd = 0;

    control_cmd_.header.stamp = this->get_clock()->now();

    // Publish message
    control_cmd_publisher_->publish(control_cmd_);

    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

} // namespace controls
} // namespace utfr_dv
