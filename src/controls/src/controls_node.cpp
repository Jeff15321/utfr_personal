/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: controls_node.cpp
* auth: Youssef Elhadad
* desc: controller publisher node class for ros2 template
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
  // wheelbase
  // Initialize Params with default values
  this->declare_parameter("wheelbase", 1.58);

  // Load params from parameter server
  wheelbase_ = this->get_parameter("wheelbase").as_double();

  // lookahead_distance_scaling_factor
  this->declare_parameter("lookahead_distance_scalling_factor", 0.5);

  lookahead_distance_ =
      this->get_parameter("lookahead_distance_scalling_factor").as_double();

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

  // hil_
  this->declare_parameter("hil", 0);

  hil_ = this->get_parameter("hil").as_int();

  // target_steer_test_
  this->declare_parameter("target_steer_test", 0);

  target_steer_test_ = this->get_parameter("target_steer_test").as_int();

  // test_
  this->declare_parameter("test", 1);

  test_ = this->get_parameter("test").as_int();

  ros_time_ = this->now();
}

void ControlsNode::initPublishers() {
  control_cmd_publisher_ = this->create_publisher<utfr_msgs::msg::ControlCmd>(
      topics::kControlCmd, 1);
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kControlSystemsHeartbeat, 1);
}

void ControlsNode::initSubscribers() {
  target_state_subscriber_ =
      this->create_subscription<utfr_msgs::msg::TargetState>(
          topics::kTargetState, 1,
          std::bind(&ControlsNode::targetStateCB, this, _1));

  ego_state_subscriber_ = this->create_subscription<utfr_msgs::msg::EgoState>(
      topics::kEgoState, 1, std::bind(&ControlsNode::egoStateCB, this, _1));

  jetson_subscriber_ = this->create_subscription<utfr_msgs::msg::Jetson>(
      topics::kJetson, 1, std::bind(&ControlsNode::jetsonCB, this, _1));
}

void ControlsNode::initTimers() {
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(this->update_rate_),
      std::bind(&ControlsNode::timerCB, this));
}

void ControlsNode::initController() {
  pure_pursuit_ = std::make_unique<PurePursuitController>();
  steering_pid_ = std::make_unique<PIDController>();
  throttle_pid_ = std::make_unique<PIDController>();
  braking_pid_ = std::make_unique<PIDController>();
  pure_pursuit_->initController(wheelbase_, lookahead_distance_);

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

void ControlsNode::timerCB() {
  int status = utfr_msgs::msg::Heartbeat::ACTIVE;

  // Check if other nodes are sending correct messages
  if (!test_ && (ego_state == nullptr || target_state_ == nullptr)) {

    status = utfr_msgs::msg::Heartbeat::UNINITIALIZED;
    this->publishHeartbeat(status);
    if (!test_)
      return;
  }

  try {
    double target_velocity;
    double ego_velocity;
    int16_t target_angle;
    int16_t current_steering_angle_;

    double dt = (this->now() - ros_time_).seconds();
    ros_time_ = this->now();

    // get target states
    if (test_) {
      target_angle = target_steer_test_;
      target_velocity = 1.0;
      ego_velocity = jetson_msg_.velocity;
    } else if (target_state_->type == utfr_msgs::msg::TargetState::GLOBAL) {
      target_angle =
          RadToDeg(pure_pursuit_->getSteeringAngle(target_state_, ego_state));
      target_velocity = target_state_->trajectory.velocity;
    } else if (target_state_->type == utfr_msgs::msg::TargetState::ROLLOUT) {
      target_angle = -RadToDeg(target_state_->steering_angle);
      RCLCPP_INFO(this->get_logger(), "Target angle: %d", target_angle);
      target_velocity = target_state_->velocity;
    }

    //*****   Steering   *****
    current_steering_angle_ = (jetson_msg_.steering_angle) / 4.5;
    RCLCPP_INFO(this->get_logger(), "Current Steering angle: %d",
                current_steering_angle_);

    control_cmd_.str_cmd = (int16_t)steering_pid_->getCommand(
        target_angle, current_steering_angle_, dt);

    RCLCPP_INFO(this->get_logger(), "Sending str_cmd: %d",
                control_cmd_.str_cmd);

    //*****   Throttle & Brakes  *****
    ego_velocity = utfr_dv::util::vectorMagnitude(ego_state->vel);
    RCLCPP_INFO(this->get_logger(), "Current speed: %f", ego_velocity);

    control_cmd_.thr_cmd =
        throttle_pid_->getCommand(target_velocity, ego_velocity, dt);

    control_cmd_.brk_cmd =
        (uint8_t)braking_pid_->getCommand(target_velocity, ego_velocity, dt);

    if (ego_velocity < target_velocity) {
      RCLCPP_INFO(this->get_logger(), "Accelerating to: %f", target_velocity);
      control_cmd_.brk_cmd = 0;
    } else {
      RCLCPP_INFO(this->get_logger(), "Braking to: %f", target_velocity);
      control_cmd_.thr_cmd = 0;
    }

    //*****   End of Event   *****
    if (target_state_->trajectory.type ==
        utfr_msgs::msg::TrajectoryPoint::FINISH) {
      control_cmd_.thr_cmd = 0;
      control_cmd_.str_cmd = 0;
      control_cmd_.brk_cmd = 0;

      if (ego_velocity != 0.0) {
        control_cmd_.brk_cmd = 255;
      }
    }

    // Log params to monitor
    control_cmd_.str_angle = current_steering_angle_;
    control_cmd_.str_tar = target_angle;
    control_cmd_.header.stamp = this->get_clock()->now();

    // Publish message
    control_cmd_publisher_->publish(control_cmd_);

  } catch (int e) {
    RCLCPP_INFO(this->get_logger(), "timerCB: Error occured, error #%d", e);
    status = utfr_msgs::msg::Heartbeat::FATAL;
  }

  this->publishHeartbeat(status);
}

void ControlsNode::targetStateCB(const utfr_msgs::msg::TargetState &msg) {
  target_state_ = std::make_shared<utfr_msgs::msg::TargetState>(msg);
}

void ControlsNode::egoStateCB(const utfr_msgs::msg::EgoState &msg) {
  ego_state = std::make_shared<utfr_msgs::msg::EgoState>(msg);
}

void ControlsNode::jetsonCB(const utfr_msgs::msg::Jetson &msg) {
  jetson_msg_ = msg;
}

} // namespace controls
} // namespace utfr_dv
