/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: jetson_interface_node.cpp
* auth: Youssef Elhadad
* desc: jetson_interface node class
*/

#include <jetson_interface_node.hpp>

namespace utfr_dv {
namespace jetson_interface {

JetsonInterface::JetsonInterface() : Node("jetson_interface_node") {
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
  this->initCAN();
  this->initSensors();
}

void JetsonInterface::initParams() {
  // Initialize Params with default values
  this->declare_parameter("update_rate", 33.33);
  this->declare_parameter("test_brake", 1.0);

  update_rate_ = this->get_parameter("update_rate").as_double();

  steering_rate_cmd_ = 0;
  braking_cmd_ = 0;
  throttle_cmd_ = 0;

  motor_speed_ = 0.0;
  imu_accel_.x = 0.0;
  imu_accel_.y = 0.0;
  imu_accel_.z = 0.0;
  roll_rate_ = 0.0;
  yaw_rate_ = 0.0;

  current_steering_angle_ = 0;
}

void JetsonInterface::initSubscribers() {
  control_cmd_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ControlCmd>(
          topics::kControlCmd, 10,
          std::bind(&JetsonInterface::controlCmdCB, this, _1));
  system_status_subscriber_ =
      this->create_subscription<utfr_msgs::msg::SystemStatus>(
          topics::kSystemStatus, 10,
          std::bind(&JetsonInterface::systemStatusCB, this, _1));

  RCLCPP_INFO(this->get_logger(), "Finished Initializing Subscribers");
}

void JetsonInterface::initPublishers() {
  jetson_publisher_ =
      this->create_publisher<utfr_msgs::msg::SensorCan>(topics::kJetson, 10);
  system_status_publisher_ =
      this->create_publisher<utfr_msgs::msg::SystemStatus>(
          topics::kSystemStatus, 1);
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kControlSystemsHeartbeat, 1);

  RCLCPP_INFO(this->get_logger(), "Finished Initializing Publishers");
}

void JetsonInterface::initHeartbeat() {
  heartbeat_.module.data = "jetson_interface";
  heartbeat_.update_rate = update_rate_;
}

void JetsonInterface::publishHeartbeat(const int status) {
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_publisher_->publish(heartbeat_);
}

void JetsonInterface::initTimers() {
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(this->update_rate_),
      std::bind(&JetsonInterface::timerCB, this));

  RCLCPP_INFO(this->get_logger(), "Finished Initializing Timer");
}

void JetsonInterface::initCAN() {
  can1_ = std::make_unique<CanInterface>();

  if (can1_->connect("can1")) {
    RCLCPP_INFO(this->get_logger(), "Finished Initializing CAN");
  } else
    RCLCPP_INFO(this->get_logger(), "Failed To Initialize CAN");

  while (can1_->read_can())
    ;

  can0_ = std::make_unique<CanInterface>();

  if (can0_->connect("can0")) {
    RCLCPP_INFO(this->get_logger(), "Finished Initializing CAN");
  } else
    RCLCPP_INFO(this->get_logger(), "Failed To Initialize CAN");

  while (can0_->read_can())
    ;

  return;
}

void JetsonInterface::initSensors() {}

void JetsonInterface::controlCmdCB(const utfr_msgs::msg::ControlCmd &msg) {
  const std::string function_name{"controlCmdCB:"};

  // Get new commands
  braking_cmd_ = msg.brk_cmd;
  steering_rate_cmd_ = msg.str_cmd;
  throttle_cmd_ = msg.thr_cmd;

  //*******   Steering   *******

  // Contruct command to send
  uint16_t steeringRateToSC = 0;
  bool directionBit;

  if (!(abs(current_steering_angle_) > 750))
    steeringRateToSC = abs(steering_rate_cmd_);
  steeringRateToSC &= 0x0FFF;

  RCLCPP_INFO(this->get_logger(), "Steering Rate CMD: %d", steeringRateToSC);

  if (steering_rate_cmd_ < 0) {
    directionBit = 0;
  } else {
    directionBit = 1;
  }
  RCLCPP_INFO(this->get_logger(), "Steering Direction CCW: %d", directionBit);

  steeringRateToSC |= (uint16_t)(directionBit << 12);

  // Send command
  can1_->write_can(msg_dvjet_sensor_e::STR_RATE_CMD, (steeringRateToSC));

  //*******   Braking   *******
  RCLCPP_INFO(this->get_logger(), "Braking PWM: %d", braking_cmd_);
  can1_->write_can(msg_dvjet_sensor_e::BRK_RATE_CMD, braking_cmd_);

  //*******   Throttle   *******
  // TO DO: ADD THR CHECKS
  RCLCPP_INFO(this->get_logger(), "Sending Throttle: %f",
              (double)(int)((throttle_cmd_)));

  can1_->write_can(msg_dvjet_sensor_e::DV_THR_COMMAND,
                   (double)(((throttle_cmd_ << 1) | 0x1)));
}

void JetsonInterface::systemStatusCB(const utfr_msgs::msg::SystemStatus &) {
  utfr_msgs::msg::SystemStatus system_status_;
  const std::string function_name{"systemStatusCB:"};
}

void JetsonInterface::timerCB() {
  int status = utfr_msgs::msg::Heartbeat::ACTIVE;
  const std::string function_name{"timerCB:"};

  try {
    // Read CAN messages from car and send to rest of stack

    //*****   Steering   *****
    // Get steering angle:
    current_steering_angle_ =
        -((int16_t)(can1_->get_can(msg_dvjet_sensor_e::ANGSENREC)) / 10);

    // Check for sensor malfunction:
    if (current_steering_angle_ != -3276) {
      jetson_cmd_.steering_angle = current_steering_angle_;
      RCLCPP_INFO(this->get_logger(), "Steer Angle: %d",
                  current_steering_angle_);
    } else {
      // Should there be a specific error for each type of fault
      status = utfr_msgs::msg::Heartbeat::FATAL;
    }

    //*****   Brakes   *****
    // Get front and rear brake pressure
    front_brake_pressure_ = (can1_->get_can(msg_dvjet_sensor_e::FBP));
    rear_brake_pressure_ = (can1_->get_can(msg_dvjet_sensor_e::RBP));
    jetson_cmd_.asb_pressure_front = front_brake_pressure_;
    jetson_cmd_.asb_pressure_rear = rear_brake_pressure_;

    // Read motor speed:
    motor_speed_ = can1_->get_can(msg_dvjet_sensor_e::MOTPOS);
    motor_speed_ *= -0.021545;
    jetson_cmd_.motor_speed = motor_speed_;

    jetson_cmd_.header.stamp = this->get_clock()->now();
    jetson_publisher_->publish(jetson_cmd_);

    //***** AS State *****
    // DV driving dynamics 1:
    long dv_driving_dynamics_1 =
        can1->get_can(msg_dvjet_sensor_e::DVDrivingDynamics1);

    as_state_.speed_actual = (uint8_t)dv_driving_dynamics_1 & 0xFF;
    as_state_.speed_target = (uint8_t)(dv_driving_dynamics_1 << (8)) & 0xFF;

    as_state_.steering_angle_actual =
        (uint8_t)((dv_driving_dynamics_1 << (8 * 2)) & 0xFF);
    as_state_.steering_angle_target =
        (uint8_t)((dv_driving_dynamics_1 << (8 * 3)) & 0xFF);

    as_state_.brake_hydr_actual =
        (uint8_t)((dv_driving_dynamics_1 << (8 * 4)) & 0xFF);
    as_state_.brake_hydr_target =
        (uint8_t)((dv_driving_dynamics_1 << (8 * 5)) & 0xFF);

    as_state_.motor_moment_actual =
        (uint8_t)((dv_driving_dynamics_1 << (8 * 6)) & 0xFF);
    as_state_.motor_moment_target =
        (uint8_t)((dv_driving_dynamics_1 << (8 * 7)) & 0xFF);

    // Dv driving dynamics 2:
    long dv_driving_dynamics_2 =
        can1->get_can(msg_dvjet_sensor_e::DVDrivingDynamics2);

    as_state_.acceleration_longitudinal =
        (int16_t)dv_driving_dynamics_2 & 0xFFFF;
    as_state_.acceleration_lateral =
        (int16_t)(dv_driving_dynamics_2 << (16)) & 0xFFFF;
    as_state_.yaw_rate =
        (int16_t)((dv_driving_dynamics_2 << (16 * 2)) & 0xFFFF);

    // DV system status:
    long dv_system_status = can1->get_can(msg_dvjet_sensor_e::DVSystemStatus);

    switch (dv_system_status & 0x7) {
    case 1:
      as_state_.as_state = AS_STATE_OFF;
      break;
    case 2:
      as_state_.as_state = AS_STATE_READY;
      break;
    case 3:
      as_state_.as_state = AS_STATE_DRIVING;
      break;
    case 4:
      as_state_.as_state = AS_STATE_EMERGENCY_BRAKE;
      break;
    case 5:
      as_state_.as_state = AS_STATE_FINISH;
      break;
    default:
      as_state_.as_state = AS_STATE_OFF;
      break;
    }

    switch ((dv_system_status << 3) & 0x3) {
    case 1:
      as_state_.ebs_state = EBS_STATE_UNAVAILABLE;
      break;
    case 2:
      as_state_.ebs_state = EBS_STATE_ARMED;
      break;
    case 3:
      as_state_.ebs_state = EBS_STATE_ACTIVATED;
      break;
    default:
      as_state_.as_state = EBS_STATE_UNAVAILABLE;
      break;
    }

    switch ((dv_system_status << 5) & 0x7) {
    case 1:
      as_state_.ami_state = AMI_STATE_ACCELERATION;
      break;
    case 2:
      as_state_.ami_state = AMI_STATE_SKIDPAD;
      break;
    case 3:
      as_state_.ami_state = AMI_STATE_TRACKDRIVE;
      break;
    case 4:
      as_state_.ami_state = AMI_STATE_BRAKETEST;
      break;
    case 5:
      as_state_.ami_state = AMI_STATE_INSPECTION;
      break;
    case 6:
      as_state_.ami_state = AMI_STATE_AUTOCROSS;
      break;
    default:
      as_state_.ami_state = AMI_STATE_BRAKETEST;
      break;
    }

    as_state_.steering_state = (dv_system_status << 8) & 0x1;

    switch ((dv_system_status << 9) & 0x3) {
    case 1:
      as_state_.service_brake_state = SERVICE_BRAKE_STATE_DISENGAGED;
      break;
    case 2:
      as_state_.service_brake_state = SERVICE_BRAKE_STATE_ENGAGED;
      break;
    case 3:
      as_state_.service_brake_state = SERVICE_BRAKE_STATE_AVAILABLE;
      break;
    default:
      as_state_.service_brake_state = SERVICE_BRAKE_STATE_DISENGAGED;
      break;
    }

    as_state_.lap_counter = (uint8_t)(dv_system_status << 11) & 0x15;

    as_state_.cones_count_actual = (uint8_t)(dv_system_status << 15) & 0xFF;

    as_state_.cones_count_actual = (uint)(dv_system_status << 23) & 0x1FFFF;

    as_state_.header.stamp = this->get_clock()->now();
    system_status_publisher_->publish(jetson_cmd_);
  } catch (int e) {
    RCLCPP_INFO(this->get_logger(), "timerCB: Error occured, error #%d", e);
    status = utfr_msgs::msg::Heartbeat::FATAL;
  }
  this->publishHeartbeat(status);
}

} // namespace jetson_interface
} // namespace utfr_dv
