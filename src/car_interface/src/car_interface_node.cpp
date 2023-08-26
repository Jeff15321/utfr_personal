/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: car_interface_node.cpp
* auth: Youssef Elhadad, Daniel Asadi
* desc: Read sensor can data from car, read/write system state data from/to car,
* process main state machine logic
*/

#include <car_interface_node.hpp>

namespace utfr_dv {
namespace car_interface {

CarInterface::CarInterface() : Node("car_interface_node") {
  this->initParams();
  this->initMonitor();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initCAN();
  this->initSensors();
}

void CarInterface::initParams() {
  std::vector<std::string> default_modules = {"perception", "mapping", "ekf",
                                              "planning", "controls"};

  // Initialize Params with default values
  this->declare_parameter("update_rate", 33.33);
  this->declare_parameter("heartbeat_tolerance", 1.5);
  this->declare_parameter("heartbeat_modules", default_modules);

  update_rate_ = this->get_parameter("update_rate").as_double();
  heartbeat_tolerance_ = this->get_parameter("heartbeat_tolerance").as_double();
  heartbeat_modules_ =
      this->get_parameter("heartbeat_modules").as_string_array();

  steering_rate_cmd_ = 0;
  braking_cmd_ = 0;
  throttle_cmd_ = 0;

  motor_speed_ = 0.0;
  // imu_accel_.x = 0.0;
  // imu_accel_.y = 0.0;
  // imu_accel_.z = 0.0;

  current_steering_angle_ = 0;
}

void CarInterface::initSubscribers() {
  const std::string function_name{"CarInterface::initSubscribers: "};
  control_cmd_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ControlCmd>(
          topics::kControlCmd, 10,
          std::bind(&CarInterface::controlCmdCB, this, _1));

  for (const auto &module_name : heartbeat_modules_) {

    auto search = heartbeat_topics_map_.find(module_name);
    if (search == heartbeat_topics_map_.end()) { // Module not found :
      RCLCPP_ERROR(this->get_logger(), "%s Module %s topic not in map",
                   function_name.c_str(), module_name.c_str());
      continue;
    }

    std::string topic = heartbeat_topics_map_[module_name];
    heartbeat_subscribers_[module_name] =
        this->create_subscription<utfr_msgs::msg::Heartbeat>(
            topic, 10, std::bind(&CarInterface::heartbeatCB, this, _1));
  }

  RCLCPP_INFO(this->get_logger(), "Finished Initializing Subscribers");
}

void CarInterface::initPublishers() {
  sensor_can_publisher_ =
      this->create_publisher<utfr_msgs::msg::SensorCan>(topics::kSensorCan, 10);
  system_status_publisher_ =
      this->create_publisher<utfr_msgs::msg::SystemStatus>(
          topics::kSystemStatus, 1);

  RCLCPP_INFO(this->get_logger(), "Finished Initializing Publishers");
}

void CarInterface::initTimers() {
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(this->update_rate_),
      std::bind(&CarInterface::timerCB, this));

  RCLCPP_INFO(this->get_logger(), "Finished Initializing Timer");
}

void CarInterface::initCAN() {
  can1_ = std::make_unique<CanInterface>();
  can0_ = std::make_unique<CanInterface>();

  if (can1_->connect("can1")) {
    RCLCPP_INFO(this->get_logger(), "Finished Initializing CAN");
  } else
    RCLCPP_ERROR(this->get_logger(), "Failed To Initialize CAN");

  while (can1_->read_can())
    ;

  if (can0_->connect("can0")) {
    RCLCPP_INFO(this->get_logger(), "Finished Initializing CAN");
  } else
    RCLCPP_ERROR(this->get_logger(), "Failed To Initialize CAN");

  while (can0_->read_can())
    ;

  return;
}

void CarInterface::initMonitor() {
  heartbeat_monitor_ = std::make_unique<HeartbeatMonitor>(
      heartbeat_modules_, this->get_clock()->now(), heartbeat_tolerance_);
}

void CarInterface::initSensors() {}

void CarInterface::heartbeatCB(const utfr_msgs::msg::Heartbeat &msg) {
  heartbeat_monitor_->updateHeartbeat(msg, this->get_clock()->now());
}

void CarInterface::controlCmdCB(const utfr_msgs::msg::ControlCmd &msg) {
  const std::string function_name{"controlCmdCB:"};

  // Get latest commands
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
  can1_->write_can(dv_can_msg::STR_RATE_CMD, (steeringRateToSC));

  //*******   Braking   *******
  RCLCPP_INFO(this->get_logger(), "Braking PWM: %d", braking_cmd_);
  can1_->write_can(dv_can_msg::BRK_RATE_CMD, braking_cmd_);

  //*******   Throttle   *******
  // TO DO: ADD THR CHECKS
  RCLCPP_INFO(this->get_logger(), "Sending Throttle: %f",
              (double)(int)((throttle_cmd_)));

  can1_->write_can(dv_can_msg::DV_THR_COMMAND,
                   (double)(((throttle_cmd_ << 1) | 0x1)));
}

void CarInterface::getSteeringAngleSensorData() {
  const std::string function_name{"getSteeringAngleSensorData:"};

  try {
    current_steering_angle_ =
        -((int16_t)(can1_->get_can(dv_can_msg::ANGSENREC)) / 10);

    // Check for sensor malfunction
    if (current_steering_angle_ == -3276) {
      RCLCPP_ERROR(this->get_logger(), "Steering angle sensor error");
    } else {
      sensor_can_.steering_angle = current_steering_angle_;
    }
  } catch (int e) {
    RCLCPP_INFO(this->get_logger(),
                "getSteeringAngleSensorData: Error occured, error #%d", e);
  }
}

void CarInterface::getMotorSpeedData() {
  const std::string function_name{"getMotorSpeedData:"};

  try {
    motor_speed_ = can1_->get_can(dv_can_msg::MOTPOS) * -0.021545;
    sensor_can_.motor_speed = motor_speed_;

    // TO DO: Check for sensor malfunction
    // if () {
    //   RCLCPP_ERROR(this->get_logger(), "Motor speed value error");
    // } else {
    //
    // }
  } catch (int e) {
    RCLCPP_INFO(this->get_logger(),
                "getMotorSpeedData: Error occured, error #%d", e);
  }
}

void CarInterface::getServiceBrakeData() {
  const std::string function_name{"getServiceBrakeData:"};

  try {
    asb_pressure_front_ = (can1_->get_can(dv_can_msg::FBP));
    asb_pressure_rear_ = (can1_->get_can(dv_can_msg::RBP));

    // TO DO: Check for sensor malfunction
    // if () {
    //   RCLCPP_ERROR(this->get_logger(), "Service brake pressure value error");
    // } else {
    // sensor_can_.asb_pressure_front = asb_pressure_front_;
    // sensor_can_.asb_pressure_rear = asb_pressure_rear_;
    // }
  } catch (int e) {
    RCLCPP_INFO(this->get_logger(),
                "getServiceBrakeData: Error occured, error #%d", e);
  }
}

void CarInterface::getEBSPressureData() {
  const std::string function_name{"getEBSPressureData:"};

  try {
    // TO DO: Proper CAN message
    // ebs_pressure_1_ =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TO DO: Proper CAN message
    // ebs_pressure_2_ =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));

    // TO DO: Check for sensor malfunction
    // if () {
    //   RCLCPP_ERROR(this->get_logger(), "EBS pressure value error");
    // } else {
    //   sensor_can_.ebs_pressure_1 = ebs_pressure_1_;
    //   sensor_can_.ebs_pressure_2 = ebs_pressure_2_;
    // }
  } catch (int e) {
    RCLCPP_INFO(this->get_logger(),
                "getEBSPressureData: Error occured, error #%d", e);
  }
}

void CarInterface::getWheelspeedSensorData() {
  const std::string function_name{"getWheelspeedSensorData:"};

  try {
    // TO DO: Proper CAN message
    // wheelspeed_fl_ =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TO DO: Proper CAN message
    // wheelspeed_fr_ =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TO DO: Proper CAN message
    // wheelspeed_rl_ =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TO DO: Proper CAN message
    // wheelspeed_rr_ =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));

    // TO DO: Check for sensor malfunction
    // if () {
    //   RCLCPP_ERROR(this->get_logger(), "Wheel speed value error");
    // } else {
    //   sensor_can_.wheelspeed_fl = wheelspeed_fl_;
    //   sensor_can_.wheelspeed_fr = wheelspeed_fr_;
    //   sensor_can_.wheelspeed_rl = wheelspeed_rl_;
    //   sensor_can_.wheelspeed_rr = wheelspeed_rr_;
    // }
  } catch (int e) {
    RCLCPP_INFO(this->get_logger(),
                "getWheelspeedSensorData: Error occured, error #%d", e);
  }
}

void CarInterface::getIMUData() {
  const std::string function_name{"getIMUData:"};

  try {
    // TO DO: Proper CAN message
    // imu_ =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));

    // TO DO: Check for sensor malfunction
    // if () {
    //   RCLCPP_ERROR(this->get_logger(), "IMU value error");
    // } else {
    //   sensor_can_.imu_data = imu_;
    // }
  } catch (int e) {
    RCLCPP_INFO(this->get_logger(), "getIMUData: Error occured, error #%d", e);
  }
}

void CarInterface::getSensorCan() {
  const std::string function_name{"getSensorCan:"};

  try {
    // Read sensor CAN messages from car
    getSteeringAngleSensorData();
    getMotorSpeedData();
    getServiceBrakeData();
    getEBSPressureData();
    getWheelspeedSensorData();
    getIMUData();

    sensor_can_.header.stamp = this->get_clock()->now();
    sensor_can_publisher_->publish(sensor_can_);
  } catch (int e) {
    RCLCPP_INFO(this->get_logger(), "getSensorCan: Error occured, error #%d",
                e);
  }
}

void CarInterface::getSystemStatus() {
  const std::string function_name{"getSystemStatus:"};

  try {
    // Read system state CAN messages from car

    // DV driving dynamics 1
    long dv_driving_dynamics_1 = can1_->get_can(dv_can_msg::DVDrivingDynamics1);

    system_status_.speed_actual = (uint8_t)dv_driving_dynamics_1 & 0xFF;
    system_status_.speed_target =
        (uint8_t)(dv_driving_dynamics_1 << (8)) & 0xFF;

    system_status_.steering_angle_actual =
        (uint8_t)((dv_driving_dynamics_1 << (8 * 2)) & 0xFF);
    system_status_.steering_angle_target =
        (uint8_t)((dv_driving_dynamics_1 << (8 * 3)) & 0xFF);

    system_status_.brake_hydr_actual =
        (uint8_t)((dv_driving_dynamics_1 << (8 * 4)) & 0xFF);
    system_status_.brake_hydr_target =
        (uint8_t)((dv_driving_dynamics_1 << (8 * 5)) & 0xFF);

    system_status_.motor_moment_actual =
        (uint8_t)((dv_driving_dynamics_1 << (8 * 6)) & 0xFF);
    system_status_.motor_moment_target =
        (uint8_t)((dv_driving_dynamics_1 << (8 * 7)) & 0xFF);

    // Dv driving dynamics 2
    long dv_driving_dynamics_2 = can1_->get_can(dv_can_msg::DVDrivingDynamics2);

    system_status_.acceleration_longitudinal =
        (int16_t)dv_driving_dynamics_2 & 0xFFFF;
    system_status_.acceleration_lateral =
        (int16_t)(dv_driving_dynamics_2 << (16)) & 0xFFFF;
    system_status_.yaw_rate =
        (int16_t)((dv_driving_dynamics_2 << (16 * 2)) & 0xFFFF);

    // DV system status
    long dv_system_status = can1_->get_can(dv_can_msg::DVSystemStatus);

    switch (dv_system_status & 0x7) {
    case 1:
      system_status_.as_state = utfr_msgs::msg::SystemStatus::AS_STATE_OFF;
      break;
    case 2:
      system_status_.as_state = utfr_msgs::msg::SystemStatus::AS_STATE_READY;
      break;
    case 3:
      system_status_.as_state = utfr_msgs::msg::SystemStatus::AS_STATE_DRIVING;
      break;
    case 4:
      system_status_.as_state =
          utfr_msgs::msg::SystemStatus::AS_STATE_EMERGENCY_BRAKE;
      break;
    case 5:
      system_status_.as_state = utfr_msgs::msg::SystemStatus::AS_STATE_FINISH;
      break;
    default:
      system_status_.as_state = utfr_msgs::msg::SystemStatus::AS_STATE_OFF;
      break;
    }

    switch ((dv_system_status << 3) & 0x3) {
    case 1:
      system_status_.ebs_state =
          utfr_msgs::msg::SystemStatus::EBS_STATE_UNAVAILABLE;
      break;
    case 2:
      system_status_.ebs_state = utfr_msgs::msg::SystemStatus::EBS_STATE_ARMED;
      break;
    case 3:
      system_status_.ebs_state =
          utfr_msgs::msg::SystemStatus::EBS_STATE_ACTIVATED;
      break;
    default:
      system_status_.as_state =
          utfr_msgs::msg::SystemStatus::EBS_STATE_UNAVAILABLE;
      break;
    }

    switch ((dv_system_status << 5) & 0x7) {
    case 1:
      system_status_.ami_state =
          utfr_msgs::msg::SystemStatus::AMI_STATE_ACCELERATION;
      break;
    case 2:
      system_status_.ami_state =
          utfr_msgs::msg::SystemStatus::AMI_STATE_SKIDPAD;
      break;
    case 3:
      system_status_.ami_state =
          utfr_msgs::msg::SystemStatus::AMI_STATE_TRACKDRIVE;
      break;
    case 4:
      system_status_.ami_state =
          utfr_msgs::msg::SystemStatus::AMI_STATE_BRAKETEST;
      break;
    case 5:
      system_status_.ami_state =
          utfr_msgs::msg::SystemStatus::AMI_STATE_INSPECTION;
      break;
    case 6:
      system_status_.ami_state =
          utfr_msgs::msg::SystemStatus::AMI_STATE_AUTOCROSS;
      break;
    default:
      system_status_.ami_state =
          utfr_msgs::msg::SystemStatus::AMI_STATE_BRAKETEST;
      break;
    }

    system_status_.steering_state = (dv_system_status << 8) & 0x1;

    switch ((dv_system_status << 9) & 0x3) {
    case 1:
      system_status_.service_brake_state =
          utfr_msgs::msg::SystemStatus::SERVICE_BRAKE_STATE_DISENGAGED;
      break;
    case 2:
      system_status_.service_brake_state =
          utfr_msgs::msg::SystemStatus::SERVICE_BRAKE_STATE_ENGAGED;
      break;
    case 3:
      system_status_.service_brake_state =
          utfr_msgs::msg::SystemStatus::SERVICE_BRAKE_STATE_AVAILABLE;
      break;
    default:
      system_status_.service_brake_state =
          utfr_msgs::msg::SystemStatus::SERVICE_BRAKE_STATE_DISENGAGED;
      break;
    }

    system_status_.lap_counter = (uint8_t)(dv_system_status << 11) & 0x15;

    system_status_.cones_count_actual =
        (uint8_t)(dv_system_status << 15) & 0xFF;

    system_status_.cones_count_actual =
        (uint)(dv_system_status << 23) & 0x1FFFF;

  } catch (int e) {
    RCLCPP_INFO(this->get_logger(), "getSystemStatus: Error occured, error #%d",
                e);
  }
}

void CarInterface::setSystemStatusAS() {
  // TODO: Reading RES go/stop signal CAN msg
  // TODO: Review logic/edge cases
  bool heartbeat_status =
      heartbeat_monitor_->verifyHeartbeats(this->get_clock()->now());

  switch (system_status_.ami_state) {
  case utfr_msgs::msg::SystemStatus::AS_STATE_OFF: {
    gonogo_ = false; // debounce gonogo

    if (heartbeat_status) { // All critical modules loaded
      system_status_.as_state = utfr_msgs::msg::SystemStatus::AS_STATE_READY;
    }
    break;
  }
  case utfr_msgs::msg::SystemStatus::AS_STATE_READY: {
    if (!heartbeat_status) { // Heartbeats failed after loading correctly
      system_status_.as_state =
          utfr_msgs::msg::SystemStatus::AS_STATE_EMERGENCY_BRAKE;
    }

    if (gonogo_) { // RES Go recieved
      launchMission();
      system_status_.as_state = utfr_msgs::msg::SystemStatus::AS_STATE_DRIVING;
    }
    break;
  }
  case utfr_msgs::msg::SystemStatus::AS_STATE_DRIVING: {
    if (!heartbeat_status) { // Heartbeats failed after loading correctly
      system_status_.as_state =
          utfr_msgs::msg::SystemStatus::AS_STATE_EMERGENCY_BRAKE;
    }
    // TODO - switch to finished case when mission complete
    break;
  }
  case utfr_msgs::msg::SystemStatus::AS_STATE_EMERGENCY_BRAKE: {
    // TODO - shutdown system appropriately
    break;
  }
  case utfr_msgs::msg::SystemStatus::AS_STATE_FINISH: {
    // TODO - shutdown system appropriately
    break;
  }
  default: {
    // TODO
  }
  }
}

void CarInterface::launchMission() {
  switch (system_status_.as_state) {
  case utfr_msgs::msg::SystemStatus::AMI_STATE_ACCELERATION: {
    // TODO: roslaunch mission
    break;
  }
  case utfr_msgs::msg::SystemStatus::AMI_STATE_SKIDPAD: {
    // TODO: roslaunch mission
    break;
  }
  case utfr_msgs::msg::SystemStatus::AMI_STATE_TRACKDRIVE: {
    // TODO: roslaunch mission
    break;
  }
  case utfr_msgs::msg::SystemStatus::AMI_STATE_BRAKETEST: {
    // TODO: roslaunch mission
    break;
  }
  case utfr_msgs::msg::SystemStatus::AMI_STATE_INSPECTION: {
    // TODO: roslaunch mission
    break;
  }
  case utfr_msgs::msg::SystemStatus::AMI_STATE_AUTOCROSS: {
    // TODO: roslaunch mission
    break;
  }
  }
}

void CarInterface::timerCB() {
  const std::string function_name{"timerCB:"};

  try {
    // Publish sensor and state data read from CANbus
    getSensorCan();
    getSystemStatus();

    // after reading AS state from car, and updating from the DV software side,
    // publish
    system_status_.header.stamp = this->get_clock()->now();
    system_status_publisher_->publish(system_status_);

  } catch (int e) {
    RCLCPP_INFO(this->get_logger(), "timerCB: Error occured, error #%d", e);
  }
}

} // namespace car_interface
} // namespace utfr_dv
