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

  steering_cmd_ = 0;
  braking_cmd_ = 0;
  throttle_cmd_ = 0;
}

void CarInterface::initSubscribers() {
  const std::string function_name{"CarInterface::initSubscribers: "};
  control_cmd_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ControlCmd>(
          topics::kControlCmd, 10,
          std::bind(&CarInterface::controlCmdCB, this, _1));

  ego_state_subscriber_ = this->create_subscription<utfr_msgs::msg::EgoState>(
      topics::kEgoState, 10, std::bind(&CarInterface::EgoStateCB, this, _1));

  target_state_subscriber_ =
      this->create_subscription<utfr_msgs::msg::TargetState>(
          topics::kTargetState, 10,
          std::bind(&CarInterface::TargetStateCB, this, _1));

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
  // can0_ = std::make_unique<CanInterface>();

  if (can1_->connect("can1")) {
    RCLCPP_INFO(this->get_logger(), "Finished Initializing CAN");
  } else
    RCLCPP_ERROR(this->get_logger(), "Failed To Initialize CAN");

  while (can1_->read_can())
    ;

  // if (can0_->connect("can0")) {
  //   RCLCPP_INFO(this->get_logger(), "Finished Initializing CAN");
  // } else
  //   RCLCPP_ERROR(this->get_logger(), "Failed To Initialize CAN");

  // while (can0_->read_can())
  //   ;

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
  const std::string function_name{"controlCmdCB"};

  // Get latest commands
  braking_cmd_ = msg.brk_cmd;
  steering_cmd_ = msg.str_cmd;
  throttle_cmd_ = msg.thr_cmd;

  // Write system status msg
  // front is 57% less, rear scale is 0-1600
  system_status_.brake_hydr_target = msg.brk_cmd;   // TODO: convert to %
  system_status_.motor_moment_target = msg.thr_cmd; // TODO: convert to %

  //*******   Steering   *******

  // Contruct command to send
  uint16_t steeringRateToSC = abs(steering_cmd_) & 0x0FFF;
  bool directionBit;

  RCLCPP_INFO(this->get_logger(), "Steering Rate CMD: %d", steeringRateToSC);

  if (steering_cmd_ < 0) {
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
  RCLCPP_INFO(this->get_logger(), "Throttle: %f",
              (double)(int)((throttle_cmd_)));

  can1_->write_can(dv_can_msg::DV_THR_COMMAND,
                   (double)(((throttle_cmd_ << 1) | 0x1)));
}

void CarInterface::EgoStateCB(const utfr_msgs::msg::EgoState &msg) {
  system_status_.speed_actual = msg.vel.twist.linear.x * 3.6;

  system_status_.brake_hydr_actual =
      sensor_can_.asb_pressure_front; // TODO: weighted avg of front/rear
  system_status_.motor_moment_actual = sensor_can_.motor_speed; // TODO: change
  system_status_.acceleration_longitudinal = msg.accel.accel.linear.x;
  system_status_.acceleration_lateral = -msg.accel.accel.linear.y;
  system_status_.yaw_rate = -msg.vel.twist.angular.z;
}

void CarInterface::TargetStateCB(const utfr_msgs::msg::TargetState &msg) {
  system_status_.speed_target = msg.velocity * 3.6;
  system_status_.steering_angle_target = -msg.steering_angle;
}

void CarInterface::getSteeringAngleSensorData() {
  const std::string function_name{"getSteeringAngleSensorData"};
  int16_t steering_angle;

  try {
    // TO DO: Check value format
    steering_angle = -((int16_t)(can1_->get_can(dv_can_msg::ANGSENREC)) / 10);

    // Check for sensor malfunction
    if ((steering_angle == -3276) | (abs(steering_angle) > 750)) {
      RCLCPP_ERROR(this->get_logger(), "Steering angle sensor error");
      // TODO: Error handling function, change control cmds to 0 and trigger EBS
    } else {
      sensor_can_.steering_angle = steering_angle;
      system_status_.steering_angle_actual = -sensor_can_.steering_angle;
    }
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
    // TODO: Error handling function, change control cmds to 0 and trigger EBS
  }
}

void CarInterface::getMotorSpeedData() {
  const std::string function_name{"getMotorSpeedData"};
  double motor_speed;

  try {
    // TO DO: Check value format
    motor_speed = can1_->get_can(dv_can_msg::MOTPOS) * -0.021545;

    // TO DO: Check for sensor malfunction
    // if () {
    //   RCLCPP_ERROR(this->get_logger(), "Motor speed value error");
    // } else {
    //    sensor_can_.motor_speed = motor_speed;
    // }
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getServiceBrakeData() {
  const std::string function_name{"getServiceBrakeData"};
  uint16_t asb_pressure_front; // TODO: Check proper var type
  uint16_t asb_pressure_rear;  // TODO: Check proper var type

  try {
    // TO DO: Check value format
    asb_pressure_front = (can1_->get_can(dv_can_msg::FBP));
    asb_pressure_rear = (can1_->get_can(dv_can_msg::RBP));

    // TO DO: Check for sensor malfunction
    // if () {
    //   RCLCPP_ERROR(this->get_logger(), "Service brake pressure value error");
    // } else {
    // sensor_can_.asb_pressure_front = asb_pressure_front;
    // sensor_can_.asb_pressure_rear = asb_pressure_rear;
    // }
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getWheelspeedSensorData() {
  const std::string function_name{"getWheelspeedSensorData"};
  double wheelspeed_fl; // TODO: Check proper var type
  double wheelspeed_fr; // TODO: Check proper var type
  double wheelspeed_rl; // TODO: Check proper var type
  double wheelspeed_rr; // TODO: Check proper var type

  try {
    // TO DO: Proper CAN message
    // wheelspeed_fl =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TO DO: Proper CAN message
    // wheelspeed_fr =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TO DO: Proper CAN message
    // wheelspeed_rl =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TO DO: Proper CAN message
    // wheelspeed_rr =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));

    // TO DO: Check for sensor malfunction
    // if () {
    //   RCLCPP_ERROR(this->get_logger(), "Wheel speed value error");
    // } else {
    //   sensor_can_.wheelspeed_fl = wheelspeed_fl;
    //   sensor_can_.wheelspeed_fr = wheelspeed_fr;
    //   sensor_can_.wheelspeed_rl = wheelspeed_rl;
    //   sensor_can_.wheelspeed_rr = wheelspeed_rr;
    // }
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getIMUData() {
  const std::string function_name{"getIMUData"};

  try {
    // TO DO: Check reference frames, double check value format
    sensor_msgs::msg::Imu imu;
    imu.linear_acceleration.x =
        (double)((can1_->get_can(dv_can_msg::ImuX) >> (8 * 4)) & 255) / 100;
    imu.linear_acceleration.y =
        -(double)((can1_->get_can(dv_can_msg::ImuY) >> (8 * 4)) & 255) / 100;
    imu.linear_acceleration.z =
        (double)((can1_->get_can(dv_can_msg::ImuZ) >> (8 * 4)) & 255) / 100;
    imu.angular_velocity.x =
        (double)(can1_->get_can(dv_can_msg::ImuX) & 255) / 10;
    imu.angular_velocity.y =
        (double)(can1_->get_can(dv_can_msg::ImuY) & 255) / 10;

    // TO DO: Check for sensor malfunction
    // if () {
    //   RCLCPP_ERROR(this->get_logger(), "IMU value error");
    // } else {
    //   sensor_can_.imu_data = imu;
    // }
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getSensorCan() {
  const std::string function_name{"getSensorCan"};

  try {
    // Read sensor CAN messages from car
    getSteeringAngleSensorData();
    getMotorSpeedData();
    getServiceBrakeData();
    getWheelspeedSensorData();
    getIMUData();

    sensor_can_.header.stamp = this->get_clock()->now();
    sensor_can_publisher_->publish(sensor_can_);
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getDVState() {
  // Get DV state from car
  long dv_state = can1_->get_can(dv_can_msg::DVState);

  system_status_.as_state = dv_state & 0x7;

  system_status_.ebs_state = (dv_state << 3) & 0x3;

  system_status_.ami_state = (dv_state << 5) & 0x7;

  system_status_.steering_state = (dv_state << 8) & 0x1;

  system_status_.service_brake_state = (dv_state << 9) & 0x3;
}

void CarInterface::setDVLogs() {
  const std::string function_name{"setDVLogs"};

  try {
    // Read system state CAN messages from car

    // DV driving dynamics 1
    // CHANGE TO WRITING TO CAN
    long dv_driving_dynamics_1;

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
    // States are already set in getDVState, but still need to write to can msg

    system_status_.lap_counter = (uint8_t)(dv_system_status << 11) & 0x15;

    system_status_.cones_count_actual =
        (uint8_t)(dv_system_status << 15) & 0xFF;

    system_status_.cones_count_actual =
        (uint)(dv_system_status << 23) & 0x1FFFF;

    can1_->write_can(dv_can_msg::STR_RATE_CMD, (steeringRateToSC));

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::setDVPCState() {
  const std::string function_name{"setDVPCState"};

  try {
    // TODO: Reading RES go/stop signal CAN msg
    // TODO: Review logic/edge cases
    bool heartbeat_status =
        heartbeat_monitor_->verifyHeartbeats(this->get_clock()->now());

    switch (system_status_.as_state) {
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
        system_status_.as_state =
            utfr_msgs::msg::SystemStatus::AS_STATE_DRIVING;
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
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::launchMission() {
  const std::string function_name{"launchMission"};

  try {
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
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::timerCB() {
  const std::string function_name{"timerCB"};

  try {
    // Publish sensor and state data that is read from CANbus
    getSensorCan();
    getSystemStatus();
    // Update status from DV software side
    setSystemStatusAS();
    system_status_.header.stamp = this->get_clock()->now();
    system_status_publisher_->publish(system_status_);

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

} // namespace car_interface
} // namespace utfr_dv
