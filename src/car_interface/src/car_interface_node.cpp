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
  this->initCAN();
  this->initTimers();
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

void CarInterface::heartbeatCB(const utfr_msgs::msg::Heartbeat &msg) {
  heartbeat_monitor_->updateHeartbeat(msg, this->get_clock()->now());
}

void CarInterface::controlCmdCB(const utfr_msgs::msg::ControlCmd &msg) {
  const std::string function_name{"controlCmdCB"};

  //*******   Steering   *******

  // Contruct command to send
  uint16_t steeringRateToSC = abs(msg.str_cmd) & 0x0FFF;
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

  //*******   Braking   *******
  RCLCPP_INFO(this->get_logger(), "Braking PWM: %d", braking_cmd_);

  //*******   Throttle   *******
  // TODO: ADD THR CHECKS
  RCLCPP_INFO(this->get_logger(), "Throttle: %f",
              (double)(int)((throttle_cmd_)));

  // Finalize commands
  if (cmd_) {
    braking_cmd_ = msg.brk_cmd;
    steering_cmd_ = steeringRateToSC;
    throttle_cmd_ = msg.thr_cmd;
  } else {
    braking_cmd_ = 0;
    steering_cmd_ = 0;
    throttle_cmd_ = 0;
  }

  // TODO: These are probably not correct unit/format, brake is still PWM
  system_status_.brake_hydr_target = braking_cmd_;    // TODO: Convert to %
  system_status_.motor_moment_target = throttle_cmd_; // TODO: Convert to %
}

void CarInterface::EgoStateCB(const utfr_msgs::msg::EgoState &msg) {
  system_status_.speed_actual = msg.vel.twist.linear.x * 3.6;
  system_status_.acceleration_longitudinal = msg.accel.accel.linear.x;
  system_status_.acceleration_lateral = -msg.accel.accel.linear.y;
  system_status_.yaw_rate = -msg.vel.twist.angular.z;
  system_status_.lap_counter = msg.lap_count; // TODO: Should this be in
                                              // ego state or in a planning msg
  finished_ = msg.finished;
}

void CarInterface::TargetStateCB(const utfr_msgs::msg::TargetState &msg) {
  system_status_.speed_target = msg.speed * 3.6;
  system_status_.steering_angle_target = -msg.steering_angle;
}

void CarInterface::getSteeringAngleSensorData() {
  const std::string function_name{"getSteeringAngleSensorData"};
  int16_t steering_angle; // TODO: Check proper var type

  try {
    // TODO: Check value format
    steering_angle = -((int16_t)(can1_->get_can(dv_can_msg::ANGSENREC)) / 10);

    // Check for sensor malfunction
    if ((steering_angle == -3276) | (abs(steering_angle) > 750)) {
      RCLCPP_ERROR(this->get_logger(), "Steering angle sensor error");
      // TODO: Error handling function, change control cmds to 0 and trigger EBS
    } else {
      // TODO: Check frame
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
  double motor_speed; // TODO: Check proper var type

  try {
    // TODO: Check value format
    motor_speed = can1_->get_can(dv_can_msg::MOTPOS) *
                  -0.021545; // TODO: Where is this factor from?
    if (abs(motor_speed) > 32767) {
      RCLCPP_ERROR(this->get_logger(), "Motor speed value error");
    } else {
      sensor_can_.motor_speed = motor_speed;
    }
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getMotorTorqueData() {
  const std::string function_name{"getMotorTorqueData"};
  double motor_torque; // TODO: Check proper var type

  try {
    // TODO: Check value format
    motor_torque = can1_->get_can(dv_can_msg::APPS);
    if (abs(motor_torque) > 3276.7) {
      RCLCPP_ERROR(this->get_logger(), "Motor torque value error");
    } else {
      system_status_.motor_moment_actual = motor_torque; // TODO: Convert to %
    }
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getServiceBrakeData() {
  const std::string function_name{"getServiceBrakeData"};
  // uint16_t asb_pressure_front; // TODO: Check proper var type
  uint16_t asb_pressure_rear; // TODO: Check proper var type

  try {
    // TODO: Check value format
    // asb_pressure_front = (can1_->get_can(dv_can_msg::FBP));
    asb_pressure_rear = (can1_->get_can(dv_can_msg::RBP));

    // TODO: Check for sensor malfunction
    if (false) {
      RCLCPP_ERROR(this->get_logger(), "Service brake pressure value error");
    } else {
      system_status_.brake_hydr_actual =
          (int)(100 * asb_pressure_rear / MAX_BRK_PRS); // Converting to %
    }
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
    // TODO: Proper CAN message
    // wheelspeed_fl =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TODO: Proper CAN message
    // wheelspeed_fr =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TODO: Proper CAN message
    // wheelspeed_rl =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TODO: Proper CAN message
    // wheelspeed_rr =
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));

    // TODO: Check for sensor malfunction
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
    // TODO: Check reference frames, double check value format
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

    // TODO: Check for sensor malfunction
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
  long dv_state = can1_->get_can(dv_can_msg::DV_STATE);

  system_status_.as_state = dv_state & 0x7;

  system_status_.ebs_state = (dv_state << 3) & 0x3;

  system_status_.ami_state = (dv_state << 5) & 0x7;

  system_status_.steering_state = (dv_state << 8) & 0x1;

  system_status_.service_brake_state = (dv_state << 9) & 0x3;
}

void CarInterface::setDVLogs() {
  const std::string function_name{"setDVLogs"};

  // TODO: Check all values to follow FSG format

  try {
    // DV driving dynamics 1
    long dv_driving_dynamics_1 = 0;
    uint8_t i = 0; // To iterate through each byte of message

    dv_driving_dynamics_1 |= system_status_.speed_actual << (8 * (i++));
    dv_driving_dynamics_1 |= system_status_.speed_target << (8 * (i++));
    dv_driving_dynamics_1 = system_status_.speed_target << (8 * (i++));
    dv_driving_dynamics_1 |= system_status_.steering_angle_actual
                             << (8 * (i++));
    dv_driving_dynamics_1 |= system_status_.steering_angle_target
                             << (8 * (i++));
    dv_driving_dynamics_1 |= system_status_.brake_hydr_actual << (8 * (i++));
    dv_driving_dynamics_1 |= system_status_.brake_hydr_target << (8 * (i++));
    dv_driving_dynamics_1 |= system_status_.motor_moment_actual << (8 * (i++));
    dv_driving_dynamics_1 |= system_status_.motor_moment_target << (8 * (i++));

    can1_->write_can(dv_can_msg::DVDrivingDynamics1, dv_driving_dynamics_1);

    // Dv driving dynamics 2
    long dv_driving_dynamics_2 = 0;
    i = 0;

    dv_driving_dynamics_2 |= system_status_.acceleration_longitudinal
                             << (16 * (i++));
    dv_driving_dynamics_2 |= system_status_.acceleration_lateral
                             << (16 * (i++));
    dv_driving_dynamics_2 |= system_status_.yaw_rate << (16 * (i++));

    can1_->write_can(dv_can_msg::DVDrivingDynamics2, dv_driving_dynamics_2);

    // DV system status
    long dv_system_status = 0;
    i = 0;

    dv_system_status |= (system_status_.as_state & 0x7);
    dv_system_status |= (system_status_.ebs_state & 0x3) << 3;
    dv_system_status |= (system_status_.ami_state & 0x7) << 5;
    dv_system_status |= (system_status_.steering_state & 0x1) << 8;
    dv_system_status |= (system_status_.service_brake_state & 0x3) << 9;
    dv_system_status |= (system_status_.lap_counter & 0x15) << 11;
    dv_system_status |= (system_status_.cones_count_actual & 0xFF)
                        << 15; // TODO: Get from perception
    dv_system_status |= (system_status_.cones_count_all & 0x1FFFF)
                        << 23; // TODO: Get from perception

    system_status_.header.stamp = this->get_clock()->now();
    system_status_publisher_->publish(system_status_);

    can1_->write_can(dv_can_msg::DVSystemStatus, dv_system_status);

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::setDVStateAndCommand() {
  const std::string function_name{"setDVStateAndCommand"};

  try {
    bool heartbeat_status =
        heartbeat_monitor_->verifyHeartbeats(this->get_clock()->now());

    switch (system_status_.as_state) {
    case utfr_msgs::msg::SystemStatus::AS_STATE_OFF: {
      // Check if autonomous mission is set
      if (system_status_.ami_state ==
              std::clamp(system_status_.ami_state,
                         utfr_msgs::msg::SystemStatus::AMI_STATE_ACCELERATION,
                         utfr_msgs::msg::SystemStatus::AMI_STATE_AUTOCROSS) &&
          !launched_) {
        launchMission(); // Launch other dv nodes
        launched_ = true;
      } else if (heartbeat_status) {
        dv_pc_state_ = DV_PC_STATE::READY;
        cmd_ = false;
      } else {
        dv_pc_state_ = DV_PC_STATE::OFF;
        cmd_ = false;
      }
      break;
    }
    case utfr_msgs::msg::SystemStatus::AS_STATE_READY: {
      if (heartbeat_status) {
        dv_pc_state_ = DV_PC_STATE::READY;
        cmd_ = false;
      } else {
        dv_pc_state_ = DV_PC_STATE::EMERGENCY; // Is this required or can we go
                                               // back to off?
        cmd_ = false;
      }
      break;
    }
    case utfr_msgs::msg::SystemStatus::AS_STATE_DRIVING: {
      if (heartbeat_status) {
        dv_pc_state_ = DV_PC_STATE::DRIVING;
        cmd_ = true;
        if (finished_) {
          dv_pc_state_ = DV_PC_STATE::FINISH;
        }
      } else {
        dv_pc_state_ = DV_PC_STATE::EMERGENCY;
        cmd_ = false;
      }
      break;
    }
    case utfr_msgs::msg::SystemStatus::AS_STATE_EMERGENCY_BRAKE: {
      dv_pc_state_ = DV_PC_STATE::EMERGENCY;
      cmd_ = false;
      if (!shutdown_) {
        shutdownNodes();
        shutdown_ = true;
      }
      break;
    }
    case utfr_msgs::msg::SystemStatus::AS_STATE_FINISH: {
      dv_pc_state_ = DV_PC_STATE::FINISH; // Should already be finish
      cmd_ = false;                       // Should already be false
      if (!shutdown_) {
        shutdownNodes();
        shutdown_ = true;
      }
      break;
    }
    default: {
      dv_pc_state_ = DV_PC_STATE::EMERGENCY;
      cmd_ = false;
      if (!shutdown_) {
        shutdownNodes();
        shutdown_ = true;
      }
      break;
    }
    }

    // Write to can
    long dv_command = 0;
    dv_command |= dv_pc_state_;
    if (cmd_) {
      dv_command |= (throttle_cmd_ & 0xFFFF) << 3;
      dv_command |= (steering_cmd_ & 0x1FFF) << 19;
      dv_command |= (braking_cmd_ & 0xFF) << 32;
    }
    can1_->write_can(dv_can_msg::DV_COMMAND, dv_command);

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::launchMission() {
  const std::string function_name{"launchMission"};

  std::string launchCmd =
      "ros2 launch launcher dv.launch.py -p mission:=" +
      std::to_string(system_status_.ami_state); // TODO: Param implementation
  // Execute the launch command
  int result = std::system(launchCmd.c_str());

  if (result != 0) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured",
                 function_name.c_str());
  }
}

void CarInterface::shutdownNodes() {
  const std::string function_name{"shutdownNodes"};
  // TODO: Fix
  // // Create a node to retrieve the list of active nodes
  // auto node = std::make_shared<rclcpp::Node>("shutdown_nodes");

  // // Retrieve the list of active node names
  // auto node_names = rclcpp::Node::get_node_names(node);

  // for (const auto &name : node_names) {
  //   if (name != "car_interface") {
  //     auto node_interface = node->get_node_base_interface();
  //     node_interface->get_node_by_name(name)->shutdown();
  //   }
  // }

  // rclcpp::shutdown(); TODO: Decide to kill car_interface or not
}

void CarInterface::timerCB() {
  const std::string function_name{"timerCB"};

  try {
    getSensorCan(); // Publish sensor and state data that is read from can
    getDVState();   // Read DV state from car from can
    setDVLogs();    // Publish FSG log format over ros and send over can
    setDVStateAndCommand(); // Send state of dv computer and control cmd to car

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

} // namespace car_interface
} // namespace utfr_dv
