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
  std::vector<std::string> default_modules = {
      "perception",          "lidar_proc",      "ekf",
      "mapping_build",       "mapping_compute", "planning_cp",
      "planning_controller", "controls",
  };

  this->declare_parameter("update_rate", 33.33);
  this->declare_parameter("ema_gain", 0.6);
  this->declare_parameter("heartbeat_tolerance", 1.5);
  this->declare_parameter("heartbeat_modules", default_modules);
  this->declare_parameter("heartbeat_modules_accel", default_modules);
  this->declare_parameter("heartbeat_modules_inspection", default_modules);
  this->declare_parameter("testing", 0);

  update_rate_ = this->get_parameter("update_rate").as_double();
  ema_gain_ = this->get_parameter("ema_gain").as_double();
  heartbeat_tolerance_ = this->get_parameter("heartbeat_tolerance").as_double();
  heartbeat_modules_ =
      this->get_parameter("heartbeat_modules").as_string_array();
  heartbeat_modules_accel_ =
      this->get_parameter("heartbeat_modules_accel").as_string_array();
  heartbeat_modules_inspection_ =
      this->get_parameter("heartbeat_modules_inspection").as_string_array();

  testing_ = this->get_parameter("testing").as_int();
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

  cone_detection_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ConeDetections>(
          topics::kConeDetections, 10,
          std::bind(&CarInterface::ConeDetectionCB, this, _1));

  cone_map_subscriber_ = this->create_subscription<utfr_msgs::msg::ConeMap>(
      topics::kConeMap, 10, std::bind(&CarInterface::ConeMapCB, this, _1));

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
          topics::kSystemStatus, 10);

  RCLCPP_INFO(this->get_logger(), "Finished Initializing Publishers");
}

void CarInterface::initTimers() {
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(this->update_rate_),
      std::bind(&CarInterface::timerCB, this));

  RCLCPP_INFO(this->get_logger(), "Finished Initializing Timer");
}

void CarInterface::initCAN() {
  can0_ = std::make_unique<CanInterface>();

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

void CarInterface::heartbeatCB(const utfr_msgs::msg::Heartbeat &msg) {
  heartbeat_monitor_->updateHeartbeat(msg, this->get_clock()->now());

  if (msg.module.data == "controller_node") {
    system_status_.lap_counter = msg.lap_count;
    if (msg.status == utfr_msgs::msg::Heartbeat::FINISH) {
      finished_ = true;
    }
  }
}

void CarInterface::controlCmdCB(const utfr_msgs::msg::ControlCmd &msg) {
  const std::string function_name{"controlCmdCB"};

  if (cmd_ || testing_) {
    braking_cmd_ = (int16_t)msg.brk_cmd;
    steering_cmd_ = (((int32_t)(msg.str_cmd * 45454)) & 0xFFFFFFFF);
    throttle_cmd_ = (int16_t)msg.thr_cmd;
  } else {
    braking_cmd_ = 0;
    steering_cmd_ = 0;
    throttle_cmd_ = 0;
  }

  // TODO: Map brake PWM to pressure?
  system_status_.brake_hydr_target = braking_cmd_; // TODO: Convert to %
  system_status_.motor_moment_target =
      (int)(100 * throttle_cmd_ / 200); // Converting to %
}

void CarInterface::EgoStateCB(const utfr_msgs::msg::EgoState &msg) {
  system_status_.speed_actual = msg.vel.twist.linear.x * 3.6;
  system_status_.acceleration_longitudinal = msg.accel.accel.linear.x;
  system_status_.acceleration_lateral = -msg.accel.accel.linear.y;
  system_status_.yaw_rate = -msg.vel.twist.angular.z;
}

void CarInterface::TargetStateCB(const utfr_msgs::msg::TargetState &msg) {
  system_status_.speed_target = msg.speed * 3.6;
  system_status_.steering_angle_target = -msg.steering_angle;
}

void CarInterface::ConeDetectionCB(const utfr_msgs::msg::ConeDetections &msg) {
  system_status_.cones_count_actual =
      msg.left_cones.size() + msg.right_cones.size() +
      msg.large_orange_cones.size() + msg.small_orange_cones.size() +
      msg.unknown_cones.size();
}

void CarInterface::ConeMapCB(const utfr_msgs::msg::ConeMap &msg) {
  system_status_.cones_count_all =
      msg.left_cones.size() + msg.right_cones.size() +
      msg.large_orange_cones.size() + msg.small_orange_cones.size() +
      msg.unknown_cones.size();
}

void CarInterface::sendDVLogs() {
  const std::string function_name{"sendDVLogs"};

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

    // Torques
    dv_driving_dynamics_1 |= system_status_.motor_moment_actual << (8 * (i++));
    dv_driving_dynamics_1 |= system_status_.motor_moment_target << (8 * (i++));

    can0_->write_can(dv_can_msg::DVDrivingDynamics1, dv_driving_dynamics_1,
                     true);

    // Dv driving dynamics 2
    long dv_driving_dynamics_2 = 0;
    i = 0;

    dv_driving_dynamics_2 |= system_status_.acceleration_longitudinal
                             << (16 * (i++));
    dv_driving_dynamics_2 |= system_status_.acceleration_lateral
                             << (16 * (i++));
    dv_driving_dynamics_2 |= system_status_.yaw_rate << (16 * (i++));

    can0_->write_can(dv_can_msg::DVDrivingDynamics2, dv_driving_dynamics_2,
                     true);

    // DV system status
    long dv_system_status = 0;
    i = 0;

    // TODO: Not reading the states from canbus at the moment
    dv_system_status |= (system_status_.as_state & 0x7);
    dv_system_status |= (system_status_.ebs_state & 0x3) << 3;
    dv_system_status |= (system_status_.ami_state & 0x7) << 5;
    dv_system_status |= (system_status_.steering_state & 0x1) << 8;
    dv_system_status |= (system_status_.service_brake_state & 0x3) << 9;
    dv_system_status |= (system_status_.lap_counter & 0x15) << 11;
    dv_system_status |= (system_status_.cones_count_actual & 0xFF) << 15;
    dv_system_status |= (system_status_.cones_count_all & 0x1FFFF) << 23;

    system_status_.header.stamp = this->get_clock()->now();
    system_status_publisher_->publish(system_status_);

    // can0_->write_can(dv_can_msg::DVSystemStatus, dv_system_status);

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::DVCompStateMachine() {
  const std::string function_name{"DVCompStateMachine"};

  try {
    bool heartbeat_status =
        heartbeat_monitor_->verifyHeartbeats(this->get_clock()->now());

    RCLCPP_INFO(this->get_logger(), "%s: Current state: %d",
                function_name.c_str(), system_status_.as_state);

    switch (system_status_.as_state) {
    case utfr_msgs::msg::SystemStatus::AS_STATE_OFF: {
      // Check if autonomous mission is set
      if (system_status_.ami_state ==
              std::clamp(system_status_.ami_state,
                         utfr_msgs::msg::SystemStatus::AMI_STATE_ACCELERATION,
                         utfr_msgs::msg::SystemStatus::AMI_STATE_AUTOCROSS) &&
          !launched_) {
        launched_ = launchMission(); // Launch other dv nodes
      } else if (heartbeat_status || str_motor_state_ > 0) {
        dv_pc_state_ = DV_PC_STATE::READY;
        cmd_ = false;
      } else {
        dv_pc_state_ = DV_PC_STATE::OFF;
        cmd_ = false;
      }
      break;
    }
    case utfr_msgs::msg::SystemStatus::AS_STATE_READY: {
      if (heartbeat_status || str_motor_state_ > 0) {
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
      if (heartbeat_status || str_motor_state_ > 0) {
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
        // shutdown_ = shutdownNodes();
      }
      break;
    }
    case utfr_msgs::msg::SystemStatus::AS_STATE_FINISH: {
      dv_pc_state_ = DV_PC_STATE::FINISH; // Should already be finish
      cmd_ = false;                       // Should already be false
      if (!shutdown_) {
        // shutdown_ = shutdownNodes();
      }
      break;
    }
    default: {
      dv_pc_state_ = DV_PC_STATE::OFF;
      cmd_ = false;
      if (!shutdown_) {
        // shutdown_ = shutdownNodes();
      }
      break;
    }
    }

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::sendStateAndCmd() {
  const std::string function_name{"sendStateAndCmd"};

  try {

    // DV computer state
    uint64_t dv_comp_state = 0;
    // Todo: need to be tested. 
    dv_comp_state = can0_->setSignal(dv_comp_state, 0, 3, 1, dv_pc_state_);
    // dv_comp_state = can0_->setSignal(dv_comp_state, 0, 3, 1, 2);

    // uint64_t position = steering_cmd_;
    uint64_t steering_position = 30; // TEMP FOR TESTING
    
    if (dv_pc_state_ == DV_PC_STATE::READY) {
      // Origin inspection.
      steering_position = 0; 
    }
    // Steering motor position
    // can use different mode to command speed/accel
    // can0_->write_can(dv_can_msg::SetSTRMotorPos, ((long)steering_cmd_) << 32,
    // true);

    uint64_t steering_canfd = 0;
    // Extended CAN
    // Speed0: Start Bit = 24, Length = 8
    // Set SCALE TO 0 for INITIAL CAN TESTING
    steering_canfd =
        can0_->setSignal(steering_canfd, 24, 8, 0.0001, 0x000000FF & steering_position);
    steering_canfd = can0_->setSignal(steering_canfd, 16, 8, 0.0001,
                                      (0x0000FF00 & steering_position) >> 8);
    steering_canfd = can0_->setSignal(steering_canfd, 8, 8, 0.0001,
                                      (0x00FF0000 & steering_position) >> 16);
    steering_canfd = can0_->setSignal(steering_canfd, 0, 8, 0.0001,
                                      (0xFF000000 & steering_position) >> 24);

    can0_->write_can(dv_can_msg::STR_MOTOR_CMD, steering_canfd, true);

    // Motor/inverter command
    uint64_t inverter_canfd = 0;

    /*
    if (braking_cmd_ == 0) {
      // Zero commanded torque
      inverter_canfd = can0_->setSignal(inverter_canfd, 0, 16, 1, 0x0000);
      // Commanded speed
      inverter_canfd = can0_->setSignal(inverter_canfd, 16, 16, 1, throttle_cmd_
    & 0xFFFF); } else if (braking_cmd_ < 0) {
      // TODO: review regen
      // Commanded negative torque
      inverter_canfd = can0_->setSignal(inverter_canfd, 0, 16, 1, braking_cmd_ &
    0xFFFF);
      // Zero commanded speed
      inverter_canfd = can0_->setSignal(inverter_canfd, 16, 16, 1, 0x0000);
    }

    if (cmd_ || testing_) {
      // Enable Inverter
      inverter_canfd = can0_->setSignal(inverter_canfd, 40, 8, 1, 0x01);
    } else {
      // Disable Inverter
      inverter_canfd = can0_->setSignal(inverter_canfd, 40, 8, 1, 0x00);
    }

    // Forward direction.
    inverter_canfd = can0_->setSignal(inverter_canfd, 32, 8, 1, 0x01);
    // Torque limit.
    inverter_canfd = can0_->setSignal(inverter_canfd, 48, 16, 1, 0x0000);
    */

    int torque_commanded = 10;
    int speed_commanded = 0;

    inverter_canfd =
        can0_->setSignal(inverter_canfd, 0, 8, 1, torque_commanded % 256);
    inverter_canfd =
        can0_->setSignal(inverter_canfd, 8, 8, 1, torque_commanded / 256);
    can0_->write_can(dv_can_msg::DV_COMMANDED, inverter_canfd, true);

    inverter_canfd =
        can0_->setSignal(inverter_canfd, 16, 8, 1, speed_commanded % 256);
    inverter_canfd =
        can0_->setSignal(inverter_canfd, 24, 8, 1, speed_commanded / 256);
    /*
    inverter_canfd = can0_->setSignal(inverter_canfd, 32, 8, 1, 1);
    inverter_canfd = can0_->setSignal(inverter_canfd, 40, 8, 1, (1 | 4));
    inverter_canfd = can0_->setSignal(inverter_canfd, 48, 8, 1, 0);
    inverter_canfd = can0_->setSignal(inverter_canfd, 56, 8, 1, 0);
    */

    // Transmit
    can0_->write_can(dv_can_msg::DV_COMP_STATE, dv_comp_state, false);
    // can0_->write_can(dv_can_msg::COMMANDED_TORQUE, inverter_canfd, true);

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

bool CarInterface::launchMission() {
  const std::string function_name{"launchMission"};

  std::string launchCmd;
  std::vector<std::string> modules;

  switch (system_status_.ami_state) {
  case utfr_msgs::msg::SystemStatus::AMI_STATE_INSPECTION:
    RCLCPP_INFO(this->get_logger(), "%s: Launching Inspection mission",
                function_name.c_str());
    launchCmd = "ros2 launch launcher dv_inspection.launch.py";
    modules = heartbeat_modules_inspection_;
    break;

  case utfr_msgs::msg::SystemStatus::AMI_STATE_BRAKETEST:
  case utfr_msgs::msg::SystemStatus::AMI_STATE_ACCELERATION:
    RCLCPP_INFO(this->get_logger(), "%s: Launching Accel/EBS test mission",
                function_name.c_str());
    launchCmd = "ros2 launch launcher dv_accel.launch.py";
    modules = heartbeat_modules_accel_;
    break;

  default:
    RCLCPP_INFO(this->get_logger(), "%s: Launching DV mission",
                function_name.c_str());
    launchCmd = "ros2 launch launcher dv.launch.py";
    modules = heartbeat_modules_;
    break;
  }
  // Execute the launch command
  int result = std::system(launchCmd.c_str());

  heartbeat_monitor_->updateModules(modules, this->get_clock()->now());

  if (result != 0) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occurred",
                 function_name.c_str());
    return false;
  } else {
    return true;
  }
}

bool CarInterface::shutdownNodes() {
  const std::string function_name{"shutdownNodes"};
  try {
    RCLCPP_INFO(this->get_logger(), "%s: Shutting down car_interface node",
                function_name.c_str());
    rclcpp::shutdown();
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occurred",
                 function_name.c_str());
    return false;
  }
  return true;
}

void CarInterface::timerCB() {
  const std::string function_name{"timerCB"};

  try {
    // getSensorCan(); // Publish sensor and state data that is read from can
    getDVState(); // Read DV state from car from can
    // sendDVLogs(); // Publish FSG log format over ros and send over can
    DVCompStateMachine(); // Set DV coputer state
    sendStateAndCmd();    // Send DV computer state to RC and actuator commands
                          // over can
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

} // namespace car_interface
} // namespace utfr_dv