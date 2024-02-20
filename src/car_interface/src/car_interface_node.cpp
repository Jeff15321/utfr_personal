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
  this->declare_parameter("heartbeat_tolerance", 1.5);
  this->declare_parameter("heartbeat_modules", default_modules);
  this->declare_parameter("testing", 0);

  update_rate_ = this->get_parameter("update_rate").as_double();
  heartbeat_tolerance_ = this->get_parameter("heartbeat_tolerance").as_double();
  heartbeat_modules_ =
      this->get_parameter("heartbeat_modules").as_string_array();
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

  if (can1_->connect("can1")) {
    RCLCPP_INFO(this->get_logger(), "Finished Initializing CAN");
  } else
    RCLCPP_ERROR(this->get_logger(), "Failed To Initialize CAN");

  while (can1_->read_can())
    ;

  return;
}

void CarInterface::initMonitor() {
  heartbeat_monitor_ = std::make_unique<HeartbeatMonitor>(
      heartbeat_modules_, this->get_clock()->now(), heartbeat_tolerance_);
}

void CarInterface::heartbeatCB(const utfr_msgs::msg::Heartbeat &msg) {
  heartbeat_monitor_->updateHeartbeat(msg, this->get_clock()->now());

  if (msg.module.data = "controller_node") {
    system_status_.lap_counter = msg.lap_count;
    if (msg.status == utfr_msgs::msg::Heartbeat::FINISH) {
      finished_ = true;
    }
  }
}

void CarInterface::controlCmdCB(const utfr_msgs::msg::ControlCmd &msg) {
  const std::string function_name{"controlCmdCB"};

  steering_cmd_ = msg.str_cmd;
  // Clamp steering angle to +/- MAX_STR
  std::clamp(steering_cmd_, MAX_STR, -MAX_STR);

  // Finalize commands

  if (cmd_ || testing_) {
    braking_cmd_ = (int16_t)msg.brk_cmd;
    steering_cmd_ = (((int32_t)(msg.str_cmd * 45000)) &
                     0xFFFFFFFF); // *4.5 for motor to wheels angle
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
      dv_pc_state_ = DV_PC_STATE::OFF;
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

    if (cmd_ || testing_) {
      dv_command |= (long)(dv_pc_state_)&7;
      dv_command |= (long)(throttle_cmd_ & 0xFFFF) << 3;
      dv_command |= (long)(braking_cmd_ & 0xFF) << 19;

      // Write to steering motor
      // RCLCPP_INFO(this->get_logger(), "Steer: %d", steering_cmd_);
      can1_->write_can(
          dv_can_msg::SetMotorPos,
          ((long)steering_cmd_)
              << 32); // can use different mode to command speed/accel
    }
    // Need to always send dv command so RC always knows DC status
    can1_->write_can(dv_can_msg::DV_COMMAND, dv_command);

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::launchMission() {
  const std::string function_name{"launchMission"};

  std::string launchCmd = "ros2 launch launcher dv.launch.py";
  // Execute the launch command
  int result = std::system(launchCmd.c_str());

  if (result != 0) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occurred",
                 function_name.c_str());
  }
}

void CarInterface::shutdownNodes() {
  const std::string function_name{"shutdownNodes"};

  // // Retrieve the list of active node names, including rosbag recording
  // auto node_names = rclcpp::Node::get_node_names();

  // for (const auto &name : node_names) {
  //   if (name != "car_interface") {
  //     // Create a node to shutdown the active node
  //     auto node = std::make_shared<rclcpp::Node>(name);

  //     // Shutdown the active node
  //     node->rclcpp::shutdown();
  //     RCLCPP_INFO(this->get_logger(), "%s: Shutting down node: %s",
  //                 function_name.c_str(), name.c_str());
  //   }
  // }

  // // Shutdown the car_interface node
  // rclcpp::shutdown();
  // RCLCPP_INFO(this->get_logger(), "%s: Shutting down car_interface node",
  //             function_name.c_str());
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
