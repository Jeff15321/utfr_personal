/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: sensor_util.cpp
* auth: Daniel Asadi
* desc: Sensor data parsing over canbus
*/

#include <car_interface_node.hpp>

namespace utfr_dv {
namespace car_interface {

void CarInterface::getSteeringAngleSensorData() {
  const std::string function_name{"getSteeringAngleSensorData"};
  int16_t steering_angle; // TODO: Check proper var type

  try {
    // TODO: Check value format
    steering_angle =
        (uint16_t)((int16_t)(can1_->get_can(dv_can_msg::StrMotorStatus)) / 10);
    // RCLCPP_INFO(this->get_logger(), "%s: Steer angle: %d",
    //             function_name.c_str(), steering_angle);
    // Check for sensor malfunction
    if ((abs(steering_angle) > 50)) {
      RCLCPP_ERROR(this->get_logger(), "%s: Value error",
                   function_name.c_str());
    } else {
      // TODO: Check frame
      sensor_can_.steering_angle = steering_angle * 0.22;
      system_status_.steering_angle_actual = -sensor_can_.steering_angle;
    }
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getMotorSpeedData() {
  const std::string function_name{"getMotorSpeedData"};
  double motor_speed; // TODO: Check proper var type

  try {
    // TODO: Check value format
    motor_speed = can1_->get_can(dv_can_msg::MOTPOS) * -0.021545;

    sensor_can_.motor_speed = motor_speed;
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

    system_status_.motor_moment_actual =
        (int)(100 * motor_torque / MAX_THROTTLE); // Converting to %
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getServiceBrakeData() {
  const std::string function_name{"getServiceBrakeData"};
  uint16_t front_pressure;
  uint16_t rear_pressure;

  try {
    // TODO: Check value format
    front_pressure = (can1_->get_can(dv_can_msg::FBP));
    rear_pressure = (can1_->get_can(dv_can_msg::RBP)) & 0xFFFF;
    //    RCLCPP_INFO(this->get_logger(), "rear brake pressure:
    //    %d",rear_pressure);

    sensor_can_.rear_pressure = front_pressure;
    sensor_can_.front_pressure = rear_pressure;

    system_status_.brake_hydr_actual =
        (int)(100 * rear_pressure / MAX_BRK_PRS); // Converting to %
    //    RCLCPP_INFO(this->get_logger(), "rear brake pressure:
    //    %d",rear_pressure);

    sensor_can_.rear_pressure = front_pressure;
    sensor_can_.front_pressure = rear_pressure;

    system_status_.brake_hydr_actual =
        (int)(100 * rear_pressure / MAX_BRK_PRS); // Converting to %

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getWheelspeedSensorData() {
  const std::string function_name{"getWheelspeedSensorData"};
  double wheelspeed_fl = 0; // TODO: Check proper var type
  double wheelspeed_fr = 0; // TODO: Check proper var type
  double wheelspeed_rl = 0; // TODO: Check proper var type
  double wheelspeed_rr = 0; // TODO: Check proper var type

  try {
    // Wheel speeds in total ticks, 14 ticks per revolution
    wheelspeed_fl = 
        (uint16_t)(can1_->get_can(dv_can_msg::SPEEDFL));
    wheelspeed_fr = 
        (uint16_t)(can1_->get_can(dv_can_msg::SPEEDFR));
    wheelspeed_rl = 
        (uint16_t)(can1_->get_can(dv_can_msg::SPEEDRL));
    wheelspeed_rr = 
        (uint16_t)(can1_->get_can(dv_can_msg::SPEEDRR));

    // Calculate time step between now and last count
    double dt = this->get_clock()->now().nanoseconds() / (1.0 * 1e9) - sensor_can_.header.stamp.nanosec / (1.0 * 1e9);

    // Calculate how many new ticks
    int delta_fl = wheelspeed_fl - prev_wheelspeed_fl_;
    int delta_fr = wheelspeed_fr - prev_wheelspeed_fr_;
    int delta_rl = wheelspeed_rl - prev_wheelspeed_rl_;
    int delta_rr = wheelspeed_rr - prev_wheelspeed_rr_;

    // Update previous wheelspeed
    prev_wheelspeed_fl_ = wheelspeed_fl;
    prev_wheelspeed_fr_ = wheelspeed_fr;
    prev_wheelspeed_rl_ = wheelspeed_rl;
    prev_wheelspeed_rr_ = wheelspeed_rr;

    // Calculate RPM
    double rpm_fl = delta_fl / 14.0 / dt * 60;
    double rpm_fr = delta_fr / 14.0 / dt * 60;
    double rpm_rl = delta_rl / 14.0 / dt * 60;
    double rpm_rr = delta_rr / 14.0 / dt * 60;

    // Apply EMA filter
    double filtered_rpm_fl = ema_gain_ * rpm_fl + (1 - ema_gain_) * ema_prev_fl_;
    double filtered_rpm_fr = ema_gain_ * rpm_fr + (1 - ema_gain_) * ema_prev_fr_;
    double filtered_rpm_rl = ema_gain_ * rpm_rl + (1 - ema_gain_) * ema_prev_rl_;
    double filtered_rpm_rr = ema_gain_ * rpm_rr + (1 - ema_gain_) * ema_prev_rr_;
    // Update previous filtered values
    ema_prev_fl_ = filtered_rpm_fl;
    ema_prev_fr_ = filtered_rpm_fr;
    ema_prev_rl_ = filtered_rpm_rl;
    ema_prev_rr_ = filtered_rpm_rr;

    // Update sensor_can_ values
    sensor_can_.wheelspeed_fl = filtered_rpm_fl;
    sensor_can_.wheelspeed_fr = filtered_rpm_fr;
    sensor_can_.wheelspeed_rl = filtered_rpm_rl;
    sensor_can_.wheelspeed_rr = filtered_rpm_rr;
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
        (double)(((long)can1_->get_can(dv_can_msg::ImuX) >> (32)) & 255) / 100;
    imu.linear_acceleration.y =
        -(double)(((long)can1_->get_can(dv_can_msg::ImuY) >> (32)) & 255) / 100;
    imu.linear_acceleration.z =
        (double)(((long)can1_->get_can(dv_can_msg::ImuZ) >> (32)) & 255) / 100;
    imu.angular_velocity.x =
        (double)(can1_->get_can(dv_can_msg::ImuX) & 255) / 10;
    imu.angular_velocity.y =
        (double)(can1_->get_can(dv_can_msg::ImuY) & 255) / 10;

    sensor_can_.imu_data = imu;
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getGPSData() {
  const std::string function_name{"getGPSData"};

  try {
    std_msgs::msg::Bool gps_error; 
    gps_error.set__data(
      (bool) can1_->getSignalBE(
        dv_can_msg::GPS_ERROR_CODE, 0, 8, false, 1
      )
    );

    sensor_msgs::msg::NavSatFix latlong;
    // Latitude, Longitude, Altitude 
    latlong.latitude = (int32_t) can1_->getSignalBE(
      dv_can_msg::GPS_LAT_LONG, 0, 32, true, pow(2, -24)
    );
    latlong.longitude = (int32_t) can1_->getSignalBE(
      dv_can_msg::GPS_LAT_LONG, 32, 32, true, pow(2, -23)
    );
    latlong.altitude = (uint32_t) can1_->getSignalBE(
      dv_can_msg::GPS_ALT_ELLIP, 0, 32, false, pow(2, -15)
    ); 

    // Euler Angles ROS msg: Vector3Stamped
    // stamped = time stamped all msgs need time stamp 

    geometry_msgs::msg::QuaternionStamped gps_orientation; 
    // Quaternion Angles (need to change firmware to quaternion angles, euler angles can msg is misleading);
    double gps_quat_scale = pow(2, -7);
    gps_orientation.quaternion.x = 
      (int16_t) can1_->getSignalBE(dv_can_msg::GPS_ORIENTATION, 0, 16, true, gps_quat_scale);
    gps_orientation.quaternion.y =
      (int16_t) can1_->getSignalBE(dv_can_msg::GPS_ORIENTATION, 16, 16, true, gps_quat_scale);
    gps_orientation.quaternion.w = 
      (int16_t) can1_->getSignalBE(dv_can_msg::GPS_ORIENTATION, 32, 16, true, gps_quat_scale); 
    gps_orientation.quaternion.z =
      (int16_t) can1_->getSignalBE(dv_can_msg::GPS_ORIENTATION, 48, 16, true, gps_quat_scale);
    
    geometry_msgs::msg::TwistStamped gps_velocity; 
    double gps_vel_scale = pow(2, -6); 
    gps_velocity.twist.linear.x = (int16_t) can1_->getSignalBE(
      dv_can_msg::GPS_VEL_XYZ, 0, 16, true, gps_vel_scale
    ); 
    gps_velocity.twist.linear.y = (int16_t) can1_->getSignalBE(
      dv_can_msg::GPS_VEL_XYZ, 16, 16, true, gps_vel_scale
    ); 
    gps_velocity.twist.linear.z = (int16_t) can1_->getSignalBE(
      dv_can_msg::GPS_VEL_XYZ, 32, 16, true, gps_vel_scale
    );

    geometry_msgs::msg::Accel gps_accel; 
    double gps_accel_scale = pow(2, -8); 
    gps_accel.linear.x = (int16_t) can1_->getSignalBE(
      dv_can_msg::GPS_ACCELERATION, 0, 16, true, gps_accel_scale
    ); 
    gps_accel.linear.y = (int16_t) can1_->getSignalBE(
      dv_can_msg::GPS_ACCELERATION, 16, 16, true, gps_accel_scale
    );
    gps_accel.linear.z = (int16_t) can1_->getSignalBE( 
      dv_can_msg::GPS_ACCELERATION, 32, 16, true, gps_accel_scale
    );

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occurred, error #%d", 
      function_name.c_str(), e);
  }
}

void CarInterface::getTorque() {
  const std::string function_name{"getTorque"};

  try {
    // ROS message uint
    // get torque inverter -> dv computer 
    // system status msgs <- check 
    system_status_.motor_moment_actual = (int16_t) can1_->getSignal(
      dv_can_msg::ACTUAL_TORQUE, 16, 16, false, 1
    );
    // check speed mode torque commanded docs

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occurred, error #%d", 
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
    getGPSData();

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
  // RCLCPP_INFO(this->get_logger(), "AS_STATE: %d", system_status_.as_state);
  system_status_.ebs_state = (dv_state << 3) & 0x3;

  system_status_.ami_state = (dv_state << 5) & 0x7;

  system_status_.steering_state = (dv_state << 8) & 0x1;

  system_status_.service_brake_state = (dv_state << 9) & 0x3;
}

} // namespace car_interface
} // namespace utfr_dv