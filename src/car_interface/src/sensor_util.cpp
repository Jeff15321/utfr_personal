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

void CarInterface::getSteeringMotorData() { // TODO: Review
  const std::string function_name{"getSteeringMotorData"};
  int16_t steering_angle; // TODO: Check proper var type

  try {
    // servo mode
    str_motor_state_ =
        (uint8_t)can0_->getSignal(dv_can_msg::StrMotorInfo, 56, 8, false, 1);
    // TODO: figure out whether getSignal can be used with high byte / low byte
    // format
    steering_angle = // degrees
        -(uint16_t)can0_->getSignal(dv_can_msg::StrMotorInfo, 0, 16, true, 0.1);

    // RCLCPP_INFO(this->get_logger(), "%s: Steer angle: %d",
    //             function_name.c_str(), steering_angle);
    // Check for sensor malfunction
    RCLCPP_WARN(this->get_logger(), "Steering Motor Angle: %d", steering_angle);
    if ((abs(steering_angle) > 50)) {
      RCLCPP_ERROR(this->get_logger(), "%s: Value error",
                   function_name.c_str());
    } else {
      // TODO: Check frame
      sensor_can_.steering_angle =
          steering_angle * 0.22; // convert motor angle to steering angle
      system_status_.steering_angle_actual =
          -sensor_can_.steering_angle; // FSG logging ref frame
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
    // Motor Speed 1
    motor_speed = can0_->getSignal(dv_can_msg::MOTPOS, 16, 16, true, 0.1);

    sensor_can_.motor_speed = motor_speed;
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getMotorTorqueData() {
  const std::string function_name{"getMotorTorqueData"};

  try {
    system_status_.motor_moment_actual = (int16_t)can0_->getSignal(
        dv_can_msg::ACTUAL_TORQUE, 16, 16, false, 0.1);
    system_status_.motor_moment_target = (int16_t)can0_->getSignal(
        dv_can_msg::COMMANDED_TORQUE, 0, 16, true, 0.1);
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getServiceBrakeData() { // TODO: Review
  const std::string function_name{"getServiceBrakeData"};
  uint16_t front_pressure;
  uint16_t rear_pressure;

  try {
    // TODO: Check value format
    front_pressure = (can0_->get_can(dv_can_msg::FBP));
    rear_pressure = (can0_->get_can(dv_can_msg::RBP)) & 0xFFFF;

    sensor_can_.rear_pressure = (int)static_cast<double>(rear_pressure) *
                                (1600.0 / 65535.0); // Convert to psi
    sensor_can_.front_pressure = (int)static_cast<double>(front_pressure) *
                                 (1600.0 / 65535.0); // Convert to psi

    system_status_.brake_hydr_actual =
        (int)(100 * rear_pressure / MAX_BRK_PRS); // Converting to %

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getWheelspeedSensorData() { // Not needed atm
  const std::string function_name{"getWheelspeedSensorData"};
  double wheelspeed_fl = 0; // TODO: Check proper var type
  double wheelspeed_fr = 0; // TODO: Check proper var type
  double wheelspeed_rl = 0; // TODO: Check proper var type
  double wheelspeed_rr = 0; // TODO: Check proper var type

  try {
    // Wheel speeds in total ticks, 14 ticks per revolution
    wheelspeed_fl = (uint16_t)(can0_->get_can(dv_can_msg::SPEEDFL));
    wheelspeed_fr = (uint16_t)(can0_->get_can(dv_can_msg::SPEEDFR));
    wheelspeed_rl = (uint16_t)(can0_->get_can(dv_can_msg::SPEEDRL));
    wheelspeed_rr = (uint16_t)(can0_->get_can(dv_can_msg::SPEEDRR));

    // Calculate time step between now and last count
    double dt = this->get_clock()->now().nanoseconds() / (1.0 * 1e9) -
                sensor_can_.header.stamp.nanosec / (1.0 * 1e9);

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
    double filtered_rpm_fl =
        ema_gain_ * rpm_fl + (1 - ema_gain_) * ema_prev_fl_;
    double filtered_rpm_fr =
        ema_gain_ * rpm_fr + (1 - ema_gain_) * ema_prev_fr_;
    double filtered_rpm_rl =
        ema_gain_ * rpm_rl + (1 - ema_gain_) * ema_prev_rl_;
    double filtered_rpm_rr =
        ema_gain_ * rpm_rr + (1 - ema_gain_) * ema_prev_rr_;
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

void CarInterface::getIMUData() { // Not needed atm
  const std::string function_name{"getIMUData"};

  try {
    // TODO: Check reference frames, double check value format
    sensor_msgs::msg::Imu imu;
    imu.linear_acceleration.x =
        (double)(((long)can0_->get_can(dv_can_msg::ImuX) >> (32)) & 255) / 100;
    imu.linear_acceleration.y =
        -(double)(((long)can0_->get_can(dv_can_msg::ImuY) >> (32)) & 255) / 100;
    imu.linear_acceleration.z =
        (double)(((long)can0_->get_can(dv_can_msg::ImuZ) >> (32)) & 255) / 100;
    imu.angular_velocity.x =
        (double)(can0_->get_can(dv_can_msg::ImuX) & 255) / 10;
    imu.angular_velocity.y =
        (double)(can0_->get_can(dv_can_msg::ImuY) & 255) / 10;

    sensor_can_.imu_data = imu;
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getGPSData() {
  const std::string function_name{"getGPSData"};

  try {
    std_msgs::msg::Header gps_header;
    gps_header.stamp.sec = (int32_t)can0_->getSignalBE(
        dv_can_msg::GPS_SAMPLE_TIME, 0, 32, false, 1);
    sensor_can_.set__gps_header(gps_header);

    std_msgs::msg::Bool gps_error;
    gps_error.set__data(
        (bool)can0_->getSignalBE(dv_can_msg::GPS_ERROR_CODE, 0, 8, false, 1));
    sensor_can_.set__gps_error(gps_error); 

    sensor_msgs::msg::NavSatFix latlong;
    // Latitude, Longitude, Altitude
    latlong.latitude = (int32_t)can0_->getSignalBE(dv_can_msg::GPS_LAT_LONG, 0,
                                                   32, true, pow(2, -24));
    latlong.longitude = (int32_t)can0_->getSignalBE(dv_can_msg::GPS_LAT_LONG,
                                                    32, 32, true, pow(2, -23));
    latlong.altitude = (uint32_t)can0_->getSignalBE(dv_can_msg::GPS_ALT_ELLIP,
                                                    0, 32, false, pow(2, -15));
    sensor_can_.set__gps_gnss(latlong); 

    // stamped = time stamped all msgs need time stamp
    geometry_msgs::msg::Vector3Stamped gps_rpy;
    gps_rpy.header = gps_header;
    double gps_rpy_scale = pow(2, -7);
    gps_rpy.vector.x = (int16_t)can0_->getSignalBE(dv_can_msg::GPS_RPY, 0, 16,
                                                   true, gps_rpy_scale);
    RCLCPP_ERROR(this->get_logger(), "%s: ROLL: %f",function_name.c_str(), can0_->getSignalBE(dv_can_msg::GPS_RPY, 0, 16,
                                                   true, gps_rpy_scale));
    gps_rpy.vector.y = (int16_t)can0_->getSignalBE(dv_can_msg::GPS_RPY, 16, 16,
                                                   true, gps_rpy_scale);
    RCLCPP_ERROR(this->get_logger(), "%s: PITCH: %f",function_name.c_str(), can0_->getSignalBE(dv_can_msg::GPS_RPY, 16, 16,
                                                   true, gps_rpy_scale));
    gps_rpy.vector.z = (int16_t)can0_->getSignalBE(dv_can_msg::GPS_RPY, 32, 16,
                                                   true, gps_rpy_scale);
    RCLCPP_ERROR(this->get_logger(), "%s: YAW: %f",function_name.c_str(), can0_->getSignalBE(dv_can_msg::GPS_RPY, 32, 16,
                                                   true, gps_rpy_scale));
    sensor_can_.set__gps_rpy(gps_rpy);

    geometry_msgs::msg::QuaternionStamped gps_orientation;
    gps_orientation.header = gps_header;
    double gps_quat_scale = pow((pow(2, 15) - 1), -1);
    gps_orientation.quaternion.x = (int16_t)can0_->getSignalBE(
        dv_can_msg::GPS_ORIENTATION, 0, 16, true, gps_quat_scale);
    gps_orientation.quaternion.y = (int16_t)can0_->getSignalBE(
        dv_can_msg::GPS_ORIENTATION, 16, 16, true, gps_quat_scale);
    gps_orientation.quaternion.w = (int16_t)can0_->getSignalBE(
        dv_can_msg::GPS_ORIENTATION, 32, 16, true, gps_quat_scale);
    gps_orientation.quaternion.z = (int16_t)can0_->getSignalBE(
        dv_can_msg::GPS_ORIENTATION, 48, 16, true, gps_quat_scale);
    sensor_can_.set__gps_orientation(gps_orientation); 

    geometry_msgs::msg::TwistStamped gps_velocity;
    gps_velocity.header = gps_header;
    double gps_vel_scale = pow(2, -6);
    gps_velocity.twist.linear.x = (int16_t)can0_->getSignalBE(
        dv_can_msg::GPS_VEL_XYZ, 0, 16, true, gps_vel_scale);
    gps_velocity.twist.linear.y = (int16_t)can0_->getSignalBE(
        dv_can_msg::GPS_VEL_XYZ, 16, 16, true, gps_vel_scale);
    gps_velocity.twist.linear.z = (int16_t)can0_->getSignalBE(
        dv_can_msg::GPS_VEL_XYZ, 32, 16, true, gps_vel_scale);
    sensor_can_.set__gps_velocity(gps_velocity);

    geometry_msgs::msg::Accel gps_accel;
    double gps_accel_scale = pow(2, -8);
    gps_accel.linear.x = (int16_t)can0_->getSignalBE(
        dv_can_msg::GPS_ACCELERATION, 0, 16, true, gps_accel_scale);
    gps_accel.linear.y = (int16_t)can0_->getSignalBE(
        dv_can_msg::GPS_ACCELERATION, 16, 16, true, gps_accel_scale);
    gps_accel.linear.z = (int16_t)can0_->getSignalBE(
        dv_can_msg::GPS_ACCELERATION, 32, 16, true, gps_accel_scale);
    sensor_can_.set__gps_accel(gps_accel);

  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occurred, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getSensorCan() {
  const std::string function_name{"getSensorCan"};

  try {
    // Read sensor CAN messages from car
    getSteeringMotorData();
    getMotorSpeedData();
    getMotorTorqueData();
    getServiceBrakeData();
    // getWheelspeedSensorData();
    // getIMUData();
    getGPSData();

    sensor_can_.header.stamp = this->get_clock()->now();
    sensor_can_publisher_->publish(sensor_can_);
  } catch (int e) {
    RCLCPP_ERROR(this->get_logger(), "%s: Error occured, error #%d",
                 function_name.c_str(), e);
  }
}

void CarInterface::getDVState() {
  const std::string function_name{"getDVState"};

  // Get DV state from car
  system_status_.as_state =
      can0_->getSignal(dv_can_msg::FULL_AS_STATE, 0, 3, true, 1);
  RCLCPP_ERROR(this->get_logger(), "%s: AS STATE: %d",function_name.c_str(), system_status_.as_state);
  system_status_.ebs_state =
      can0_->getSignal(dv_can_msg::FULL_AS_STATE, 3, 2, true, 1);
  RCLCPP_ERROR(this->get_logger(), "%s: EBS STATE: %d",function_name.c_str(), system_status_.ebs_state);
  system_status_.ami_state =
      can0_->getSignal(dv_can_msg::FULL_AS_STATE, 5, 3, true, 1);
  RCLCPP_ERROR(this->get_logger(), "%s: AMI STATE: %d",function_name.c_str(), system_status_.ami_state);
  system_status_.steering_state = (str_motor_state_ == 0) ? 1 : 0;
  RCLCPP_ERROR(this->get_logger(), "%s: STEERING STATE: %d",function_name.c_str(), system_status_.steering_state);
  system_status_.service_brake_state =
      can0_->getSignal(dv_can_msg::FULL_AS_STATE, 9, 2, true, 1);
  RCLCPP_ERROR(this->get_logger(), "%s: BRAKE STATE: %d",function_name.c_str(), system_status_.service_brake_state);
}

} // namespace car_interface
} // namespace utfr_dv