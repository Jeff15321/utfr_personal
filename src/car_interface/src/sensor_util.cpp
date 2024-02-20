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

#include <sensor_util.hpp>

namespace utfr_dv {
namespace car_interface {

void CarInterface::getSteeringAngleSensorData() {
  const std::string function_name{"getSteeringAngleSensorData"};
  int16_t steering_angle; // TODO: Check proper var type

  try {
    // TODO: Check value format
    steering_angle =
        (uint16_t)((int16_t)(can1_->get_can(dv_can_msg::StrMotorStatus)) / 10) /
        4.5;
    RCLCPP_INFO(this->get_logger(), "%s: Steer angle: %d",
                function_name.c_str(), steering_angle);
    // Check for sensor malfunction
    if ((abs(steering_angle) > 50)) {
      RCLCPP_ERROR(this->get_logger(), "%s: Value error",
                   function_name.c_str());
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
    // TODO: Proper CAN message
    // wheelspeed_fl
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TODO: Proper CAN message
    // wheelspeed_fr
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TODO: Proper CAN message
    // wheelspeed_rl
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));
    // TODO: Proper CAN message
    // wheelspeed_rr
    //     (uint16_t)(can1_->get_can(dv_can_msg::TODO));

    sensor_can_.wheelspeed_fl = wheelspeed_fl;
    sensor_can_.wheelspeed_fr = wheelspeed_fr;
    sensor_can_.wheelspeed_rl = wheelspeed_rl;
    sensor_can_.wheelspeed_rr = wheelspeed_rr;
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
  // RCLCPP_INFO(this->get_logger(), "AS_STATE: %d", system_status_.as_state);
  system_status_.ebs_state = (dv_state << 3) & 0x3;

  system_status_.ami_state = (dv_state << 5) & 0x7;

  system_status_.steering_state = (dv_state << 8) & 0x1;

  system_status_.service_brake_state = (dv_state << 9) & 0x3;
}

} // namespace car_interface
} // namespace utfr_dv