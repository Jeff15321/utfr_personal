/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: ekf_node.cpp
* auth: Arthur Xu
* desc: ekf node class
*/

#include <cmath>
#include <ekf_node.hpp>
#include <vector>

namespace utfr_dv {
namespace ekf {

EkfNode::EkfNode() : Node("ekf_node") {
  this->initParams();
  this->initHeartbeat();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::NOT_READY);
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
}

void EkfNode::initParams() {
  this->declare_parameter("update_rate", 33.33);
  update_rate_ = this->get_parameter("update_rate").as_double();
  this->declare_parameter("mapping_mode", 0);
  mapping_mode_ = this->get_parameter("mapping_mode").as_int();
  this->declare_parameter("ekf_on", 1);
  ekf_on_ = this->get_parameter("ekf_on").as_int();

  P_ = 1 * Eigen::MatrixXd::Identity(6, 6);
  prev_time_ = this->now();
  current_state_.pose.pose.position.x = 0.0;
  current_state_.pose.pose.position.y = 0.0;
  datum_lla = geometry_msgs::msg::Vector3();
  vehicle_params_ = VehicleParameters();

  last_gps_ = {0.0, 0.0};
  datum_yaw_ = NULL;
}

void EkfNode::initSubscribers() {

  if (mapping_mode_ != 2 && ekf_on_ == 1) {
    sensorcan_subscriber_ =
        this->create_subscription<utfr_msgs::msg::SensorCan>(
            topics::kSensorCan, 1,
            std::bind(&EkfNode::sensorCB, this, std::placeholders::_1));

    /* gps_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ground_truth/odom", 1,
        std::bind(&EkfNode::gpsCB, this, std::placeholders::_1));

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 1,
        std::bind(&EkfNode::imuCB, this, std::placeholders::_1));
    */
  }
}

void EkfNode::initPublishers() {
  if (mapping_mode_ != 2 && ekf_on_ == 1) {
    ego_state_publisher_ =
        this->create_publisher<utfr_msgs::msg::EgoState>(topics::kEgoState, 10);
  }

  state_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
      "IM_GONNA_KMS", 10);
}

void EkfNode::initTimers() {
  if (mapping_mode_ != 2 && ekf_on_ == 1) {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(this->update_rate_),
        std::bind(&EkfNode::timerCB, this));
  }
}

void EkfNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kEKFHeartbeat, 10);
  heartbeat_.module.data = "ekf";
  heartbeat_.update_rate = update_rate_;
}

void EkfNode::publishHeartbeat(const int status) {
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_publisher_->publish(heartbeat_);
}

void EkfNode::sensorCB(const utfr_msgs::msg::SensorCan msg) {
  double gps_x = msg.position.latitude;
  double gps_y = msg.position.longitude;

  if (datum_lla.x == 0.0 && datum_lla.y == 0.0 && datum_lla.z == 0.0) {
    datum_lla = geometry_msgs::msg::Vector3();
    datum_lla.x = gps_x;
    datum_lla.y = gps_y;
    datum_lla.z = msg.position.altitude;
  }

  if (datum_yaw_ == NULL) {
    datum_yaw_ = utfr_dv::util::degToRad(msg.rpy.z);
  }

  utfr_msgs::msg::EgoState res;

  // Check if the current gps position is the same as the last gps position
  if (last_gps_[0] != gps_x && last_gps_[1] != gps_y) {
    geometry_msgs::msg::Vector3 lla;
    lla.x = gps_x;
    lla.y = gps_y;
    lla.z = msg.position.altitude;

    geometry_msgs::msg::Vector3 gps_ned = utfr_dv::util::convertLLAtoNED(
      lla, datum_lla);

    gps_x = gps_ned.x;
    gps_y = gps_ned.y;

    gps_x = gps_x * cos(datum_yaw_) + gps_y * sin(datum_yaw_);
    gps_y = -gps_x * sin(datum_yaw_) + gps_y * cos(datum_yaw_);

    // Get yaw directly from IMU
    double imu_yaw = utfr_dv::util::degToRad(msg.rpy.z);
    imu_yaw -= datum_yaw_;

    // Update state with GPS position, IMU yaw, and extracted velocities
    res = updateState(gps_x, gps_y, imu_yaw);
    // res.pose.pose.position.x = gps_x;
    // res.pose.pose.position.y = gps_y;
    res.header.stamp = this->get_clock()->now();

    // Extract velocity data from SensorCan message
    double vel_x = msg.velocity.linear.x;
    double vel_y = msg.velocity.linear.y;
    double vel_yaw = 0;

    res.vel.twist.linear.x = vel_x;
    res.vel.twist.linear.y = vel_y;
    res.vel.twist.angular.z = vel_yaw;

    current_state_ = res;

    last_gps_[0] = gps_x;
    last_gps_[1] = gps_y;
  }

  // Extrapolate state using IMU data
  double dt = (this->now() - prev_time_).seconds();
  prev_time_ = this->now();
  res = extrapolateState(msg.imu, dt);
  res.pose.pose.orientation = utfr_dv::util::yawToQuaternion(utfr_dv::util::degToRad(msg.rpy.z) - datum_yaw_);
  res.header.stamp = this->get_clock()->now();
  current_state_ = res;


  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "os_sensor";
  marker.header.stamp = this->get_clock()->now();
  marker.type = visualization_msgs::msg::Marker::ARROW;

  // marker.pose.position.x = res.pose.pose.position.x;
  // marker.pose.position.y = res.pose.pose.position.y;
  // marker.pose.position.z = 0.0;

  // marker.pose.position.x = 0.0;
  // marker.pose.position.y = 0.0;
  // marker.pose.position.z = 0.0;

  // RCLCPP_WARN(this->get_logger(), "estimated: x: %f, y: %f",
  //             res.pose.pose.position.x, res.pose.pose.position.y);

  // RCLCPP_WARN(this->get_logger(), "gps: x: %f, y: %f", gps_x, gps_y);
  // RCLCPP_WARN(this->get_logger(), "ekf: x: %f, y: %f", current_state_.pose.pose.position.x, current_state_.pose.pose.position.y);

  if (false) {
    marker.pose.position.y = gps_x;
    marker.pose.position.x = gps_y;
    marker.pose.position.z = 0.0;
  } else {
    marker.pose.position.x = current_state_.pose.pose.position.y;
    marker.pose.position.y = current_state_.pose.pose.position.x;
    marker.pose.position.z = 0.0;
  }

  marker.pose.orientation = res.pose.pose.orientation;

  marker.scale.x = 0.5;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  state_publisher->publish(marker);

  // Publish the updated state
  ego_state_publisher_->publish(res);
}

// sensor_msgs/NavSatFix position
// sensor_msgs/Imu imu
/*
void EkfNode::gpsCB(const nav_msgs::msg::Odometry msg) {
  double x = msg.pose.pose.position.x;
  double y = msg.pose.pose.position.y;
  double yaw = utfr_dv::util::quaternionToYaw(msg.pose.pose.orientation);
  // Add random noise to the measurement with a covarience of 0.001
  double standardDeviation = std::sqrt(0.001);

  // Initialize a random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<double> distribution(0.0, standardDeviation);

  std::default_random_engine generator;
  // Create a uniform distribution for angle between 0 and 2*PI
  std::uniform_real_distribution<double> angle_distribution(0.0, 2 * M_PI);

  // Generate a random angle
  double angle = angle_distribution(generator);
  // Add noise to the ground truth value
  x += distribution(gen) * cos(angle);
  y += distribution(gen) * sin(angle);

  // get the velocity from the GPS
  double vel_x = msg.twist.twist.linear.x;
  double vel_y = msg.twist.twist.linear.y;
  double vel_yaw = msg.twist.twist.angular.z;

  utfr_msgs::msg::EgoState res = updateState(x, y, -yaw);
  // std::cout << "x: " << res.pose.pose.position.x << " y: " <<
  // res.pose.pose.position.y << std::endl;
  res.header.stamp = this->get_clock()->now();
  res.vel.twist.linear.x = vel_x;
  res.vel.twist.linear.y = vel_y;
  res.vel.twist.angular.z = vel_yaw;
  current_state_ = res;

  res.pose.pose.position.y = -res.pose.pose.position.y;
  ego_state_publisher_->publish(res);
}

void EkfNode::imuCB(const sensor_msgs::msg::Imu msg) {
  double dt = (this->now() - prev_time_).seconds();
  prev_time_ = this->now();

  utfr_msgs::msg::EgoState res = extrapolateState(msg, dt);
  res.header.stamp = this->get_clock()->now();
  current_state_ = res;

  res.pose.pose.position.y = -res.pose.pose.position.y;
  ego_state_publisher_->publish(res);
  // std::cout << "x: " << res.pose.pose.position.x << " y: " <<
  // res.pose.pose.position.y << std::endl;
}
*/

void EkfNode::kinematicBicycleModel(const float &throttle, const float &brake,
                                    const float &steering_angle,
                                    const double dt) {
  // Get the parameters of the vehicle
  double mass = vehicle_params_.inertia.mass;
  double l_f = vehicle_params_.kinematics.l_f;
  double l_r = vehicle_params_.kinematics.l_r;
  double drag_coefficient = vehicle_params_.kinematics.drag_coefficient;
  double rolling_resistance_coefficient =
      vehicle_params_.kinematics.rolling_resistance_coefficient;

  // Get the current angle
  double yaw =
      utfr_dv::util::quaternionToYaw(current_state_.pose.pose.orientation);

  // Get the convertsion matrix
  Eigen::MatrixXd R_2D = Eigen::MatrixXd::Zero(2, 2);
  R_2D(0, 0) = cos(yaw);
  R_2D(0, 1) = -sin(yaw);
  R_2D(1, 0) = sin(yaw);
  R_2D(1, 1) = cos(yaw);

  // Convert the global velocity to the local frame
  Eigen::Vector2d global_velocity = Eigen::Vector2d(
      current_state_.vel.twist.linear.x, current_state_.vel.twist.linear.y);
  Eigen::Vector2d local_velocity = R_2D * global_velocity;

  // Get the local velocity components
  double local_velocity_x = local_velocity(0);
  double local_velocity_y = local_velocity(1);
  double local_velocity_magnitude =
      std::sqrt(local_velocity_x * local_velocity_x +
                local_velocity_y * local_velocity_y);

  // Get the local position
  Eigen::Vector2d global_position = Eigen::Vector2d(
      current_state_.pose.pose.position.x, current_state_.pose.pose.position.y);
  Eigen::Vector2d local_position = R_2D.transpose() * global_position;
  double local_x = local_position(0);
  double local_y = local_position(1);

  // Get the mnet force in the x direction
  double throttle_force = throttle * mass;
  double brake_force = (brake)*mass;
  double drag_force = drag_coefficient * local_velocity_x * local_velocity_x;
  double rolling_resistance_force =
      (local_velocity_magnitude > 0)
          ? rolling_resistance_coefficient * mass * 9.81
          : 0.0;
  double net_force_x =
      throttle_force - brake_force - drag_force - rolling_resistance_force;

  // Calculate the lateral force
  double lateral_force =
      (mass * local_velocity_x * local_velocity_x / (l_f + l_r)) *
      tan(steering_angle);

  // Calculate acceleration
  double acceleration_x = net_force_x / mass;
  double acceleration_y = lateral_force / mass;

  // Update velocity
  double updated_local_velocity_x = local_velocity_x + acceleration_x * dt;
  double updated_local_velocity_y = local_velocity_y + acceleration_y * dt;

  // Position update
  double updated_local_x =
      local_x + local_velocity_x * dt + 0.5 * acceleration_x * dt * dt;
  double updated_local_y =
      local_y + local_velocity_y * dt + 0.5 * acceleration_y * dt * dt;

  // Yaw update
  double yaw_rate =
      local_velocity_magnitude * tan(steering_angle) / (l_f + l_r);
  double updated_yaw = yaw + yaw_rate * dt;

  // Convert the local frame back to global frame
  Eigen::Vector2d updated_global_velocity =
      R_2D.transpose() *
      Eigen::Vector2d(updated_local_velocity_x, updated_local_velocity_y);
  Eigen::Vector2d updated_global_position =
      R_2D * Eigen::Vector2d(updated_local_x, updated_local_y);

  // Update the current state
  current_state_.pose.pose.position.x = updated_global_position(0);
  current_state_.pose.pose.position.y = updated_global_position(1);
  current_state_.vel.twist.linear.x = updated_global_velocity(0);
  current_state_.vel.twist.linear.y = updated_global_velocity(1);
  current_state_.pose.pose.orientation =
      utfr_dv::util::yawToQuaternion(updated_yaw);
  current_state_.vel.twist.angular.z = yaw_rate;
}

void EkfNode::dynamicBicycleModel(const float &throttle, const float &brake,
                                  const float &steering_angle,
                                  const double dt) {
  // Get the parameters of the vehicle
  double mass = vehicle_params_.inertia.mass;
  double c_r = vehicle_params_.tire.c_r;
  double c_f = vehicle_params_.tire.c_f;
  double l_f = vehicle_params_.kinematics.l_f;
  double l_r = vehicle_params_.kinematics.l_r;
  double Izz = vehicle_params_.inertia.Izz;
  double drag_coefficient = vehicle_params_.kinematics.drag_coefficient;
  double rolling_resistance_coefficient =
      vehicle_params_.kinematics.rolling_resistance_coefficient;

  // Get the current state of the vehicle
  double global_x = current_state_.pose.pose.position.x;
  double global_y = current_state_.pose.pose.position.y;
  double angular_velocity = -1 * current_state_.vel.twist.angular.z;
  double yaw =
      utfr_dv::util::quaternionToYaw(current_state_.pose.pose.orientation);

  // Form a matrix to convert the global velocity to the local frame
  Eigen::MatrixXd R_2D = Eigen::MatrixXd::Zero(2, 2);
  R_2D(0, 0) = cos(yaw);
  R_2D(0, 1) = -sin(yaw);
  R_2D(1, 0) = sin(yaw);
  R_2D(1, 1) = cos(yaw);

  // Convert the global velocity to the local frame
  Eigen::Vector2d global_velocity = Eigen::Vector2d(
      current_state_.vel.twist.linear.x, current_state_.vel.twist.linear.y);
  Eigen::Vector2d local_velocity = R_2D * global_velocity;
  double local_velocity_x = local_velocity(0);
  double local_velocity_y = local_velocity(1);

  // Find the slip angles for the front and rear tires and the lateral forces
  double slip_angle_front =
      atan((local_velocity_y + l_r * angular_velocity) / local_velocity_x);
  double Fy_f = c_r * slip_angle_front;
  double slip_angle_rear =
      atan((local_velocity_y - l_f * angular_velocity) / local_velocity_x) -
      steering_angle;
  double Fy_r = c_f * slip_angle_rear;

  // Calculate the net longitudinal force
  double Fx = throttle * mass - brake * mass -
              drag_coefficient * local_velocity_x -
              rolling_resistance_coefficient * mass * 9.81;

  // Calculate the accerlation in the x_direction
  double a_x = Fx / mass;

  // Find the state updates
  double delta_vx = angular_velocity * local_velocity_y + a_x -
                    (drag_coefficient * local_velocity_x * local_velocity_x +
                     2 * Fy_f * sin(steering_angle)) /
                        mass;
  double delta_vy = 2 * (Fy_f * cos(steering_angle) + Fy_r) / mass -
                    angular_velocity * local_velocity_x;
  double delta_vphi = 2 * (Fy_f * l_f * cos(steering_angle) - Fy_r * l_r) / Izz;

  // Update the local velocity
  local_velocity_x += delta_vx * dt;
  local_velocity_y += delta_vy * dt;

  // Converit it back to the global frame
  Eigen::Vector2d updated_global_velocity =
      R_2D.transpose() * Eigen::Vector2d(local_velocity_x, local_velocity_y);

  // Update the global position
  global_x += updated_global_velocity(0) * dt;
  global_y += updated_global_velocity(1) * dt;

  // Update the current state
  current_state_.pose.pose.position.x = global_x;
  current_state_.pose.pose.position.y = global_y;
  current_state_.vel.twist.linear.x = updated_global_velocity(0);
  current_state_.vel.twist.linear.y = updated_global_velocity(1);
  yaw += angular_velocity * dt;
  current_state_.pose.pose.orientation = utfr_dv::util::yawToQuaternion(yaw);
  current_state_.vel.twist.angular.z += delta_vphi * dt;
}

utfr_msgs::msg::EgoState EkfNode::updateState(const double x, const double y,
                                              const double yaw) {

  Eigen::MatrixXd H; // Measurement matrix
  Eigen::MatrixXd R; // Measurement noise covariance matrix

  H = Eigen::MatrixXd::Zero(3, 6);
  H(0, 0) = 1; // Map x position
  H(1, 1) = 1; // Map y_position
  H(2, 4) = 1; // Map yaw

  R = Eigen::MatrixXd::Identity(3, 3);
  R(0, 0) = 0.005; // Variance for x measurement
  R(1, 1) = 0.005; // Variance for y measurement
  R(2, 2) = 0.005; // Variance for yaw measurement

  // Compute the Kalman gain
  // K = P * H^T * (H * P * H^T + R)^-1
  Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

  // Update the estimate with the measurement
  // x = x + K * (z - H * x)
  Eigen::VectorXd state = Eigen::VectorXd(6);
  state << current_state_.pose.pose.position.x,
      current_state_.pose.pose.position.y, current_state_.vel.twist.linear.x,
      current_state_.vel.twist.linear.y,
      utfr_dv::util::quaternionToYaw(current_state_.pose.pose.orientation),
      current_state_.vel.twist.angular.z;

  // Output the Kalman gain
  // std::cout << "Kalman gain: " << std::endl;
  // std::cout << K << std::endl;
  // std::cout << state << std::endl;

  Eigen::VectorXd measurement = Eigen::VectorXd(3);
  measurement << x, y, yaw;
  state = state + K * (measurement - H * state);

  // Update the uncertainty
  // P = (I - K * H) * P(I - K * H)^T + K * R * K^T
  P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_ *
           (Eigen::MatrixXd::Identity(6, 6) - K * H).transpose() +
       K * R * K.transpose();

  utfr_msgs::msg::EgoState state_msg = current_state_;
  state_msg.pose.pose.position.x = state(0);
  state_msg.pose.pose.position.y = state(1);
  state_msg.pose.pose.orientation = utfr_dv::util::yawToQuaternion(
      state(4)); // Update the orientation using the yaw state

  return state_msg;
}

utfr_msgs::msg::EgoState EkfNode::extrapolateState(const sensor_msgs::msg::Imu imu, const double dt) {

  Eigen::MatrixXd F; // State transition matrix
  Eigen::MatrixXd G; // Control input matrix

  double yaw =
      utfr_dv::util::quaternionToYaw(current_state_.pose.pose.orientation);
  F = Eigen::MatrixXd::Identity(6, 6);
  F(0, 2) = cos(yaw) * dt;
  F(0, 3) = sin(yaw) * dt;
  F(1, 2) = -sin(yaw) * dt;
  F(1, 3) = cos(yaw) * dt;
  F(4, 5) = dt;
  F(5, 5) = 0;

  G = Eigen::MatrixXd::Zero(6, 3);
  G(0, 0) = 0.5 * dt * dt * cos(yaw);
  G(1, 0) = -0.5 * dt * dt * sin(yaw);
  G(0, 1) = 0.5 * dt * dt * sin(yaw);
  G(1, 1) = 0.5 * dt * dt * cos(yaw);
  G(2, 0) = dt;
  G(3, 0) = dt;
  G(4, 2) = dt;
  G(5, 2) = 1;

  // Extrapolate state
  // x_new = Fx + Gu

  Eigen::VectorXd state = Eigen::VectorXd(6);
  state << current_state_.pose.pose.position.x,
      current_state_.pose.pose.position.y, current_state_.vel.twist.linear.x,
      current_state_.vel.twist.linear.y, yaw,
      current_state_.vel.twist.angular.z;

  Eigen::VectorXd input = Eigen::VectorXd(3);
  input << imu.linear_acceleration.x, imu.linear_acceleration.y,
      -imu.angular_velocity.z;
  state = F * state + G * input;

  // Extrapolate uncertainty
  // P = FPF^T + Q
  Eigen::MatrixXd Q_; // Process noise covariance matrix
  Q_ = Eigen::MatrixXd::Identity(6, 6);
  Q_(0, 0) = 0.01;    // Variance for x position, in m
  Q_(1, 1) = 0.01;    // Variance for y position
  Q_(2, 2) = 0.05;    // Variance for x velocity, m/s
  Q_(3, 3) = 0.05;    // Variance for y velocity
  Q_(4, 4) = 0.00872; // Variance for yaw in radian
  Q_(5, 5) = 1;  // Variance for yaw rate, to be tuned

  P_ = F * P_ * F.transpose() + Q_;

  // std::cout << "Yaw: " << state(4) << std::endl;

  // Add the state to the message
  utfr_msgs::msg::EgoState state_msg = current_state_;
  state_msg.pose.pose.position.x = state(0);
  state_msg.pose.pose.position.y = state(1);
  state_msg.vel.twist.linear.x = state(2);
  state_msg.vel.twist.linear.y = state(3);
  state_msg.pose.pose.orientation = utfr_dv::util::yawToQuaternion(state(4));
  state_msg.vel.twist.angular.z = -imu.angular_velocity.z;

  return state_msg;
}


void EkfNode::timerCB() {}

} // namespace ekf
} // namespace utfr_dv
