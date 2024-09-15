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

  tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
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

  gps_state_publisher =
      this->create_publisher<visualization_msgs::msg::Marker>("GPS_STATE", 10);

  navsatfix_publisher_ =
      this->create_publisher<sensor_msgs::msg::NavSatFix>("xsens/gnss", 10);
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

geometry_msgs::msg::TransformStamped
EkfNode::map_to_imu_link(const utfr_msgs::msg::EgoState &state) {

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->get_clock()->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "imu_link";

  transform.transform.translation.x = state.pose.pose.position.x;
  transform.transform.translation.y = state.pose.pose.position.y;
  transform.transform.translation.z = 0.265;

  transform.transform.rotation = state.pose.pose.orientation;
  return transform;
}

geometry_msgs::msg::TransformStamped
EkfNode::map_to_base_footprint(const utfr_msgs::msg::EgoState &state) {

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->get_clock()->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_footprint";

  transform.transform.translation.x = state.pose.pose.position.x;
  transform.transform.translation.y = state.pose.pose.position.y;
  transform.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(M_PI, 0, util::quaternionToYaw(state.pose.pose.orientation));

  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  return transform;
}

void EkfNode::sensorCB(const utfr_msgs::msg::SensorCan msg) {
  double gps_x = msg.position.latitude;
  double gps_y = msg.position.longitude;

  navsatfix_publisher_->publish(msg.position);

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

  // Get yaw directly from IMU
  double imu_yaw = utfr_dv::util::degToRad(msg.rpy.z);
  imu_yaw -= datum_yaw_;

  // Check if the current gps position is the same as the last gps position
  if (last_gps_[0] != gps_x && last_gps_[1] != gps_y) {
    geometry_msgs::msg::Vector3 lla;
    lla.x = gps_x;
    lla.y = gps_y;
    lla.z = msg.position.altitude;

    geometry_msgs::msg::Vector3 gps_ned =
        utfr_dv::util::convertLLAtoNED(lla, datum_lla);

    gps_y = gps_ned.x * cos(datum_yaw_) + gps_ned.y * sin(datum_yaw_);
    gps_x = -gps_ned.x * sin(datum_yaw_) + gps_ned.y * cos(datum_yaw_);

    // Update state with GPS position, IMU yaw, and extracted velocities
    res = updateState(gps_x, gps_y, imu_yaw);
    // res.pose.pose.position.x = gps_x;
    // res.pose.pose.position.y = gps_y;
    res.header.stamp = this->get_clock()->now();
    res.header.frame_id = "map";

    current_state_ = res;

    last_gps_[0] = gps_x;
    last_gps_[1] = gps_y;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.type = visualization_msgs::msg::Marker::CUBE;

    marker.pose.position.y = current_state_.pose.pose.position.y;
    marker.pose.position.x = current_state_.pose.pose.position.x;
    marker.pose.position.z = 0.0;

    marker.pose.orientation = res.pose.pose.orientation;

    marker.scale.x = 1.8;
    marker.scale.y = 1.5;
    marker.scale.z = 0.5;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    gps_state_publisher->publish(marker);

  } else {
    current_state_.pose.pose.orientation =
        utfr_dv::util::yawToQuaternion(imu_yaw);
  }

  // Extract velocity data from SensorCan message
  double vel_x = msg.velocity.linear.x;
  double vel_y = msg.velocity.linear.y;
  double vel_yaw = msg.velocity.angular.z;

  current_state_.vel.twist.linear.x =
      vel_y * cos(datum_yaw_) + vel_x * sin(datum_yaw_);
  current_state_.vel.twist.linear.y =
      -vel_y * sin(datum_yaw_) + vel_x * cos(datum_yaw_);
  current_state_.vel.twist.angular.z = vel_yaw;

  // Extrapolate state using IMU data
  double dt = (this->now() - prev_time_).seconds();
  prev_time_ = this->now();
  res = extrapolateState(msg.imu, dt);
  res.pose.pose.orientation = utfr_dv::util::yawToQuaternion(
      utfr_dv::util::degToRad(msg.rpy.z) - datum_yaw_);

  res.vel.twist.linear.x =
      sqrt(pow(msg.velocity.linear.x, 2) + pow(msg.velocity.linear.y, 2));
  res.vel.twist.linear.y = 0;
  res.header.stamp = this->get_clock()->now();

  current_state_ = res;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = this->get_clock()->now();
  marker.type = visualization_msgs::msg::Marker::CUBE;

  if (false) {
    marker.pose.position.y = 0.0;
    marker.pose.position.x = 0.0;
    marker.pose.position.z = 0.0;
  } else {
    marker.pose.position.x = current_state_.pose.pose.position.x;
    marker.pose.position.y = current_state_.pose.pose.position.y;
    marker.pose.position.z = 0.0;
  }

  marker.pose.orientation = res.pose.pose.orientation;

  marker.scale.x = 1.8;
  marker.scale.y = 0.8;
  marker.scale.z = 0.5;

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  state_publisher->publish(marker);

  // Publish the updated state
  ego_state_publisher_->publish(res);
  tf_br_->sendTransform(map_to_imu_link(res));
  tf_br_->sendTransform(map_to_base_footprint(res));
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
  R(0, 0) = 0.06; // Variance for x measurement
  R(1, 1) = 0.06; // Variance for y measurement
  R(2, 2) = 0.06; // Variance for yaw measurement
  R(0, 0) = 0.06; // Variance for x measurement
  R(1, 1) = 0.06; // Variance for y measurement
  R(2, 2) = 0.06; // Variance for yaw measurement

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

utfr_msgs::msg::EgoState
EkfNode::extrapolateState(const sensor_msgs::msg::Imu imu, const double dt) {

  Eigen::MatrixXd F; // State transition matrix
  Eigen::MatrixXd G; // Control input matrix

  double yaw =
      utfr_dv::util::quaternionToYaw(current_state_.pose.pose.orientation);
  F = Eigen::MatrixXd::Identity(6, 6);
  F(0, 2) = dt;
  F(0, 3) = dt;
  F(1, 2) = dt;
  F(1, 3) = dt;
  F(0, 2) = dt;
  F(0, 3) = dt;
  F(1, 2) = dt;
  F(1, 3) = dt;
  F(4, 5) = dt;
  F(5, 5) = 0;

  G = Eigen::MatrixXd::Zero(6, 3);
  G(0, 0) = 0.5 * dt * dt;
  G(1, 0) = -0.5 * dt * dt;
  G(0, 1) = 0.5 * dt * dt;
  G(1, 1) = 0.5 * dt * dt;
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

  // Transform the acceleration data from the IMU to the map frame
  double accel_x = imu.linear_acceleration.x;
  double accel_y = imu.linear_acceleration.y;

  sensor_msgs::msg::Imu imu_msg = imu;

  imu_msg.linear_acceleration.y =
      accel_x * cos(datum_yaw_) + accel_y * sin(datum_yaw_);
  imu_msg.linear_acceleration.x =
      -accel_x * sin(datum_yaw_) + accel_y * cos(datum_yaw_);

  Eigen::VectorXd input = Eigen::VectorXd(3);
  input << imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
      -imu_msg.angular_velocity.z;
  state = F * state + G * input;

  // Extrapolate uncertainty
  // P = FPF^T + Q
  Eigen::MatrixXd Q_; // Process noise covariance matrix
  Q_ = Eigen::MatrixXd::Identity(6, 6);
  Q_(0, 0) = 0.001;   // Variance for x position, in m
  Q_(1, 1) = 0.001;   // Variance for y position
  Q_(2, 2) = 0.01;    // Variance for x velocity, m/s
  Q_(3, 3) = 0.01;    // Variance for y velocity
  Q_(4, 4) = 0.00872; // Variance for yaw in radian
  Q_(5, 5) = 0.08;    // Variance for yaw rate, to be tuned

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
