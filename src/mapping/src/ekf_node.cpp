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

#include <ekf_node.hpp>

namespace utfr_dv {
namespace ekf {

EkfNode::EkfNode() : Node("ekf_node") {

  // set heartbeat state to not ready
  HeartBeatState heartbeat_state_ = HeartBeatState::NOT_READY;
  heartbeat_rate_ = 1;
  this->initTimers();
  
  this->initHeartbeat();
  this->publishHeartbeat();
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  
  // set heartbeat state to active
  heartbeat_state_ = HeartBeatState::ACTIVE;
}

void EkfNode::initParams() {
  P_ = 1 * Eigen::MatrixXd::Identity(6, 6);
  prev_time_ = this->now();
  current_state_.pose.pose.position.x = 0.0;
  current_state_.pose.pose.position.y = 0.0;
}

void EkfNode::initSubscribers() {

  sensorcan_subscriber_ = this->create_subscription<utfr_msgs::msg::SensorCan>(
      topics::kSensorCan, 1,
      std::bind(&EkfNode::sensorCB, this, std::placeholders::_1));

  gps_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/ground_truth/odom", 1,
      std::bind(&EkfNode::gpsCB, this, std::placeholders::_1));

  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 1,
      std::bind(&EkfNode::imuCB, this, std::placeholders::_1));
}

void EkfNode::initPublishers() {
  state_estimation_publisher_ =
      this->create_publisher<utfr_msgs::msg::EgoState>(topics::kEgoState, 10);
  pose_publisher_ = this->create_publisher<utfr_msgs::msg::EgoState>(topics::kPose, 10);
}

void EkfNode::initTimers() {
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(heartbeat_rate_),
        std::bind(&EkfNode::publishHeartbeat, this));
}


void EkfNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kEKFHeartbeat, 10);
}

void EkfNode::sensorCB(const utfr_msgs::msg::SensorCan msg) {
  // We gotta figure out a way to determine if we're getting a GPS message or
  // an IMU one. Just follow our train of thought here:

  bool is_gps = false;
  bool is_imu = false;

  if (is_gps) {

    // Get the position from the GPS
    // ...
    //throwing in random value for now
    double x = 1.0;
    double y = 2.0;
    updateState(x, y, 0);
    
  } else if (is_imu) {

    float dt = 0.01; // TODO: Get this from the message
    extrapolateState(msg.imu_data, dt);
  }

  // Publish the state estimation
  // ...
}

void EkfNode::publishHeartbeat() {
    utfr_msgs::msg::Heartbeat heartbeat_msg;
    heartbeat_msg.status = static_cast<uint8_t>(heartbeat_state_);  
    heartbeat_msg.header.stamp = this->now();  

    heartbeat_publisher_->publish(heartbeat_msg);
}

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
  // Add noise to the ground truth value
  x += distribution(gen);
  y += distribution(gen);

  utfr_msgs::msg::EgoState res = updateState(x, y, -yaw);
  // std::cout << "x: " << res.pose.pose.position.x << " y: " << res.pose.pose.position.y << std::endl;
  current_state_ = res;
  res.pose.pose.position.y = -res.pose.pose.position.y;
  pose_publisher_->publish(res);
}

void EkfNode::imuCB(const sensor_msgs::msg::Imu msg) {
  double dt = (this->now() - prev_time_).seconds();
  prev_time_ = this->now();
  utfr_msgs::msg::EgoState res = extrapolateState(msg, dt);
  current_state_ = res;  

  res.pose.pose.position.y = -res.pose.pose.position.y;
  pose_publisher_->publish(res);
  // std::cout << "x: " << res.pose.pose.position.x << " y: " << res.pose.pose.position.y << std::endl;
}

void EkfNode::vehicleModel(const float &throttle, const float &brake,
                           const float &steering_angle) {}

utfr_msgs::msg::EgoState EkfNode::updateState(const double x, const double y, const double yaw) {

  Eigen::MatrixXd H; // Measurement matrix
  Eigen::MatrixXd R; // Measurement noise covariance matrix

  H = Eigen::MatrixXd::Zero(3, 6);
  H(0, 0) = 1; // Map x position
  H(1, 1) = 1; // Map y position
  H(2, 4) = 1; // Map yaw

  R = Eigen::MatrixXd::Identity(3, 3);
  R(0, 0) = 0.01; // Variance for x measurement
  R(1, 1) = 0.01; // Variance for y measurement
  R(2, 2) = 0.01; // Variance for yaw measurement
  // Compute the Kalman gain
  // K = P * H^T * (H * P * H^T + R)^-1
  Eigen::MatrixXd S = H * P_ * H.transpose() + R;
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

  // Update the estimate with the measurement
  // x = x + K * (z - H * x)
  Eigen::VectorXd state = Eigen::VectorXd(6);
  state << current_state_.pose.pose.position.x, current_state_.pose.pose.position.y,
      current_state_.vel.twist.linear.x, current_state_.vel.twist.linear.y,
      utfr_dv::util::quaternionToYaw(current_state_.pose.pose.orientation), current_state_.vel.twist.angular.z;
  
  // Output the Kalman gain
  // std::cout << "Kalman gain: " << std::endl;
  // std::cout << K << std::endl;  
  // std::cout << state << std::endl;

  Eigen::VectorXd measurement = Eigen::VectorXd(3);
  measurement << x, y, yaw;

  state = state + K * (measurement - H * state);

  // Update the uncertainty
  // P = (I - K * H) * P(I - K * H)^T + K * R * K^T
  P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_ * (Eigen::MatrixXd::Identity(6, 6) - K * H).transpose() + K * R * K.transpose();

  utfr_msgs::msg::EgoState state_msg = current_state_;
  state_msg.pose.pose.position.x = state(0);
  state_msg.pose.pose.position.y = state(1);
  state_msg.pose.pose.orientation = utfr_dv::util::yawToQuaternion(state(4)); // Update the orientation using the yaw state

  return state_msg;
}

utfr_msgs::msg::EgoState EkfNode::extrapolateState(const sensor_msgs::msg::Imu imu_data, const double dt) {
  
  Eigen::MatrixXd F; // State transition matrix
  Eigen::MatrixXd G; // Control input matrix


  double yaw = utfr_dv::util::quaternionToYaw(current_state_.pose.pose.orientation);
   F = Eigen::MatrixXd::Identity(6, 6);
  F(0, 2) = cos(yaw)*dt;
  F(0, 3) = sin(yaw)*dt;
  F(1, 2) = -sin(yaw)*dt;
  F(1, 3) = cos(yaw)*dt;
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
  state << current_state_.pose.pose.position.x, current_state_.pose.pose.position.y,
      current_state_.vel.twist.linear.x, current_state_.vel.twist.linear.y,
      yaw, current_state_.vel.twist.angular.z;

  Eigen::VectorXd input = Eigen::VectorXd(3);
  input << imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, 
   -imu_data.angular_velocity.z;

  state = F * state + G * input;

  // Extrapolate uncertainty
  // P = FPF^T + Q
  Eigen::MatrixXd Q_; // Process noise covariance matrix
  Q_ = Eigen::MatrixXd::Identity(6, 6); 
  Q_(0, 0) = 0.01; // Variance for x position, in m
  Q_(1, 1) = 0.01; // Variance for y position
  Q_(2, 2) = 0.05; // Variance for x velocity, m/s
  Q_(3, 3) = 0.05; // Variance for y velocity
  Q_(4, 4) = 0.00872; // Variance for yaw in radian
  Q_(5, 5) = 0.0064; // Variance for yaw rate, to be tuned

  P_ = F * P_ * F.transpose() + Q_;

  // std::cout << "Yaw: " << state(4) << std::endl;

  // Add the state to the message
  utfr_msgs::msg::EgoState state_msg = current_state_;
  state_msg.pose.pose.position.x = state(0);
  state_msg.pose.pose.position.y = state(1);
  state_msg.vel.twist.linear.x = state(2);
  state_msg.vel.twist.linear.y = state(3);
  state_msg.pose.pose.orientation = utfr_dv::util::yawToQuaternion(state(4));
  state_msg.vel.twist.angular.z = -imu_data.angular_velocity.z;

  return state_msg;
}

} // namespace ekf
} // namespace utfr_dv
