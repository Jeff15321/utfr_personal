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
#include <cmath>
#include <vector>


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
  P_ = 1000.0 * Eigen::MatrixXd::Identity(5, 5);
}

void EkfNode::initSubscribers() {

  sensorcan_subscriber_ = this->create_subscription<utfr_msgs::msg::SensorCan>(
      topics::kSensorCan, 1,
      std::bind(&EkfNode::sensorCB, this, std::placeholders::_1));
}

void EkfNode::initPublishers() {
  state_estimation_publisher_ =
      this->create_publisher<utfr_msgs::msg::EgoState>(topics::kEgoState, 10);
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
    updateState(x, y);
    
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


void EkfNode::vehicleModel(const float &throttle, const float &brake,
                           const float &steering_angle) {}

utfr_msgs::msg::EgoState EkfNode::updateState(const double x, const double y) {

  Eigen::MatrixXd H; // Measurement matrix
  Eigen::MatrixXd R; // Measurement noise covariance matrix

  H = Eigen::MatrixXd::Zero(2, 6);
  H(0, 0) = 1; // Map x position
  H(1, 1) = 1; // Map y position

  R = Eigen::MatrixXd::Identity(2, 2);
  R(0, 0) = 9.0; // Variance for x measurement
  R(1, 1) = 9.0; // Variance for y measurement

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
  
  Eigen::VectorXd measurement = Eigen::VectorXd(2);
  measurement << x, y;

  state = state + K * (measurement - H * state);

  // Update the uncertainty
  // P = (I - K * H) * P(I - K * H)^T + K * R * K^T
  P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_ * (Eigen::MatrixXd::Identity(6, 6) - K * H).transpose() + K * R * K.transpose();

  utfr_msgs::msg::EgoState state_msg = current_state_;
  state_msg.pose.pose.position.x = state(0);
  state_msg.pose.pose.position.y = state(1);
  state_msg.vel.twist.linear.x = state(2);
  state_msg.vel.twist.linear.y = state(3);
  state_msg.pose.pose.orientation = utfr_dv::util::yawToQuaternion(state(4));
  state_msg.vel.twist.angular.z = state(5);

  return state_msg;
}

utfr_msgs::msg::EgoState EkfNode::extrapolateState(const sensor_msgs::msg::Imu imu_data, const double dt) {
  
  Eigen::MatrixXd F; // State transition matrix
  Eigen::MatrixXd G; // Control input matrix

  F = Eigen::MatrixXd::Identity(6, 6);
  F(0, 2) = dt;
  F(1, 3) = dt;
  F(4, 5) = dt;

  G = Eigen::MatrixXd::Zero(6, 3);
  G(2, 0) = dt;
  G(3, 1) = dt;
  G(5, 2) = dt;
  
  // Extrapolate state
  // x_new = Fx + Gu

  Eigen::VectorXd state = Eigen::VectorXd(6);
  state << current_state_.pose.pose.position.x, current_state_.pose.pose.position.y,
      current_state_.vel.twist.linear.x, current_state_.vel.twist.linear.y,
      utfr_dv::util::quaternionToYaw(current_state_.pose.pose.orientation), current_state_.vel.twist.angular.z;

  Eigen::VectorXd input = Eigen::VectorXd(3);
  input << imu_data.linear_acceleration.x, imu_data.linear_acceleration.y,
      imu_data.angular_velocity.z;

  state = F * state + G * input;

  // Extrapolate uncertainty
  // P = FPF^T + Q
  Eigen::MatrixXd Q_; // Process noise covariance matrix
  Q_ = 100 * Eigen::MatrixXd::Identity(6, 6); 

  P_ = F * P_ * F.transpose() + Q_;

  // Add the state to the message
  utfr_msgs::msg::EgoState state_msg = current_state_;
  state_msg.pose.pose.position.x = state(0);
  state_msg.pose.pose.position.y = state(1);
  state_msg.vel.twist.linear.x = state(2);
  state_msg.vel.twist.linear.y = state(3);
  state_msg.pose.pose.orientation = utfr_dv::util::yawToQuaternion(state(4));
  state_msg.vel.twist.angular.z = state(5);

  return state_msg;
}
std::vector<double> datum_lla={std::nan(""),std::nan(""),std::nan("")};
std::vector<double> EkfNode::lla2ecr(std::vector<double>& inputVector){
  double lat = inputVector[0];
  double lon = inputVector[1];
  double h = inputVector[2];
  double Re = 6378137;       // Earth_Equatorial_Radius, in m
  double Rp = 6356752;       // Earth_Polar_Radius, in m
  double f  = static_cast<double>(Re-Rp)/Re;    // Earth_Flattening Coefficient

  // Compute the ECR coordinates
  double temp = Re / std::sqrt(1 + std::pow((1 - f) * std::tan(lat), 2));
  


  double x = (temp + h * std::cos(lat)) * std::cos(lon);
  double y = (temp + h * std::cos(lat)) * std::sin(lon);
  double z = temp * (std::pow((1 - f), 2) * std::tan(lat)) + h * std::sin(lat);
  std::vector<double> resultVector ={x,y,z};
  return resultVector;
}



void EkfNode::ecr2enu(double& x, double& y, double& z, std::vector<double>& datum_lla) {
    double lat = datum_lla[0];
    double lon = datum_lla[1];
    double h = datum_lla[2];
  
    std::vector<std::vector<double>> T = {
        {-std::sin(lon), std::cos(lon), 0.0},
        {-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat)},
        {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)}
    };

    // Assuming transform function returns radar_ecr
    std::vector<double> radar_ecr = lla2ecr(datum_lla);

    // Matrix multiplication T * radar_ecr
    std::vector<double> radar_rrc(3, 0.0);
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            radar_rrc[i] += T[i][j] * radar_ecr[j];
        }
    }

    // Matrix multiplication T * [x, y, z]
    std::vector<double> flat_xyz = {x, y, z}; // assuming x, y, z are individual float variables
    std::vector<double> out(3, 0.0);
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            out[i] += T[i][j] * flat_xyz[j];
        }
    }

    // Adjust x, y, z
    x = out[0] - radar_rrc[0];
    y = out[1] - radar_rrc[1];
    z = out[2] - radar_rrc[2];
}
std::vector<double> EkfNode::lla2enu(std::vector<double>& inputVector){

  std::vector<double> resultVector = {0,0,0};
  if (std::isnan(datum_lla[0])){
    for(int i = 0; i <3; i++){
      datum_lla[i] = inputVector[i];
      
    }

    return resultVector;

  }

  
  std::vector<double> inputECR = lla2ecr(inputVector);
  double x = inputECR[0];
  double y = inputECR[1];
  double z = inputECR[2];

  
  ecr2enu(x,y,z,datum_lla);//now x y z is in enu

  resultVector[0] = x;
  resultVector[1] = y;
  resultVector[2] = z;
  return resultVector;

}

} // namespace ekf
} // namespace utfr_dv
 