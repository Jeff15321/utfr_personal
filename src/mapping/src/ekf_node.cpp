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
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
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

void EkfNode::initTimers() {}

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

    updateState(double x, double y);
    
  } else if (is_imu) {

    // Get acceleration and steering from the IMU
    // ...

    extrapolateState(double accel_cmd, double steering_cmd, double dt);
  }

  // Publish the state estimation
  // ...
}

void EkfNode::vehicleModel(const float &throttle, const float &brake,
                           const float &steering_angle) {}

void EkfNode::updateState(const double x, const double y) {

  // Compute the Kalman gain
  // K = P * H^T * (H * P * H^T + R)^-1
  kalman_gain_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();

  // Update the estimate with the measurement
  // x = x + K * (z - H * x)

  // Update the uncertainty
  // P = (I - K * H) * P(I - K * H)^T + K * R * K^T
}

void EkfNode::extrapolateState(const double accel_cmd,
                               const double steering_cmd, const double dt) {
  // Extrapolate state
  // x_new = Fx + Gu
Eigen::Matrix2d F;
Eigen::Matrix2d G;
double L=1.58; //wheelbase is 1.58m
//double x_new= //current_x+[current_vx*dt+0.5*ax*pow(dt,2)]*cosine(steering_cmd)
 //+sine(steering_cmd)[current_vy*dt+0.5*ay*pow(dt,2)]
//double y_new= //current_y+[current_vy*dt+0.5*ay*pow(dt,2)]*cosine(steering_cmd)
 //+sine(steering_cmd)[current_vx*dt+0.5*ax*pow(dt,2)]
//double Vx_new= //vx+ax*dt
//double Vy_new= //vy+ay*dt
//double ax_new=//ax
//double ay_new=//ay
//double steering_cmd_new=//steering_cmd+(vy+ay*dt)/ L *dt;


F=<<1,0,cos(steering_cmd)*dt, sin(steering_cmd)*dt,0,
    0,1,sin(steering_cmd)*dt,cos(steering_cmd)*dt,0,
    0,0,1,0,0,
    0,0,0,1,0,
    0,0,0,dt/L,1;

G<<cos(steering_cmd)*0.5*pow(dt,2),sin(steering_cmd)*0.5*pow(dt,2),
   sin(steering_cmd)*0.5*pow(dt,2), cos(steering_cmd)*0.5*pow(dt,2);

Eigen::Vector5d x(current_state_.pose.pose.position.x, current_state_.pose.pose.position.y,
current_state_.vel.twist.linear.x, current_state_.vel.twist.linear.y, 
current_state_.steering_angle);
Eigen::Vector2d u(current_state_.accel.linear.x, current_state_.accel.linear.y);
Eigen::Vector5d x_new=F*x+G*u;


  // Extrapolate uncertainty
  // P = FPF^T + Q  
  //value randomly assigned
Eigen::Matrix2d P;
Eigen::Matrix2d Q;
P<<1,0
   0,1;
Q<<1,0
   0,1;
Eigen::Matrix2d P_new=F*P*F.transpose()+Q;
}

} // namespace ekf
} // namespace utfr_dv
