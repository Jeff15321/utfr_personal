/*
 * file: ekf_node.cpp
 * auth: Arthur Xu
 * desc: ekf node class
 */

#include <cmath>
#include "ekf_node.hpp"
#include <vector>
#include <limits>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace utfr_dv {
namespace ekf {

EkfNode::EkfNode() : Node("ekf_node") {
  this->initParams();
  this->initHeartbeat();
  this->initSubscribers();
  this->initPublishers();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::NOT_READY);
  this->initTimers();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
}

void EkfNode::initParams() {
  std::cout << "You acc suck";
  this->declare_parameter("update_rate", 33.33);
  update_rate_ = 10.0; //this->get_parameter("update_rate").as_double();
  this->declare_parameter("mapping_mode", 0);
  mapping_mode_ = this->get_parameter("mapping_mode").as_int();
  this->declare_parameter("ekf_on", 1);
  ekf_on_ = this->get_parameter("ekf_on").as_int();
  prev_time_ = this->now();
  current_state_.pose.pose.position.x = 0.0;
  current_state_.pose.pose.position.y = 0.0;
  datum_lla = geometry_msgs::msg::Vector3();
  datum_yaw_ = std::numeric_limits<double>::quiet_NaN();
  imu_yaw_ = 0.0;

  //tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void EkfNode::initSubscribers() {
  if (mapping_mode_ != 2 && ekf_on_ == 1) {
    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/filter/positionlla", rclcpp::QoS(10),
        std::bind(&EkfNode::poseCB, this, std::placeholders::_1));

    twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/filter/twist", rclcpp::QoS(10),
        std::bind(&EkfNode::twistCB, this, std::placeholders::_1));

    orientation_subscriber_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
        "/filter/quaternion", rclcpp::QoS(10),
        std::bind(&EkfNode::orientationCB, this, std::placeholders::_1));
  }
}

void EkfNode::initPublishers() {
  if (mapping_mode_ != 2 && ekf_on_ == 1) {
    ego_state_publisher_ = this->create_publisher<utfr_msgs::msg::EgoState>(
        topics::kEgoState, 10);
  }

  state_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "IM_GONNA_KMS", 10);

  gps_state_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "GPS_STATE", 10);

  navsatfix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "xsens/gnss", 10);

  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kEKFHeartbeat, 10);
}

void EkfNode::initTimers() {
  if (mapping_mode_ != 2 && ekf_on_ == 1) {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(this->update_rate_),
        std::bind(&EkfNode::timerCB, this));
  }
}

void EkfNode::initHeartbeat() {
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
  q.setRPY(M_PI, 0, utfr_dv::util::quaternionToYaw(state.pose.pose.orientation));

  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  return transform;
}

void EkfNode::poseCB(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
  std::cout << "pose cb start";

  double gps_x = msg->vector.x;
  double gps_y = msg->vector.y;
  double gps_z = msg->vector.z;

  if (datum_lla.x == 0.0 && datum_lla.y == 0.0 && datum_lla.z == 0.0) {
    datum_lla.x = gps_x;
    datum_lla.y = gps_y;
    datum_lla.z = gps_z;
  
  std::cout << "pose cb end";
  }

  geometry_msgs::msg::Vector3 lla;
  lla.x = gps_x;
  lla.y = gps_y;
  lla.z = gps_z;

  geometry_msgs::msg::Vector3 gps_ned =
      utfr_dv::util::convertLLAtoNED(lla, datum_lla);

  // Convert NED to map frame
  double gps_x_map =
      gps_ned.x * cos(datum_yaw_) + gps_ned.y * sin(datum_yaw_);
  double gps_y_map =
      gps_ned.x * sin(datum_yaw_) - gps_ned.y * cos(datum_yaw_);

  current_state_.pose.pose.position.x = gps_x_map;
  current_state_.pose.pose.position.y = gps_y_map;

  // Publish GPS state marker
  visualization_msgs::msg::Marker marker2;
  marker2.header.frame_id = "map";
  marker2.header.stamp = this->get_clock()->now();
  marker2.type = visualization_msgs::msg::Marker::CUBE;

  marker2.pose.position.x = gps_x_map;
  marker2.pose.position.y = gps_y_map;
  marker2.pose.position.z = 0.0;

  marker2.pose.orientation = current_state_.pose.pose.orientation;

  marker2.scale.x = 1.8;
  marker2.scale.y = 1.5;
  marker2.scale.z = 0.5;

  marker2.color.a = 1.0;
  marker2.color.r = 0.0;
  marker2.color.g = 0.0;
  marker2.color.b = 1.0;

  gps_state_publisher_->publish(marker2);
}

void EkfNode::orientationCB(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg) {
  tf2::Quaternion q(
      msg->quaternion.x,
      msg->quaternion.y,
      msg->quaternion.z,
      msg->quaternion.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  imu_yaw_ = yaw;
  if (std::isnan(datum_yaw_)) {
    datum_yaw_ = imu_yaw_;
  }
  imu_yaw_ -= datum_yaw_;
  current_state_.pose.pose.orientation = utfr_dv::util::yawToQuaternion(imu_yaw_); 
}

void EkfNode::twistCB(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  // Extract velocity data
  double vel_x = msg->twist.linear.x;
  double vel_y = msg->twist.linear.y;
  double vel_yaw = msg->twist.angular.z;

  // Transform velocities from global to map frame
  double vel_x_map = vel_y * cos(datum_yaw_+imu_yaw_) -
                 vel_x * sin(datum_yaw_+imu_yaw_);
  double vel_y_map = vel_y * sin(datum_yaw_+imu_yaw_) +
                 vel_x * cos(datum_yaw_+imu_yaw_);

  current_state_.vel.twist.linear.x = vel_x_map;
  current_state_.vel.twist.linear.y = vel_y_map;
  current_state_.vel.twist.angular.z = vel_yaw;

  //tf_br_->sendTransform(map_to_imu_link(current_state_));
  //tf_br_->sendTransform(map_to_base_footprint(current_state_));
}

void EkfNode::timerCB() {
  // Timer callback (if needed)
  // Publish the updated state
  current_state_.header.stamp = this->now();
  current_state_.header.frame_id = "map";
  ego_state_publisher_->publish(current_state_);
}

}  // namespace ekf
}  // namespace utfr_dv
