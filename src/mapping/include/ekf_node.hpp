/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: ekf_node.hpp
* auth: Arthur Xu
* desc: ekf node header
*/
#pragma once

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// System Requirements
#include <chrono>
#include <fstream>
#include <functional>
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string>
#include <vector>

// Message Requirements
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/system_status.hpp>
#include <utfr_msgs/msg/sensor_can.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace ekf {

class EkfNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, initPublishers and initTimers.
   */
  EkfNode();

private:
  /*! Initialize and load params from config.yaml:
   */
  void initParams();

  /*! Initialize Subscribers:
   */
  void initSubscribers();

  /*! Initialize Publishers:
   */
  void initPublishers();

  /*! Initialize Timers:
   */
  void initTimers();

  /*! Initialize Heartbeat:
   */
  void initHeartbeat();

  /*! CAN sensor callback function
  */
  void sensorCB(const utfr_msgs::msg::SensorCan msg);

  /*! Implement a dynamic vehicle model
  *  Use the throttle, brake, and steering angle to update the vehicle model state
  *  @param[in] throttle float&, throttle data
  *  @param[in] brake float&, braking data
  *  @param[in] steering_angle float&, steering angle, in radians
  *  @param[out] state geometry_msgs::msg::EgoState&, estimated state via the vehicle model
  */
  void vehicleModel(const float& throttle, 
                    const float& brake, 
                    const float& steering_angle 
                    );

  /*! Main EKF function.
  *  The main EKF loop. Takes in measurement data for the EKF and performes a single update step.
  *  @param[in] imu_data sensor_msgs::msg::Imu&, IMU data.
  *  @param[in] gps_data ?, GPS data.
  *  @param[in] vehicle_model_data utfr_msgs::msg::EgoState&, vehicle model data.
  */
  void EKF();

  // Publishers
  rclcpp::Publisher<utfr_msgs::msg::EgoState>::SharedPtr state_estimation_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;

  // Subscribers
  // SensorCAN handles GPS, IMU, and wheel/steering speed data! 
  rclcpp::Subscription<utfr_msgs::msg::SensorCan>::SharedPtr 
      sensorcan_subscriber_;

  // Global variables
  utfr_msgs::msg::EgoState current_state_; // Estimated state of the vehicle
  double kalman_gain_; // Kalman gain
};
} // namespace ekf
} // namespace utfr_dv
