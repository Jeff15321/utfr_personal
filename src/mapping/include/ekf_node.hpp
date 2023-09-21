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
#include <utfr_msgs/msg/sensor_can.hpp>
#include <utfr_msgs/msg/system_status.hpp>

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
   *  Use the throttle, brake, and steering angle to update the vehicle model
   * state
   *  @param[in] throttle float&, throttle data
   *  @param[in] brake float&, braking data
   *  @param[in] steering_angle float&, steering angle, in radians
   *  @param[out] state geometry_msgs::msg::EgoState&, estimated state via the
   * vehicle model
   */
  void vehicleModel(const float &throttle, const float &brake,
                    const float &steering_angle);

  /*! Given the vehicle's current state, and a collection of inputs like
    * throttle and steering angle, calculate the state of the vehicle after a
    * given time.
    * 2023 old bicycle model
    *  @param[in] EgoState ego: Current ego state of car, utfr_msgs::msg::EgoState msg
    *  @param[in] double velocity_cmd: Acceleration input to car, in m/s^2
    *  @param[in] double steering_cmd: Steering angle of car, in radians
    *  @param[in] double dt: Change in time to calcuate new position, in seconds
    *  @returns utfr_msgs::msg::EgoState of vehicle's estimated state
  */
  utfr_msgs::msg::EgoState forwardPropagate(const utfr_msgs::msg::EgoState& ego, 
    const double velocity_cmd, const double steering_cmd, const double dt);

  /*! Main EKF function.
   *  The main EKF loop. Takes in measurement data for the EKF and performes a
   *  single update step.
   *  @param[in] imu_data sensor_msgs::msg::Imu&, IMU data.
   *  @param[in] gps_data ?, GPS data. TODO: Figure out GPS return type
   *  @param[in] vehicle_model_data utfr_msgs::msg::EgoState&, vehicle model
   * data.
   */
  void EKF();

  // Publishers
  rclcpp::Publisher<utfr_msgs::msg::EgoState>::SharedPtr
      state_estimation_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;

  // Subscribers
  // SensorCAN handles GPS, IMU, and wheel/steering speed data!
  rclcpp::Subscription<utfr_msgs::msg::SensorCan>::SharedPtr
      sensorcan_subscriber_;

  // Global variables
  utfr_msgs::msg::EgoState current_state_; // Estimated state of the vehicle
  double kalman_gain_;                     // Kalman gain
};
} // namespace ekf
} // namespace utfr_dv
