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
#include <cmath>
#include <fstream>
#include <functional>
#include <random>
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string>
#include <vector>

// Message Requirements
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
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

#include "vehicle_params.hpp"

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

  /*! Setup Heartbeat message with appropriate module name and update rate.
   */
  void initHeartbeat();

  /*! Send Heartbeat on every timer loop.
   *
   *  @param[in] status current module status, using Heartbeat status enum.
   */
  void publishHeartbeat(const int status);

  enum class HeartBeatState {
    NOT_READY = 1,
    READY = 2,
    ACTIVE = 3,
    ERROR = 4,
    FINISH = 5
  };

  HeartBeatState heartbeat_state_;
  double heartbeat_rate_;

  /*! CAN sensor callback function
   */
  void sensorCB(const utfr_msgs::msg::SensorCan msg);

  /*! GPS sensor callback function
   */
 // void gpsCB(const nav_msgs::msg::Odometry msg);

  /*! IMU sensor callback function
   */
  //void imuCB(const sensor_msgs::msg::Imu msg);

  /*! Implement a kinematic / dynamic vehicle model
   *  Use the throttle, brake, and steering angle to update the vehicle model
   * state
   *  @param[in] throttle float&, throttle data
   *  @param[in] brake float&, braking data
   *  @param[in] steering_angle float&, steering angle, in radians
   *  @param[out] state geometry_msgs::msg::EgoState&, estimated state via the
   * vehicle model
   */

  void dynamicBicycleModel(const float &throttle, const float &brake,
                           const float &steering_angle, const double dt);

  void kinematicBicycleModel(const float &throttle, const float &brake,
                             const float &steering_angle, const double dt);

  /*! Given the vehicle's current state, and a collection of inputs like
   * throttle and steering angle, calculate the state of the vehicle after
   * asensorcan_subscriber_ given time. 2023 old bicycle model
   *  @param[in] EgoState ego: Current ego state of car,
   * utfr_msgs::msg::EgoState msg
   *  @param[in] double velocity_cmd: Acceleration input to car, in m/s^2
   *  @param[in] double steering_cmd: Steering angle of car, in radians
   *  @param[in] double dt: Change in time to calcuate new position, in seconds
   *  @returns utfr_msgs::msg::EgoState of vehicle's estimated state
   */
  utfr_msgs::msg::EgoState forwardPropagate(const utfr_msgs::msg::EgoState &ego,
                                            const double velocity_cmd,
                                            const double steering_cmd,
                                            const double dt);

  /* Given an acceleration input, perform the state extrapolation
   *  @param[in] double accel_cmd: Acceleration input to car, in m/s^2
   *  @param[in] double steering_cmd: Steering angle of car, in radians
   *  @param[in] double dt: Change in time to calcuate new position, in seconds
   *  @returns utfr_msgs::msg::EgoState of vehicle's estimated state
   */
  utfr_msgs::msg::EgoState
  extrapolateState(const sensor_msgs::msg::Imu imu_data, const double dt);

  /* Given a GPS message, perform a measurement update step
   *  @param[in] double x: x position of car, in meters
   *  @param[in] double y: y position of car, in meters
   *  @param[in] double yaw: yaw angle of car, in radians
   *  @returns utfr_msgs::msg::EgoState of vehicle's estimated state
   */

  std::vector<double> lla2ecr(const std::vector<double> &inputVector);
  /*  Helper function for lla2enu: converts lla to ecr
   *  @param[in] std::vector<double>& inputVector: vector with 3 elements: Lat,
   * Lon, Alt in RADIANS, RADIANS, meters
   *  @returns std::vector<double> a vector with 3 elements in ecr
   */
  void ecr2enu(double &x, double &y, double &z, std::vector<double> &datum_lla);
  /*  Helper function for lla2enu: modifys x y and z to their enu value relative
   * to the datum
   *  @param[in] std::vector<double>& datum_lla position vector with 3 elements:
   * Lat, Lon, Alt in RADIANS, RADIANS, meters
   *  @param[in] double& x ecr x value
   *  @param[in] double& y ecr y value
   *  @param[in] double& z ecr z value
   */
  std::vector<double> lla2enu(const std::vector<double> &inputVector);
  /* Given an lla input returns enu relative to datum_lla
   * The first call sets datum_lla and returns {0,0,0}
   *  @param[in] std::vector<double>& inputVector: vector with 3 elements: Lat,
   * Lon, Alt in RADIANS, RADIANS, meters
   *  @returns std::vector<double> a vector with 3 elements east north up all in
   * meters
   */

  utfr_msgs::msg::EgoState updateState(const double x, const double y,
                                       const double yaw);

  /*! Primary callback function
   */
  void timerCB();

  // Publishers
  rclcpp::Publisher<utfr_msgs::msg::EgoState>::SharedPtr ego_state_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;

  // Subscribers
  // SensorCAN handles GPS, IMU, and wheel/steering speed data!
  rclcpp::Subscription<utfr_msgs::msg::SensorCan>::SharedPtr
      sensorcan_subscriber_;

  //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_subscriber_;

  //rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  // Global variables
  utfr_msgs::msg::EgoState current_state_; // Estimated state of the vehicle
  Eigen::MatrixXd P_;
  std::vector<double> datum_lla;

  VehicleParameters vehicle_params_;
  utfr_msgs::msg::Heartbeat heartbeat_;
  double update_rate_;
  int mapping_mode_;
  int ekf_on_;
  rclcpp::TimerBase::SharedPtr main_timer_;

  rclcpp::Time prev_time_;
};
} // namespace ekf
} // namespace utfr_dv
