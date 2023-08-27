/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: controls_node.hpp
* auth: Youssef Elhadad
* desc: controllers publisher and subscribers node header (ros 2)
*/
#pragma once

#include <controller/pid_controller.hpp>
#include <controller/pure_pursuit_controller.hpp>

// System Requirements
#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <string>

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// UTFR Common Requirements
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Message Requirements
#include <std_msgs/msg/string.hpp>
#include <utfr_msgs/msg/control_cmd.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/jetson.hpp>
#include <utfr_msgs/msg/target_state.hpp>

// Misc Requirements
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace controls {

#define PI 3.1415926535
#define RadToDeg(ang) ang * 180 / PI
#define DegToRad(ang) ang *PI / 180

class ControlsNode : public rclcpp::Node {
public:
  ControlsNode();

private:
  /*! Load params from config file
   *
   *  wheelbase_ : double :
   *    Length of the car
   *
   *  lookahead_distance_scaling_factor: double :
   *    contains the lookahead distance scaling factor
   *
   *  str_ctrl_params_ : std::vector<double> :
   *    parameters for steering PID controller
   *
   *  thr_ctrl_params : double :
   *    parameters for throttle PID controller
   *
   *  brk_ctrl_params : double :
   *    parameters for braking PID controller
   *
   *  update_rate : double :
   *    Publish rate in [ms] for example_publisher_
   */
  void initParams();

  /*! Initialize Publishers:
   *
   *  control_cmd_publisher_ : utfr_msgs::ControlCmd, topic: kControlCmd
   *    utfr_msgs to be published
   *
   *  heartbeat_publisher : utfr_msgs::msg::Heartbeat, topic:
   * kControlSystemsHeartbeat utfr_msgs to be published
   *
   */
  void initPublishers();

  /*! Initialize Subscribers:
   *
   *  target_state_subscriber_:
   *  msg: utfr_msgs::TargetState, topic: kTargetState,
   *  callback: targetStateCB
   *
   *  ego_state_subscriber_:
   *  msg: utfr_msgs::EgoState, topic: kEgoState,
   *  callback: egoStateCB
   *
   *  jetson_subscriber_:
   *  msg: utfr_msgs::msg::Jetson, topic: kJetson,
   *  callback: jetsonCB
   *
   */
  void initSubscribers();

  /*! Initialize Timers:
   *
   *  main_timer_:
   *  Calls callback every update_rate_ seconds
   *  callback: ControlsNode::timerCB
   *
   */
  void initTimers();

  /*! Initialize Pure Pursuit Controller by sending wheelbase and ld_sf params.
   */
  void initController();

  /*! Setup Heartbeat message with appropriate module name and update rate.
   */
  void initHeartbeat();

  /*! Send Heartbeat on every timer loop.
   *
   *  @param[in] status current module status, using Heartbeat status enum.
   */
  void publishHeartbeat(const int status);

  /*! Target State callback function for target_state_subscriber_
   *
   *  @param[in] msg utfr_msgs::TargetState incoming message
   */
  void targetStateCB(const utfr_msgs::msg::TargetState &msg);

  /*! Ego State callback function for ego_state_subscriber_
   *
   *  @param[in] msg utfr_msgs::EgoState incoming message
   */
  void egoStateCB(const utfr_msgs::msg::EgoState &msg);

  /*! Jetson callback function for jetson_subscriber_
   *
   *  @param[in] msg utfr_msgs::Jetson incoming message
   */
  void jetsonCB(const utfr_msgs::msg::Jetson &msg);

  /*! Primary callback loop for Controllers.
   *  Calculates steering angle command through pure pursuit controller class.
   *  Propgates velocity target from target_state.
   *  Publishes control commands.
   */
  void timerCB();

  // Publisher
  rclcpp::Publisher<utfr_msgs::msg::ControlCmd>::SharedPtr
      control_cmd_publisher_;

  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;

  PurePursuitControllerUPtr pure_pursuit_{nullptr};
  PIDControllerUPTr steering_pid_{nullptr};
  PIDControllerUPTr throttle_pid_{nullptr};
  PIDControllerUPTr braking_pid_{nullptr};

  // Subscriber
  rclcpp::Subscription<utfr_msgs::msg::TargetState>::SharedPtr
      target_state_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      ego_state_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::Jetson>::SharedPtr jetson_subscriber_;

  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Time ros_time_;

  // Params
  double update_rate_;
  double wheelbase_;
  double lookahead_distance_;
  double target_radius_;
  int hil_;
  int test_;
  int target_steer_test_;
  std::vector<double> str_ctrl_params_;
  std::vector<double> thr_ctrl_params_;
  std::vector<double> brk_ctrl_params_;

  // Callback
  utfr_msgs::msg::TargetState::SharedPtr target_state_{nullptr};
  utfr_msgs::msg::EgoState::SharedPtr ego_state{nullptr};
  utfr_msgs::msg::ControlCmd control_cmd_;
  utfr_msgs::msg::Jetson jetson_msg_;
  utfr_msgs::msg::Heartbeat heartbeat_;
};

} // namespace controls
} // namespace utfr_dv
