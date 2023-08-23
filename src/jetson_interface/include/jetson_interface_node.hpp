/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: jetson_interface_node.hpp
* auth: Youssef Elhadad
* desc: jetson_interface node class header
*/

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// System Requirements
#include <chrono>
#include <functional>
#include <pthread.h>
#include <string>

// Message Requirements
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <utfr_msgs/msg/cone_detections.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Message Requirements
#include <utfr_msgs/msg/control_cmd.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/jetson.hpp>
#include <utfr_msgs/msg/system_status.hpp>

// CAN Interface
#include <can_interface.hpp>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace jetson_interface {

class JetsonInterface : public rclcpp::Node {
public:
  JetsonInterface();

private:
  /*! Initialize and load params from config.yaml:
   *
   *  publish_rate_ : double :
   *    Publish rate in [ms] for jetson_interface_publisher_
   */
  void initParams();

  /*! Initialize Subscribers:
   *
   *  control_subscriber_:
   *    msg: utfr_msgs::msg::ControlCmd, topic: kControlCmd
   *    callback: controlCmdCB
   *  system_status_subscriber_:
   *    msg: utfr_msgs::msg::SystemStatus, topic: kSystemStatus
   *    callback: systemStatusCB
   */
  void initSubscribers();

  /*! Initialize Publishers:
   *
   *  jetson_publisher_:
   *    msg: utfr_msgs::msg::Teensy, topic: kTeensy
   */
  void initPublishers();

  /*! Setup Heartbeat message with appropriate module name and update rate.
   */
  void initHeartbeat();

  /*! Send Heartbeat on every timer loop.
   *
   *  @param[in] status current module status, using Heartbeat status enum.
   */
  void publishHeartbeat(const int status);

  /*! Enable the CAN Bus and connect in this function
   *
   *  Initialize the CAN connection in the Jetson
   */
  void initCAN();

  /*! Config Sensors
   *
   */
  void initSensors();

  /*! Initialize Timers:
   *
   *  main_timer_:
   *  Calls callback every update_rate_ seconds
   *  callback: ControlSystemsNode::timerCB
   *
   */
  void initTimers();

  /*! Callback function for control_cmd_subscriber_
   *
   *  @param[in] msg utfr_msgs::msg::ControlCmd latest message from control
   * systems
   */
  void controlCmdCB(const utfr_msgs::msg::ControlCmd &msg);

  /*! Callback function for system_status_subscriber_
   *
   *  @param[in] msg utfr_msgs::msg::SystemStatus latest status of system
   */
  void systemStatusCB(const utfr_msgs::msg::SystemStatus &msg);

  /*! Callback function for timer
   */
  void timerCB();

  // Publishers, Subscribers and Timers
  rclcpp::Subscription<utfr_msgs::msg::ControlCmd>::SharedPtr
      control_cmd_subscriber_;
  rclcpp::Publisher<utfr_msgs::msg::Jetson>::SharedPtr jetson_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::SystemStatus>::SharedPtr
      system_status_publisher_;
  rclcpp::Subscription<utfr_msgs::msg::SystemStatus>::SharedPtr
      system_status_subscriber_;
  rclcpp::TimerBase::SharedPtr main_timer_;

  // Parameters
  double update_rate_;

  // Callback Variable
  utfr_msgs::msg::ControlCmd control_cmd_;
  utfr_msgs::msg::SystemStatus system_status_;
  utfr_msgs::msg::Heartbeat heartbeat_;

  // Published jetson message
  utfr_msgs::msg::SensorCan jetson_cmd_;
  utfr_msgs::msg::SystemStatus as_state_;

  // Commands to rest of car
  int16_t steering_rate_cmd_;
  uint8_t braking_cmd_;
  int throttle_cmd_;

  // Information Recieved:
  // Steering
  int16_t current_steering_angle_;

  // Velocity
  double motor_speed_;
  //*GPS*
  geometry_msgs::msg::Vector3 imu_accel_;
  double roll_rate_;
  double yaw_rate_;

  // Braking:
  uint16_t front_brake_pressure_;
  uint16_t rear_brake_pressure_;

  // AS STATE:
  int res;

  // CAN objects
  CanInterfaceUPtr can1_{nullptr};
  CanInterfaceUPtr can0_{nullptr};

  rclcpp::TimerBase::SharedPtr can_timer_;
};

} // namespace jetson_interface
} // namespace utfr_dv
