/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: car_interface_node.hpp
* auth: Youssef Elhadad, Daniel Asadi
* desc: car_interface node class header
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
#include <utfr_msgs/msg/control_cmd.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/sensor_can.hpp>
#include <utfr_msgs/msg/system_status.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Library Requirements
#include <canbus_util.hpp>
#include <heartbeat_monitor.hpp>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace car_interface {

class CarInterface : public rclcpp::Node {
public:
  CarInterface();

private:
  /*! Initialize and load params from config.yaml:
   *
   *  publish_rate_ : double :
   *    Publish rate in [ms] for car_interface_publisher_
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
   *  sensor_can_publisher_:
   *    msg: utfr_msgs::msg::SystemStatus
   *    topic: kSensorCan
   *  system_status_publisher_:
   *    msg: utfr_msgs::msg::SystemStatus
   *    topic: kSystemStatus
   *
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

  /*! Initialize heartbeat monitor
   */
  void initMonitor();

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

  /*! Callback function for heartbeat_subscribers_[module_name]
   *
   *  @param[in] msg utfr_msgs::msg::Heartbeat latest heartbeat
   */
  void heartbeatCB(const utfr_msgs::msg::Heartbeat &msg);

  /*! Callback function for control_cmd_subscriber_
   *
   *  @param[in] msg utfr_msgs::msg::ControlCmd latest message from control
   * systems
   */
  void controlCmdCB(const utfr_msgs::msg::ControlCmd &msg);

  void getSteeringAngleSensorData();

  void getMotorSpeedData();

  void getServiceBrakeData();

  void getEBSPressureData();

  void getWheelspeedSensorData();

  void getIMUData();

  void getSensorCan();

  void getSystemStatus();

  void setSystemStatusAS();

  /*! Callback function for timer
   */
  void timerCB();

  // Publishers, Subscribers and Timers
  rclcpp::Subscription<utfr_msgs::msg::ControlCmd>::SharedPtr
      control_cmd_subscriber_;
  rclcpp::Publisher<utfr_msgs::msg::SensorCan>::SharedPtr sensor_can_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::SystemStatus>::SharedPtr
      system_status_publisher_;
  std::map<std::string,
           rclcpp::Subscription<utfr_msgs::msg::Heartbeat>::SharedPtr>
      heartbeat_subscribers_;
  rclcpp::TimerBase::SharedPtr main_timer_;

  // Parameters
  double update_rate_;
  double heartbeat_tolerance_;
  std::vector<std::string> heartbeat_modules_;

  // Callback Variables
  utfr_msgs::msg::ControlCmd control_cmd_;

  // Published messages
  utfr_msgs::msg::SensorCan sensor_can_;
  utfr_msgs::msg::SystemStatus system_status_;

  // Commands to rest of car
  int16_t steering_rate_cmd_;
  uint8_t braking_cmd_;
  int throttle_cmd_;

  // Steering
  int16_t current_steering_angle_;

  // Motor speed
  double motor_speed_;

  // Braking
  uint16_t asb_pressure_front_;
  uint16_t asb_pressure_rear_;

  uint16_t ebs_pressure_1_;
  uint16_t ebs_pressure_2_;

  // Wheel speed
  double wheelspeed_fl_;
  double wheelspeed_fr_;
  double wheelspeed_rl_;
  double wheelspeed_rr_;

  // IMU
  double imu_;

  // TODO: GNSS/INS

  // AS STATE:
  int res;

  // CAN objects
  CanInterfaceUPtr can1_{nullptr};
  CanInterfaceUPtr can0_{nullptr};
  rclcpp::TimerBase::SharedPtr can_timer_;

  // Heartbeat object
  HeartbeatMonitorUPtr heartbeat_monitor_{nullptr};

  // TO DO: Add drivers and other nodes
  // Heartbeat map
  std::unordered_map<std::string, std::string> heartbeat_topics_map_{
      {"perception", topics::kPerceptionHeartbeat},
      {"mapping", topics::kMappingHeartbeat},
      {"ekf", topics::kEKFHeartbeat},
      {"navigation", topics::kPlanningHeartbeat},
      {"controls", topics::kControlsHeartbeat},
  };
};

} // namespace car_interface
} // namespace utfr_dv
