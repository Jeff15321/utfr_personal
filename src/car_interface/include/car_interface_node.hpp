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
#include <sensor_msgs/msg/imu.hpp>
#include <utfr_msgs/msg/control_cmd.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/sensor_can.hpp>
#include <utfr_msgs/msg/system_status.hpp>
#include <utfr_msgs/msg/target_state.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Library Requirements
#include <canbus_util.hpp>
#include <heartbeat_monitor.hpp>

#define MAX_BRK_PRS 1600 // PSI
#define MAX_THROTTLE 200 // Nm, from kProcessedTHrottleMax variable in FC code

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

  /*! Enable the CAN Bus and connect in this function
   *
   *  Initialize the CAN connection in the Jetson
   */
  void initCAN();

  /*! Initialize heartbeat monitor
   */
  void initMonitor();

  /*! Initialize Timers:
   *
   *  main_timer_:
   *  Calls callback every update_rate_ seconds
   *  callback: ControlsNode::timerCB
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
   *  @param[in] msg utfr_msgs::msg::ControlCmd latest message from controls
   */
  void controlCmdCB(const utfr_msgs::msg::ControlCmd &msg);

  /*! Callback function for ego_state_subscriber_
   *
   *  @param[in] msg utfr_msgs::msg::EgoState latest message from mapping
   */
  void EgoStateCB(const utfr_msgs::msg::EgoState &msg);

  /*! Callback function for target_state_subscriber_
   *
   *  @param[in] msg utfr_msgs::msg::TargetState latest message from planning
   */
  void TargetStateCB(const utfr_msgs::msg::TargetState &msg);

  void getSteeringAngleSensorData(); // TODO: function desc.

  void getMotorSpeedData(); // TODO: function desc.

  void getMotorTorqueData(); // TODO: function desc.

  void getServiceBrakeData(); // TODO: function desc.

  void getWheelspeedSensorData(); // TODO: function desc.

  void getIMUData(); // TODO: function desc.

  void getSensorCan(); // TODO: function desc.

  void getDVState(); // TODO: function desc.

  void setDVLogs(); // TODO: function desc.

  void setDVStateAndCommand(); // TODO: function desc.

  void launchMission(); // TODO: function desc.

  void shutdownNodes(); // TODO: function desc.

  /*! Callback function for timer
   */
  void timerCB();

  // Publishers, Subscribers and Timers
  rclcpp::Subscription<utfr_msgs::msg::ControlCmd>::SharedPtr
      control_cmd_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      ego_state_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::TargetState>::SharedPtr
      target_state_subscriber_;

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
  utfr_msgs::msg::EgoState ego_state_;
  utfr_msgs::msg::TargetState target_state_;

  // Published messages
  utfr_msgs::msg::SensorCan sensor_can_;
  utfr_msgs::msg::SystemStatus system_status_;

  // TODO: Change global to local vars
  // Commands to rest of car
  int steering_cmd_;
  uint8_t braking_cmd_;
  int throttle_cmd_;
  int testing_;
  // TODO: GNSS/INS

  // State vars
  bool launched_ = false;
  bool shutdown_ = false;
  bool cmd_ = false;
  bool finished_ = false;
  enum DV_PC_STATE {
    OFF = 1,
    READY = 2,
    DRIVING = 3,
    EMERGENCY = 4,
    FINISH = 5
  };
  uint8_t dv_pc_state_;

  // CAN objects
  CanInterfaceUPtr can1_{nullptr};
  CanInterfaceUPtr can0_{nullptr};

  rclcpp::TimerBase::SharedPtr can_timer_;

  // Heartbeat object
  HeartbeatMonitorUPtr heartbeat_monitor_{nullptr};

  // TODO: Add drivers and other nodes and multiple planning nodes
  // Heartbeat map
  std::unordered_map<std::string, std::string> heartbeat_topics_map_{
      {"perception", topics::kPerceptionHeartbeat},
      {"ekf", topics::kEKFHeartbeat},
      {"mapping_build", topics::kMappingBuildHeartbeat},
      {"mapping_compute", topics::kMappingComputeHeartbeat},
      {"planning", topics::kPlanningHeartbeat},
      {"controls", topics::kControlsHeartbeat},
  };
};

} // namespace car_interface
} // namespace utfr_dv
