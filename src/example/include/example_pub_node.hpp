/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: example_pub_node.hpp
* auth: Kelvin Cui
* desc: example publisher node class header for ros2 template
*/

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// System Requirements
#include <chrono>
#include <functional>
#include <string>

// Message Requirements
#include <std_msgs/msg/string.hpp>

// UTFR Common Requirements
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

namespace utfr_dv {
namespace example {

class ExamplePubNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, initPublishers and initTimers.
   */
  ExamplePubNode();

private:
  /*! Initialize and load params from config.yaml:
   *
   *  publish_string : std::string :
   *    String to be published along with counter of message number
   *
   *  publish_rate : double :
   *    Publish rate in [ms] for example_publisher_
   */
  void initParams();

  /*! Initialize Publishers:
   *
   *  example_publisher_:
   *    msg: std_msgs::String, topic: kExample
   */
  void initPublishers();

  /*! Initialize Timers:
   *
   *  main_timer_: publishes publish_string at publish_rate
        callback: timerCB()
   */
  void initTimers();

  /*! Main callback function for main_timer_;
   *  Creates new std_msgs::String with publish_string, and appends message
   *  count. Increase message count on each call.
   */
  void timerCB();

  // Publishers, Subscribers and Timers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr example_publisher_;
  rclcpp::TimerBase::SharedPtr main_timer_;

  // Parameters
  double publish_rate_;
  std::string publish_string_;

  int counter_;
};

} // namespace example
} // namespace utfr_dv