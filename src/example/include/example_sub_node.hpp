/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: example_sub_node.hpp
* auth: Kelvin Cui
* desc: example subscriber node class header for ros2 template
*/

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// System Requirements
#include <chrono>
#include <functional>
#include <string>

// Message Requirements
#include <std_msgs/msg/string.hpp>
#include <utfr_common/topics.hpp>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace example {

class ExampleSubNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, and initSubscribers.
   */
  ExampleSubNode();

private:
  /*! Initialize and load params from config.yaml:
   *
   *  echo_string : std::string :
   *    String to be printed to console with each new incoming message
   */
  void loadParams();

  /*! Initialize Subscribers:
   *
   *  example_subscriber_:
   *    msg: std_msgs::String, topic: kExample
   */
  void initSubscribers();

  /*! Callback function for example_subscriber_
   *
   *  @param[in] msg std_msgs::msg::String latest incoming message
   *    print incoming message to console
   */
  void exampleCB(const std_msgs::msg::String &msg);

  // Publishers, Subscribers and Timers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr example_subscriber_;

  // Parameters
  std::string echo_string_;
};

} // namespace example
} // namespace utfr_dv