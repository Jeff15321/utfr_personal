/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: heartbeat_monitor.hpp
* auth: Kelvin Cui
* desc: heartbeat monitor class header - used to verify heartbeats from each
*       node, and ensure all nodes are functioning at the proper rates
*/

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// Message Requirements
#include <utfr_msgs/msg/heartbeat.hpp>

// UTFR Common Requirements
#include <utfr_common/topics.hpp>

namespace utfr_dv {
namespace car_interface {

class HeartbeatMonitor {
public:
  /*! Construct Monitor by initializing the last_heartbeats_ for all the
   *  modules that need to be monitored.
   *
   *  last_heartbeats_ are fomatted as follows:
   *  std::string of module name
   *  tuple of:
   *    {max update duration with tolerance,
   *     last recieved message timestamp,
   *     newest recived message timestamp}
   *
   *    - update duration are initialized to -1 when no messages have been
   * recieved yet
   *
   * @param[in] module_names : vector of the names of all the modules to monitor
   * @param[in] curr_time : current time for initialization
   * @param[in] tolerance : percent tolerance of variation in update rate
   */
  HeartbeatMonitor(const std::vector<std::string> &module_names,
                   const rclcpp::Time &curr_time, const double tolerance);

  /*! Update the last_heartbeats_ map with incoming Heartbeat.
   *  If module name of incoming heartbeat matches an existing module, set the
   *  configured update rate and populate the last recieved message time field
   *
   *  If a module has an error status, change module_error_ to true.
   *
   * @param[in] msg Heartbeat message from module
   */
  void updateHeartbeat(const utfr_msgs::msg::Heartbeat &msg,
                       const rclcpp::Time &curr_time);

  /*! Update the last_heartbeats_ map with the correct list of runnig modules.
   * Checks if the new modules names match existing modules, and removes those
   * that are not in the list.
   *
   * @param[in] modules A vector of module names
   */
  void updateModules(std::vector<std::string> module_names,
                     const rclcpp::Time &curr_time);
  /*! Check if all heartbeats have the correct status, and are within update
   *  rate tolerance.
   *
   * @param[in] curr_time Current Time from Mission Manager Node
   * @returns false if any module:
   *    - Has not yet sent a heartbeat (last recieved message field = -1)
   *
   *    - Duration between last heartbeat and current time exceeds update rate
   *      with tolerance applied
   *
   *    - Has an error status (module_error_ is true)
   *
   *    Returns true otherwise.
   */
  bool verifyHeartbeats(const rclcpp::Time &curr_time);

private:
  std::unordered_map<std::string, std::tuple<double, double, rclcpp::Time>>
      last_heartbeats_;
  bool module_error_ = false;

  double duration_tolerance_;
};

using HeartbeatMonitorUPtr = std::unique_ptr<HeartbeatMonitor>;
using HeartbeatMonitorSPtr = std::shared_ptr<HeartbeatMonitor>;

} // namespace car_interface
} // namespace utfr_dv
