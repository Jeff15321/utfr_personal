/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: heartbeat_monitor.cpp
* auth: Kelvin Cui
* desc: heartbeat monitor class - used to verify heartbeats from each
*       node, and ensure all nodes are functioning at the proper rates
*/

#include <heartbeat_monitor.hpp>

namespace utfr_dv {
namespace car_interface {

HeartbeatMonitor::HeartbeatMonitor(const std::vector<std::string> &module_names,
                                   const rclcpp::Time &curr_time,
                                   const double tolerance) {

  duration_tolerance_ = tolerance;

  std::tuple<double, double, rclcpp::Time> unintitalized_values = {-1.0, 0.0,
                                                                   curr_time};

  for (const auto &name : module_names) {

    last_heartbeats_[name] = unintitalized_values;
  }
}

void HeartbeatMonitor::updateHeartbeat(const utfr_msgs::msg::Heartbeat &msg,
                                       const rclcpp::Time &curr_time) {
  const std::string function_name{"updateHeartbeat: "};

  std::string module_name = msg.module.data;
  auto search = last_heartbeats_.find(module_name);

  if (search == last_heartbeats_.end()) { // Module not found :
    RCLCPP_WARN(rclcpp::get_logger("heartbeat_monitor"),
                "%s Incoming Heartbeat not listed : %s", function_name.c_str(),
                module_name.c_str());
    return;
  }

  if (msg.status == utfr_msgs::msg::Heartbeat::ERROR) {
    RCLCPP_ERROR(rclcpp::get_logger("heartbeat_monitor"),
                 "%s Incoming Heartbeat status Fatal : %s",
                 function_name.c_str(), module_name.c_str());
    module_error_ = true;
    return;
  }

  rclcpp::Time new_stamp = msg.header.stamp;
  double update_rate = msg.update_rate;
  double max_update_duration = update_rate * duration_tolerance_;

  if (std::get<0>(last_heartbeats_[module_name]) == -1.0) {
    // Calculate difference between clocks:
    double diff = (curr_time - new_stamp).nanoseconds() / 1000000;
    last_heartbeats_[module_name] = {max_update_duration, diff, new_stamp};
    return;
  }

  double diff = std::get<1>(last_heartbeats_[module_name]);
  last_heartbeats_[module_name] = {max_update_duration, diff, new_stamp};
}

bool HeartbeatMonitor::verifyHeartbeats(const rclcpp::Time &curr_time) {
  const std::string function_name{"verifyHeartbeats: "};

  if (module_error_) {
    RCLCPP_ERROR(rclcpp::get_logger("heartbeat_monitor"),
                 "%s Fatal heartbeat detected!", function_name.c_str());
    return false;
  }

  for (auto &element : last_heartbeats_) {
    double max_duration;
    double const_diff;
    rclcpp::Time new_stamp;
    std::tie(max_duration, const_diff, new_stamp) = element.second;

    if (max_duration == -1 || max_duration == -2) {
      RCLCPP_ERROR(rclcpp::get_logger("heartbeat_monitor"),
                   "%s Module %s heartbeat not yet started!",
                   function_name.c_str(), element.first.c_str());
      return false;
    }

    double current_diff = (curr_time - new_stamp).nanoseconds() / 1000000;
    double latest_duration = current_diff - const_diff;
    RCLCPP_INFO(rclcpp::get_logger("heartbeat_monitor"),
                "%s Module %s duration %f, max duration %f!",
                function_name.c_str(), element.first.c_str(), latest_duration,
                max_duration);

    if (latest_duration > max_duration) {
      RCLCPP_ERROR(rclcpp::get_logger("heartbeat_monitor"),
                   "%s Module %s exceed max duration!", function_name.c_str(),
                   element.first.c_str());
      return false;
    }
  }

  return true;
}

} // namespace car_interface
} // namespace utfr_dv
