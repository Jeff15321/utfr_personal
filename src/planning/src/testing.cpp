#include <center_path_node.hpp>

namespace utfr_dv {
namespace center_path {

void CenterPathNode::timerCBEBS() {
  if (as_state != utfr_msgs::msg::SystemStatus::AS_STATE_DRIVING){
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }
  try {
    const std::string function_name{"center_path_timerCB:"};

    if (!cone_detections_) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                  "Data not published or initialized yet. Using defaults.");
      return;
    }

    utfr_msgs::msg::ParametricSpline center_path_msg;
    if(use_autocross_for_accel_) center_path_msg = getBestPath();
    else center_path_msg = getAccelPath();

    center_path_publisher_->publish(center_path_msg);

    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void CenterPathNode::timerCBAS() {
  if (as_state != utfr_msgs::msg::SystemStatus::AS_STATE_DRIVING){
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }
  publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
}

} // namespace center_path
} // namespace utfr_dv