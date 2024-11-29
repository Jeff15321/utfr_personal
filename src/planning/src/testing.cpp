#include <center_path_node.hpp>

namespace utfr_dv {
namespace center_path {

void CenterPathNode::timerCBEBS() {
  if (as_state != 3){
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

    std::vector<double> accel_path = getAccelPath();

    utfr_msgs::msg::ParametricSpline center_path_msg;

    double m = accel_path[0];
    double c = accel_path[1];

    std::vector<double> x = {0, 0, 0, 0, 1, 0};
    std::vector<double> y = {0, 0, 0, 0, m, c};

    center_path_msg.header.stamp = this->get_clock()->now();
    center_path_msg.header.frame_id = "ground";
    center_path_msg.x_params = x;
    center_path_msg.y_params = y;
    center_path_msg.lap_count = curr_sector_;

    center_path_publisher_->publish(center_path_msg);

    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void CenterPathNode::timerCBAS() {
  if (as_state != 3){
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }
  publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
}

} // namespace center_path
} // namespace utfr_dv