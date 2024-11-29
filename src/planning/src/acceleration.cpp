#include <center_path_node.hpp>

namespace utfr_dv {
namespace center_path {

void CenterPathNode::timerCBAccel() {
  if (as_state != utfr_msgs::msg::SystemStatus::AS_STATE_DRIVING) {
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

    int left_size = cone_detections_->left_cones.size();
    int right_size = cone_detections_->right_cones.size();
    int large_orange_size = cone_detections_->large_orange_cones.size();

    if (curr_sector_ == EventState::ACCEL_STRAIGHT && left_size == 0 &&
        right_size == 0 && large_orange_size == 0) {
      curr_sector_ = EventState::ACCEL_FINISHED;
      RCLCPP_INFO(this->get_logger(), "Accel ended due to cone detections.");
    }

    if (curr_sector_ == EventState::ACCEL_STRAIGHT && 
        total_distance_traveled_ > 80) {  // accel length 75 m
      curr_sector_ = EventState::ACCEL_FINISHED;
      RCLCPP_INFO(this->get_logger(), "Accel ended due to distance traveled.");
    }

    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
    publishLapTime();
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

utfr_msgs::msg::ParametricSpline CenterPathNode::getAccelPath() {
  std::vector<utfr_msgs::msg::Cone> all_cones;

  if (cone_map_local_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Cone Map is empty");
    utfr_msgs::msg::ParametricSpline path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "ground";
    path.x_params = {0, 0, 0, 0, 0, 0};
    path.y_params = {0, 0, 0, 0, 0, 0};
    path.lap_count = curr_sector_;
    return path;
  }

  if (curr_sector_ == EventState::ACCEL_STRAIGHT) {
    all_cones.insert(all_cones.end(), cone_map_local_->left_cones.begin(),
                     cone_map_local_->left_cones.end());
    all_cones.insert(all_cones.end(), cone_map_local_->right_cones.begin(),
                     cone_map_local_->right_cones.end());
  }
  all_cones.insert(all_cones.end(), cone_map_local_->large_orange_cones.begin(),
                   cone_map_local_->large_orange_cones.end());
  all_cones.insert(all_cones.end(), cone_map_local_->small_orange_cones.begin(),
                   cone_map_local_->small_orange_cones.end());

  std::sort(
      all_cones.begin(), all_cones.end(),
      [this](const utfr_msgs::msg::Cone &a, const utfr_msgs::msg::Cone &b) {
        return this->coneDistComparitor(a, b);
      });

  int numOfCones = all_cones.size();

  bool found1 = false;
  double highest1 = 0.0, best_m1 = NAN, best_c1 = NAN;
  for (int i = 0; i < numOfCones; i++) {
    for (int j = i + 1; j < numOfCones; j++) {
      utfr_msgs::msg::ConeMap test_cones;
      utfr_msgs::msg::Cone test_cone;
      test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;
      test_cone.pos.x = all_cones[i].pos.x;
      test_cone.pos.y = all_cones[i].pos.y;
      test_cones.left_cones.push_back(test_cone);
      test_cone.pos.x = all_cones[j].pos.x;
      test_cone.pos.y = all_cones[j].pos.y;
      test_cones.left_cones.push_back(test_cone);

      std::tuple<double, double> test_line =
          util::accelLLS(test_cones.left_cones);

      double m = std::get<0>(test_line);
      double c = std::get<1>(test_line);

      if (abs(m) >= 1.0) continue;
      double count = 0;
      int cone_type = utfr_msgs::msg::Cone::UNKNOWN;
      if (all_cones[i].type == all_cones[j].type) {
        count += 1;
        cone_type = all_cones[i].type;
      }
      for (int k = 0; k < numOfCones; k++) {
        double numerator = abs(m * all_cones[k].pos.x - all_cones[k].pos.y + c);
        double denominator = sqrt(m * m + 1);
        double dist = numerator / denominator;
        if (dist >= 0.5) continue;
        count += 1;
        if (all_cones[k].type == cone_type) count += 1;
      }
      if (highest1 <= count) {
        highest1 = count;
        best_m1 = m;
        best_c1 = c;
        found1 = true;
      }
    }
  }

  if(!found1){
    utfr_msgs::msg::ParametricSpline path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "ground";
    path.x_params = {0, 0, 0, 0, 0, 0};
    path.y_params = {0, 0, 0, 0, 0, 0};
    path.lap_count = curr_sector_;
    return path;
  }

  bool found2 = false;
  double highest2 = 0.0, best_m2 = NAN, best_c2 = NAN;
  for (int i = 0; i < numOfCones; i++) {
    for (int j = i + 1; j < numOfCones; j++) {
      utfr_msgs::msg::ConeMap test_cones;
      utfr_msgs::msg::Cone test_cone;
      test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;
      test_cone.pos.x = all_cones[i].pos.x;
      test_cone.pos.y = all_cones[i].pos.y;
      test_cones.left_cones.push_back(test_cone);
      test_cone.pos.x = all_cones[j].pos.x;
      test_cone.pos.y = all_cones[j].pos.y;
      test_cones.left_cones.push_back(test_cone);

      std::tuple<double, double> test_line =
          util::accelLLS(test_cones.left_cones);

      double m = std::get<0>(test_line);
      double c = std::get<1>(test_line);

      if (abs(m - best_m1) >= 0.2 || abs(c - best_c1) <= 2.5) continue;
      double count = 0;
      int cone_type = utfr_msgs::msg::Cone::UNKNOWN;
      if (all_cones[i].type == all_cones[j].type) {
        count += 1;
        cone_type = all_cones[i].type;
      }
      for (int k = 0; k < numOfCones; k++) {
        double numerator = abs(m * all_cones[k].pos.x - all_cones[k].pos.y + c);
        double denominator = sqrt(m * m + 1);
        double dist = numerator / denominator;
        if (dist >= 0.5) continue;
        count += 1;
        if (all_cones[k].type == cone_type) count += 0.2;
      }
      if (highest2 <= count) {
        highest2 = count;
        best_m2 = m;
        best_c2 = c;
        found2 = true;
      }
    }
  }

  double final_m, final_c;

  if (found1 && found2) {
    final_m = (best_m1 + best_m2) / 2;
    final_c = (best_c1 + best_c2) / 2;
  } 
  else { // if both aren't found then only first is found
    final_m = best_m1;
    if (best_c1 > 0) final_c = best_c1 - 1.5;
    else final_c = best_c1 + 1.5;
  }

  if (curr_sector_ == EventState::ACCEL_STRAIGHT) {
    geometry_msgs::msg::PolygonStamped accel_path_msg;

    accel_path_msg.header.stamp = this->get_clock()->now();
    accel_path_msg.header.frame_id = "ground";
    geometry_msgs::msg::Point32 point;
    point.x = 0;
    point.y = -final_c;
    point.z = 0;
    accel_path_msg.polygon.points.push_back(point);
    point.x = 15;
    point.y = -(final_m * 15 + final_c);
    point.z = 0;
    accel_path_msg.polygon.points.push_back(point);

    accel_path_publisher_->publish(accel_path_msg);
  }

  utfr_msgs::msg::ParametricSpline path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "ground";
  path.x_params = {0, 0, 0, 0, 1, 0};
  path.y_params = {0, 0, 0, 0, final_m, final_c};
  path.lap_count = curr_sector_;
  return path;
}

}  // namespace center_path
}  // namespace utfr_dv