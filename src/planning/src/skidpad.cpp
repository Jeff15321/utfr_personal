#include <center_path_node.hpp>

namespace utfr_dv {
namespace center_path {

void CenterPathNode::timerCBSkidpad() {
  if (as_state != 3){
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }
  const std::string function_name{"center_path_timerCB:"};
  try {
    if (cone_detections_ == nullptr || ego_state_ == nullptr) {
      RCLCPP_WARN(get_logger(),
                  "%s Either cone detections or ego state is empty.",
                  function_name.c_str());
      return;
    }
    if (global_path_ && skidpadTransform_ != nullptr) {
      RCLCPP_INFO(this->get_logger(), "USING GLOBAL PATH");
      this->nextWaypoint();
    } else {
      RCLCPP_INFO(this->get_logger(), "USING LOCAL PATH");
      if (cone_map_local_ == nullptr) {
        RCLCPP_WARN(get_logger(), "%s Cone Map is empty",
                    function_name.c_str());
        return;
      }
      skidPadFit();
    }
    if(colourblind_){
      skidpadLapCounterColourblind();
    }
    else{
      skidpadLapCounter();
    }
    publishLapTime();
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void CenterPathNode::skidpadLapCounter() {
  rclcpp::Time curr_time = this->get_clock()->now();
  double time_diff = (curr_time - last_time).seconds();

  int large_orange_cones_size = cone_detections_->large_orange_cones.size();
  int small_orange_cones_size = cone_detections_->small_orange_cones.size();
  int left_size = cone_detections_->left_cones.size();
  int right_size = cone_detections_->right_cones.size();

  double average_distance_to_cones = 0.0;

  for (int i = 0; i < large_orange_cones_size; i++) {
    average_distance_to_cones += util::euclidianDistance2D(
        cone_detections_->large_orange_cones[i].pos.x, 0.0,
        cone_detections_->large_orange_cones[i].pos.y, 0.0);
  }

  average_distance_to_cones =
      average_distance_to_cones / large_orange_cones_size;

  if (large_orange_cones_size == 4) {
    found_4_large_orange = true;
  }

  switch (curr_sector_) {
  case EventState::SMALL_ORANGE_STRAIGHT:
    if (large_orange_cones_size > 3 && small_orange_cones_size < 3) {
      curr_sector_ += 1;
    }
    break;

  case EventState::LARGE_ORANGE_STRAIGHT:
    if (large_orange_cones_size < 3) {
      curr_sector_ += 1;
    }
    break;

  case EventState::RIGHT_CIRCLE_FIRST:
  case EventState::RIGHT_CIRCLE_SECOND:
  case EventState::LEFT_CIRCLE_FIRST:
  case EventState::LEFT_CIRCLE_SECOND:
    if (time_diff > 5.0 ) {
      if (loop_closed_) {
        if (!lock_sector_ && checkPassedDatum(getSkidpadDatum(*cone_map_global_), *ego_state_)) {
          last_time = curr_time;
          curr_sector_ += 1;
          lock_sector_ = true;
          RCLCPP_INFO(this->get_logger(), "Lap incremented: Global trigger");
        }
      } else {
        if (!lock_sector_ && found_4_large_orange &&
            large_orange_cones_size < 4 && 
            average_distance_to_cones < 5.0) {
          last_time = curr_time;
          curr_sector_ += 1;
          lock_sector_ = true;
          RCLCPP_INFO(this->get_logger(), "Lap incremented: Local trigger");
        }
      }
    }

    if (found_4_large_orange && lock_sector_ && large_orange_cones_size == 0 &&
        time_diff > 5.0) {
      lock_sector_ = false;
      found_4_large_orange = false;
    }
    // if (loop_closed_ && checkPassedDatum(getSkidpadDatum(*cone_map_global_),
    // *ego_state_)) {
    //   RCLCPP_WARN(this->get_logger(), "Global lap incremented");
    // }
    break;
  case EventState::END_SMALL_ORANGE:
    if (left_size == 0 && right_size == 0) {
      curr_sector_ += 1;
    }
  }
}

void CenterPathNode::skidpadLapCounterColourblind(){
  rclcpp::Time curr_time = this->get_clock()->now();
  double time_diff = (curr_time - last_time).seconds();

  if(curr_sector_ == 10) RCLCPP_INFO(this->get_logger(), "Start Orange Straight");
  else if(curr_sector_ == 11) RCLCPP_INFO(this->get_logger(), "Large Orange Straight");
  else if(curr_sector_ == 12) RCLCPP_INFO(this->get_logger(), "Right Circle 1");
  else if(curr_sector_ == 13) RCLCPP_INFO(this->get_logger(), "Right Circle 2");
  else if(curr_sector_ == 14) RCLCPP_INFO(this->get_logger(), "Left Circle 1");
  else if(curr_sector_ == 15) RCLCPP_INFO(this->get_logger(), "Left Circle 2");
  else if(curr_sector_ == 16) RCLCPP_INFO(this->get_logger(), "End Orange Straight");
  else RCLCPP_INFO(this->get_logger(), "Finished");

  switch (curr_sector_) {
    case EventState::SMALL_ORANGE_STRAIGHT:
    case EventState::LARGE_ORANGE_STRAIGHT:
      if (total_distance_traveled_ > 13){
        curr_sector_ += 1;
        switch_distance = total_distance_traveled_;
        last_time = this->get_clock()->now();
      }
      break;
    case EventState::RIGHT_CIRCLE_FIRST:
    case EventState::RIGHT_CIRCLE_SECOND:
    case EventState::LEFT_CIRCLE_FIRST:
    case EventState::LEFT_CIRCLE_SECOND:
      if (total_distance_traveled_ - switch_distance > 55.0 && time_diff > 2.0){
        curr_sector_ += 1;
        switch_distance = total_distance_traveled_;
        last_time = this->get_clock()->now();
      }
      break;
    case EventState::END_SMALL_ORANGE:
      if (total_distance_traveled_ - switch_distance > 20 && time_diff > 2.0){
        curr_sector_ += 1;
      }
      break;
  }
}

utfr_msgs::msg::EgoState
CenterPathNode::getSkidpadDatum(const utfr_msgs::msg::ConeMap &cone_map) {
  utfr_msgs::msg::EgoState datum;

  if (cone_map.large_orange_cones.size() == 3) {
    double x = cone_map.large_orange_cones[0].pos.x;
    double y = 0.0;
    //do x
    double baseX = x;
    for (int i = 0; i < 2; i++) {
      if (abs(baseX - cone_map.large_orange_cones[i].pos.x) > 0.5) {
        x += cone_map.large_orange_cones[i].pos.x;
        break;
      }
    }
    x = x / 2.0;
    // do y
    for (utfr_msgs::msg::Cone cone : cone_map.large_orange_cones) {
      y += cone.pos.y;
    }
    y = y / 3.0;

    datum.pose.pose.position.x = x;
    datum.pose.pose.position.y = y;
    datum.pose.pose.orientation = util::yawToQuaternion(0.0);
  } else if (cone_map.large_orange_cones.size() >= 4) {
    double x = 0.0;
    double y = 0.0;
    for (utfr_msgs::msg::Cone cone : cone_map.large_orange_cones) {
      x += cone.pos.x;
      y += cone.pos.y;
    }
    datum.pose.pose.position.x = x / cone_map.large_orange_cones.size();
    datum.pose.pose.position.y = y / cone_map.large_orange_cones.size();
    datum.pose.pose.orientation = util::yawToQuaternion(0.0);
  } else {
    datum.pose.pose.position.x = -100.0;
    datum.pose.pose.position.y = -100.0;
    datum.pose.pose.position.z = -100.0;
    datum.pose.pose.orientation = util::yawToQuaternion(0.0);
  }

  return datum;
}

void CenterPathNode::skidPadFit() {

  const std::string function_name{"skidPadFit:"};
  std::tuple<double, double, double, double> left_circle_s, left_circle_l,
      right_circle_s, right_circle_l, left_circle, right_circle;
  double m_left, m_right, c_left, c_right;
  double xc1, yc1, xc2, yc2, r1, r2;
  if (curr_sector_ == 10 || curr_sector_ == 11 || curr_sector_ == 16 ||
      curr_sector_ == 17) {
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

    geometry_msgs::msg::PolygonStamped circleavg;
    circleavg.header.frame_id = "ground";
    circleavg.header.stamp = this->get_clock()->now();
    geometry_msgs::msg::Point32 pointavg;

    pointavg.x = 0;
    pointavg.y = 0;
    pointavg.z = 0;
    circleavg.polygon.points.push_back(pointavg);

    pointavg.x = 10.0;
    pointavg.y = -(m * 10.0 + c);
    pointavg.z = 0;
    circleavg.polygon.points.push_back(pointavg);

    skidpad_path_publisher_avg_->publish(circleavg);
  }

  else if (curr_sector_ == 12 || curr_sector_ == 13) {
    std::tuple<double, double, double, double, double, double> circle;
    if (cone_detections_->large_orange_cones.size() > 0 ||
        cone_detections_->small_orange_cones.size() > 0) {
      circle = skidpadRight();
    } else {
      circle = skidpadMain();
    }

    xc1 = std::get<0>(circle);
    yc1 = std::get<1>(circle);
    r1 = std::get<2>(circle);
    xc2 = std::get<3>(circle);
    yc2 = std::get<4>(circle);
    r2 = std::get<5>(circle);

    geometry_msgs::msg::PolygonStamped circleavg;

    circleavg.header.frame_id = "ground";

    circleavg.header.stamp = this->get_clock()->now();

    double b = (xc1 + xc2) / 2.0;
    double k = (yc1 + yc2) / 2.0;
    double r = (small_radius_ + big_radius_) / 2.0;

    geometry_msgs::msg::Point32 pointavg;

    for (int i = 0; i < 50; i++) {
      double cur_ang = static_cast<double>(i) / 50 * 3.1415 / 2;
      double cur_x = b + r * cos(cur_ang);
      double cur_y = k - r * sin(cur_ang);
      pointavg.x = cur_x;
      pointavg.y = -cur_y;
      pointavg.z = 0;
      circleavg.polygon.points.push_back(pointavg);
    }

    for (int i = 50; i >= 0; i--) {
      double cur_ang = static_cast<double>(i) / 50 * 3.1415 / 2;
      double cur_x = b + r * cos(cur_ang);
      double cur_y = k - r * sin(cur_ang);
      pointavg.x = cur_x;
      pointavg.y = -cur_y;
      pointavg.z = 0;
      circleavg.polygon.points.push_back(pointavg);
    }

    skidpad_path_publisher_avg_->publish(circleavg);

    utfr_msgs::msg::ParametricSpline avg_circle_msg;
    avg_circle_msg.header.frame_id = "ground";
    avg_circle_msg.header.stamp = this->get_clock()->now();

    avg_circle_msg.skidpad_params = {b, k, r};
    avg_circle_msg.x_params = {0, 0, 0, 0, 0, 0};
    avg_circle_msg.y_params = {0, 0, 0, 0, 0, 0};
    avg_circle_msg.lap_count = curr_sector_;

    center_path_publisher_->publish(avg_circle_msg);
  }

  else if (curr_sector_ == 14 || curr_sector_ == 15) {
    std::tuple<double, double, double, double, double, double> circle;
    if (cone_detections_->large_orange_cones.size() > 0 ||
        cone_detections_->small_orange_cones.size() > 0) {
      circle = skidpadLeft();
    } else {
      circle = skidpadMain();
    }

    xc1 = std::get<0>(circle);
    yc1 = std::get<1>(circle);
    r1 = std::get<2>(circle);
    xc2 = std::get<3>(circle);
    yc2 = std::get<4>(circle);
    r2 = std::get<5>(circle);

    geometry_msgs::msg::PolygonStamped circleavg;

    circleavg.header.frame_id = "ground";
    circleavg.header.stamp = this->get_clock()->now();

    double b = (xc1 + xc2) / 2.0;
    double k = (yc1 + yc2) / 2.0;
    double r = (small_radius_ + big_radius_) / 2.0;

    geometry_msgs::msg::Point32 pointavg;

    for (int i = 0; i < 50; i++) {
      double cur_ang = static_cast<double>(i) / 50 * 3.1415 / 2;
      double cur_x = b + r * cos(cur_ang);
      double cur_y = k + r * sin(cur_ang);
      pointavg.x = cur_x;
      pointavg.y = -cur_y;
      pointavg.z = 0;
      circleavg.polygon.points.push_back(pointavg);
    }

    for (int i = 50; i >= 0; i--) {
      double cur_ang = static_cast<double>(i) / 50 * 3.1415 / 2;
      double cur_x = b + r * cos(cur_ang);
      double cur_y = k + r * sin(cur_ang);
      pointavg.x = cur_x;
      pointavg.y = -cur_y;
      pointavg.z = 0;
      circleavg.polygon.points.push_back(pointavg);
    }

    skidpad_path_publisher_avg_->publish(circleavg);

    utfr_msgs::msg::ParametricSpline avg_circle_msg;
    avg_circle_msg.header.frame_id = "ground";
    avg_circle_msg.header.stamp = this->get_clock()->now();

    avg_circle_msg.skidpad_params = {b, k, r};
    avg_circle_msg.x_params = {0, 0, 0, 0, 0, 0};
    avg_circle_msg.y_params = {0, 0, 0, 0, 0, 0};
    avg_circle_msg.lap_count = curr_sector_;

    center_path_publisher_->publish(avg_circle_msg);
  }
}

std::tuple<double, double, double, double, double, double>
CenterPathNode::skidpadMain() {
  int turning;
  if (curr_sector_ == 12 || curr_sector_ == 13) {
    turning = 1;
  } else {
    turning = 0;
  }
  std::vector<utfr_msgs::msg::Cone> all_cones;
  all_cones.insert(all_cones.end(), cone_map_local_->left_cones.begin(),
                   cone_map_local_->left_cones.end());
  all_cones.insert(all_cones.end(), cone_map_local_->right_cones.begin(),
                   cone_map_local_->right_cones.end());
  all_cones.insert(all_cones.end(), cone_map_local_->large_orange_cones.begin(),
                   cone_map_local_->large_orange_cones.end());
  std::sort(
      all_cones.begin(), all_cones.end(),
      [this](const utfr_msgs::msg::Cone &a, const utfr_msgs::msg::Cone &b) {
        return this->coneDistComparitor(a, b);
      });

  int all_size = all_cones.size();
  bool find = false;
  double best_xc_small, best_xc_large, best_yc_small, best_yc_large,
      best_r_small, best_r_large, best_xc, best_yc;
  int best = 0;

  for (int i = 0; i < all_size - 2; i++) {
    for (int j = i + 1; j < all_size - 1; j++) {
      for (int k = j + 1; k < all_size; k++) {
        if (i != j && j != k && i != k) {
          utfr_msgs::msg::ConeMap cur_test_right;
          utfr_msgs::msg::Cone cur_test_cone;
          cur_test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;

          cur_test_cone.pos.x = all_cones[i].pos.x;
          cur_test_cone.pos.y = all_cones[i].pos.y;
          (cur_test_right.right_cones).push_back(cur_test_cone);
          cur_test_cone.pos.x = all_cones[j].pos.x;
          cur_test_cone.pos.y = all_cones[j].pos.y;
          (cur_test_right.right_cones).push_back(cur_test_cone);
          cur_test_cone.pos.x = all_cones[k].pos.x;
          cur_test_cone.pos.y = all_cones[k].pos.y;
          (cur_test_right.right_cones).push_back(cur_test_cone);

          std::tuple<double, double, double> circle =
              util::circleLSF(cur_test_right.right_cones);

          double xc = std::get<0>(circle);
          double yc = std::get<1>(circle);
          double r = std::get<2>(circle);
          double closest_radius, other_radius;

          if (xc >= 5.0 || (turning == 1 && yc < 0) ||
              (turning == 0 && yc > 0)) {
            continue;
          }

          if (abs(r - small_radius_) < threshold_radius_) {
            closest_radius = small_radius_;
            other_radius = big_radius_;
          } else if (abs(r - big_radius_) < threshold_radius_) {
            closest_radius = big_radius_;
            other_radius = small_radius_;
          } else {
            closest_radius = r < (small_radius_ + big_radius_) / 2
                                 ? small_radius_
                                 : big_radius_;
            other_radius = r < (small_radius_ + big_radius_) / 2
                               ? big_radius_
                               : small_radius_;
          }
          double inner_threshold = closest_radius - threshold_radius_;
          double outer_threshold = closest_radius + threshold_radius_;
          double other_inner_threshold = other_radius - threshold_radius_;
          double other_outer_threshold = other_radius + threshold_radius_;
          int threshold = 0;
          for (int a = 0; a < all_size; a++) {
            if ((inner_threshold < sqrt(pow((yc - all_cones[a].pos.y), 2) +
                                        pow((xc - all_cones[a].pos.x), 2)) &&
                 outer_threshold > sqrt(pow((xc - all_cones[a].pos.x), 2) +
                                        pow((yc - all_cones[a].pos.y), 2))) ||
                (other_inner_threshold <
                     sqrt(pow((yc - all_cones[a].pos.y), 2) +
                          pow((xc - all_cones[a].pos.x), 2)) &&
                 other_outer_threshold >
                     sqrt(pow((xc - all_cones[a].pos.x), 2) +
                          pow((yc - all_cones[a].pos.y), 2)))) {
              threshold += 1;
            }
          }

          if (threshold >= 2 * threshold_cones_) {
            find = true;
          }

          if (threshold > best) {
            best_xc = xc;
            best_yc = yc;
            best = threshold;
          }
        }
        if (find == true) {
          break;
        }
      }
      if (find == true) {
        break;
      }
    }
    if (find == true) {
      break;
    }
  }

  if (!find) {
    best_xc = best_xc_small;
    best_yc = best_yc_small;
  }
  best_xc_small = best_xc;
  best_yc_small = best_yc;
  best_r_small = small_radius_;
  best_xc_large = best_xc;
  best_yc_large = best_yc;
  best_r_large = big_radius_;

  return std::make_tuple(best_xc_small, best_yc_small, best_r_small,
                         best_xc_large, best_yc_large, best_r_large);
}

std::tuple<double, double, double, double, double, double>
CenterPathNode::skidpadRight() {
  std::vector<utfr_msgs::msg::Cone> all_cones;
  all_cones.insert(all_cones.end(), cone_map_local_->left_cones.begin(),
                   cone_map_local_->left_cones.end());
  all_cones.insert(all_cones.end(), cone_map_local_->right_cones.begin(),
                   cone_map_local_->right_cones.end());
  all_cones.insert(all_cones.end(), cone_map_local_->large_orange_cones.begin(),
                   cone_map_local_->large_orange_cones.end());
  std::sort(
      all_cones.begin(), all_cones.end(),
      [this](const utfr_msgs::msg::Cone &a, const utfr_msgs::msg::Cone &b) {
        return this->coneDistComparitor(a, b);
      });

  int all_size = all_cones.size();
  bool find = false;
  double best_xc_small, best_xc_large, best_yc_small, best_yc_large,
      best_r_small, best_r_large, best_xc, best_yc;
  int best = 0;

  for (int i = 0; i < all_size - 2; i++) {
    for (int j = i + 1; j < all_size - 1; j++) {
      for (int k = j + 1; k < all_size; k++) {
        if (i != j && j != k && i != k) {
          utfr_msgs::msg::ConeMap cur_test_right;
          utfr_msgs::msg::Cone cur_test_cone;
          cur_test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;

          cur_test_cone.pos.x = all_cones[i].pos.x;
          cur_test_cone.pos.y = all_cones[i].pos.y;
          (cur_test_right.right_cones).push_back(cur_test_cone);
          cur_test_cone.pos.x = all_cones[j].pos.x;
          cur_test_cone.pos.y = all_cones[j].pos.y;
          (cur_test_right.right_cones).push_back(cur_test_cone);
          cur_test_cone.pos.x = all_cones[k].pos.x;
          cur_test_cone.pos.y = all_cones[k].pos.y;
          (cur_test_right.right_cones).push_back(cur_test_cone);

          std::tuple<double, double, double> circle =
              util::circleLSF(cur_test_right.right_cones);

          double xc = std::get<0>(circle);
          double yc = std::get<1>(circle);
          double r = std::get<2>(circle);
          double radius;

          if (xc >= 5.0 || yc < 0) {
            continue;
          }

          if (abs(r - small_radius_) < 2 * threshold_radius_) {
            radius = small_radius_;
          } else {
            continue;
          }

          double inner_threshold = radius - threshold_radius_;
          double outer_threshold = radius + threshold_radius_;

          int threshold = 0;
          for (int a = 0; a < all_size; a++) {
            if (inner_threshold < sqrt(pow((yc - all_cones[a].pos.y), 2) +
                                       pow((xc - all_cones[a].pos.x), 2)) &&
                outer_threshold > sqrt(pow((xc - all_cones[a].pos.x), 2) +
                                       pow((yc - all_cones[a].pos.y), 2))) {
              threshold += 1;
              if (all_cones[i].type == utfr_msgs::msg::Cone::YELLOW) {
                threshold += 3;
              }
            }
          }

          if (threshold > best) {
            best_xc = xc;
            best_yc = yc;
            best = threshold;
          }
        }
        if (find == true) {
          break;
        }
      }
      if (find == true) {
        break;
      }
    }
    if (find == true) {
      break;
    }
  }

  best_xc_small = best_xc;
  best_yc_small = best_yc;
  best_r_small = small_radius_;
  best_xc_large = best_xc;
  best_yc_large = best_yc;
  best_r_large = big_radius_;

  return std::make_tuple(best_xc_small, best_yc_small, best_r_small,
                         best_xc_large, best_yc_large, best_r_large);
}

std::tuple<double, double, double, double, double, double>
CenterPathNode::skidpadLeft() {
  std::vector<utfr_msgs::msg::Cone> all_cones;
  all_cones.insert(all_cones.end(), cone_map_local_->left_cones.begin(),
                   cone_map_local_->left_cones.end());
  all_cones.insert(all_cones.end(), cone_map_local_->right_cones.begin(),
                   cone_map_local_->right_cones.end());
  all_cones.insert(all_cones.end(), cone_map_local_->large_orange_cones.begin(),
                   cone_map_local_->large_orange_cones.end());
  std::sort(
      all_cones.begin(), all_cones.end(),
      [this](const utfr_msgs::msg::Cone &a, const utfr_msgs::msg::Cone &b) {
        return this->coneDistComparitor(a, b);
      });

  int all_size = all_cones.size();
  bool find = false;
  double best_xc_small, best_xc_large, best_yc_small, best_yc_large,
      best_r_small, best_r_large, best_xc, best_yc;
  int best = 0;

  for (int i = 0; i < all_size - 2; i++) {
    for (int j = i + 1; j < all_size - 1; j++) {
      for (int k = j + 1; k < all_size; k++) {
        if (i != j && j != k && i != k) {
          utfr_msgs::msg::ConeMap cur_test_right;
          utfr_msgs::msg::Cone cur_test_cone;
          cur_test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;

          cur_test_cone.pos.x = all_cones[i].pos.x;
          cur_test_cone.pos.y = all_cones[i].pos.y;
          (cur_test_right.right_cones).push_back(cur_test_cone);
          cur_test_cone.pos.x = all_cones[j].pos.x;
          cur_test_cone.pos.y = all_cones[j].pos.y;
          (cur_test_right.right_cones).push_back(cur_test_cone);
          cur_test_cone.pos.x = all_cones[k].pos.x;
          cur_test_cone.pos.y = all_cones[k].pos.y;
          (cur_test_right.right_cones).push_back(cur_test_cone);

          std::tuple<double, double, double> circle =
              util::circleLSF(cur_test_right.right_cones);

          double xc = std::get<0>(circle);
          double yc = std::get<1>(circle);
          double r = std::get<2>(circle);
          double radius;

          if (xc >= 5.0 || yc > 0) {
            continue;
          }

          if (abs(r - small_radius_) < 2 * threshold_radius_) {
            radius = small_radius_;
          } else {
            continue;
          }

          double inner_threshold = radius - threshold_radius_;
          double outer_threshold = radius + threshold_radius_;

          int threshold = 0;
          for (int a = 0; a < all_size; a++) {
            if (inner_threshold < sqrt(pow((yc - all_cones[a].pos.y), 2) +
                                       pow((xc - all_cones[a].pos.x), 2)) &&
                outer_threshold > sqrt(pow((xc - all_cones[a].pos.x), 2) +
                                       pow((yc - all_cones[a].pos.y), 2))) {
              threshold += 1;
              if (all_cones[i].type == utfr_msgs::msg::Cone::BLUE) {
                threshold += 3;
              }
            }
          }

          if (threshold > best) {
            best_xc = xc;
            best_yc = yc;
            best = threshold;
          }
        }
        if (find == true) {
          break;
        }
      }
      if (find == true) {
        break;
      }
    }
    if (find == true) {
      break;
    }
  }

  best_xc_small = best_xc;
  best_yc_small = best_yc;
  best_r_small = small_radius_;
  best_xc_large = best_xc;
  best_yc_large = best_yc;
  best_r_large = big_radius_;

  return std::make_tuple(best_xc_small, best_yc_small, best_r_small,
                         best_xc_large, best_yc_large, best_r_large);
}

void CenterPathNode::nextWaypoint() {
  unsigned int i = 0;
  unsigned int j = 0;

  double carX = ego_state_->pose.pose.position.x;
  double carY = ego_state_->pose.pose.position.y;
  double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);

  visited.push_back({carX, carY, yaw});
  while (i < waypoints.size() && j < visited.size()) {
    auto [pointX, pointY] = this->transformWaypoint(waypoints[i]);
    auto [carX, carY, yaw] = visited[j];
    double localY = (cos(yaw) * (pointY - carY)) - (sin(yaw) * (pointX - carX));
    double localX = (sin(yaw) * (pointY - carY)) + (cos(yaw) * (pointX - carX));
    double angle = util::wrapDeg(util::radToDeg(atan2(localY, localX)));
    if (angle >= 90 && angle <= 270) {
      i++;
    } else {
      j++;
    }
  }
  visited.pop_back();

  double vx = ego_state_->vel.twist.linear.x,
         vy = ego_state_->vel.twist.linear.y;
  double current_velocity = sqrt(vx * vx + vy * vy);
  double lookahead_distance =
      base_lookahead_distance_ + lookahead_scaling_factor_ * current_velocity;
  while (i < waypoints.size()) {
    auto [x, y] = this->transformWaypoint(waypoints[i]);
    double localX = (sin(yaw) * (y - carY)) + (cos(yaw) * (x - carX));
    double localY = (cos(yaw) * (y - carY)) - (sin(yaw) * (x - carX));
    double dx = localX + 0.79;
    double dy = localY;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist >= lookahead_distance)
      break;
    i++;
  }

  if (i == waypoints.size()) {
    RCLCPP_INFO(this->get_logger(), "NO GLOBAL PATH FOUND, USING LOCAL PATH");
    skidPadFit();
    return;
  }

  geometry_msgs::msg::Pose point;
  auto [x, y] = this->transformWaypoint(waypoints[i]);
  point.position.x = (sin(yaw) * (y - carY)) + (cos(yaw) * (x - carX));
  point.position.y = (cos(yaw) * (y - carY)) - (sin(yaw) * (x - carX));
  center_point_publisher_->publish(point);
  // static std::ofstream out("Waypoints.txt");
  // out << "(" << x << "," << y << ")" << std::endl;
}

void CenterPathNode::GlobalWaypoints() {
  if (skidpadTransform_ == nullptr)
    return;

  using geometry_msgs::msg::PolygonStamped;
  static rclcpp::Publisher<PolygonStamped>::SharedPtr waypoints_pub =
      this->create_publisher<PolygonStamped>("Waypoints", 1);

  PolygonStamped points_stamped;
  points_stamped.header.frame_id = "map";
  points_stamped.header.stamp = this->get_clock()->now();

  for (auto &p : waypoints) {
    auto [x, y] = this->transformWaypoint(p);
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = -y;
    point.z = 0;
    points_stamped.polygon.points.push_back(point);
  }
  waypoints_pub->publish(points_stamped);

  double carX = ego_state_->pose.pose.position.x;
  double carY = ego_state_->pose.pose.position.y;
  double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);
  points_stamped.header.frame_id = "ground";
  points_stamped.header.stamp = this->get_clock()->now();
  points_stamped.polygon.points.clear();

  for (auto &p : waypoints) {
    auto [x, y] = this->transformWaypoint(p);
    geometry_msgs::msg::Point32 point;
    double localX = (sin(yaw) * (y - carY)) + (cos(yaw) * (x - carX));
    double localY = (cos(yaw) * (y - carY)) - (sin(yaw) * (x - carX));
    point.x = localX;
    point.y = -localY;
    point.z = 0;
    points_stamped.polygon.points.push_back(point);
  }
  waypoints_pub->publish(points_stamped);
}

std::vector<std::pair<double, double>>
CenterPathNode::getWaypoints(std::string path) {
  std::ifstream in(path);
  std::vector<std::pair<double, double>> v;
  while (!in.eof()) {
    double x, y;
    in >> x >> y;
    v.push_back({x, y});
  }
  return v;
}

void CenterPathNode::createTransform() {
  double xLeft1, yLeft1;
  std::ifstream("src/planning/global_waypoints/LeftCentre.csv") >> xLeft1 >>
      yLeft1;
  double xRight1, yRight1;
  std::ifstream("src/planning/global_waypoints/RightCentre.csv") >> xRight1 >>
      yRight1;
  double xLeft2, yLeft2, xRight2, yRight2;
  if(colourblind_){
    std::tie(xLeft2, yLeft2, xRight2, yRight2) = this->getCentresColourblind();
    RCLCPP_INFO(this->get_logger(), "COLOURBLIND");
  }
  else{
    std::tie(xLeft2, yLeft2, xRight2, yRight2) = this->getCentres();
  }
  // static std::ofstream out("Centres.txt");
  // out << "(" << xLeft2 << "," << yLeft2 << "), (" << xRight2 << "," <<
  // yRight2 << ")" << std::endl;
  if (isnan(xLeft2) || isnan(xLeft2) || isnan(xRight2) || isnan(yRight2)) {
    skidpadTransform_ = nullptr;
    return;
  }

  auto getIntersect = [&](double x0, double y0, double x1, double y1,
                          int sign) {
    double d = util::euclidianDistance2D(x0, x1, y0, y1);
    double a = d / 2;
    double h = sqrt((15.25 / 2 + 3) * (15.25 / 2 + 3) - a * a);
    double xt = x0 + a * (x1 - x0) / d;
    double yt = y0 + a * (y1 - y0) / d;
    double x = xt + h * (y1 - y0) / d;
    double y = yt - h * (x1 - x0) / d;
    // return the coordinates that give the same signed cross product
    double cross = (x1 - x) * (y0 - y) - (x0 - x) * (y1 - y);
    if (cross * sign < 0) {
      x = xt - h * (y1 - y0) / d;
      y = yt + h * (x1 - x0) / d;
    }
    return std::make_pair(x, y);
  };

  auto [xInter1, yInter1] = getIntersect(xLeft1, yLeft1, xRight1, yRight1, 1);
  auto [xInter2, yInter2] = getIntersect(xLeft2, yLeft2, xRight2, yRight2, -1);
  MatrixXd X(3, 3); // a b tx
  X << xLeft1, yLeft1, 1, xRight1, yRight1, 1, xInter1, yInter1, 1;
  VectorXd Xb(3);
  Xb << xLeft2, xRight2, xInter2;
  VectorXd Xres = X.fullPivLu().solve(Xb);
  MatrixXd Y(3, 3); // c d ty
  Y << xLeft1, yLeft1, 1, xRight1, yRight1, 1, xInter1, yInter1, 1;
  VectorXd Yb(3);
  Yb << yLeft2, yRight2, yInter2;
  VectorXd Yres = Y.fullPivLu().solve(Yb);
  MatrixXd T(3, 3);
  T << Xres(0), Xres(1), Xres(2), Yres(0), Yres(1), Yres(2), 0, 0, 1;

  skidpadTransform_ = std::make_unique<MatrixXd>(T);
}

std::pair<double, double>
CenterPathNode::transformWaypoint(const std::pair<double, double> &point) {
  auto [x, y] = point;
  VectorXd v(3);
  v << x, y, 1;

  VectorXd out = *skidpadTransform_ * v;
  return {out(0), out(1)};
}

std::tuple<double, double, double, double> CenterPathNode::getCentres() {
  std::vector<utfr_msgs::msg::Cone> &orange = cone_map_global_->large_orange_cones;
  if (orange.size() < 4) {
    return {NAN, NAN, NAN, NAN};
  }
  auto [xLeft, yLeft, xRight, yRight] = this->skidpadCircleCentres();
  if (isnan(xLeft) || isnan(xRight) || isnan(yLeft) || isnan(yRight)) {
    return {NAN, NAN, NAN, NAN};
  }
  double xMid =
      (orange[0].pos.x + orange[1].pos.x + orange[2].pos.x + orange[3].pos.x) /
      4;
  double yMid =
      (orange[0].pos.y + orange[1].pos.y + orange[2].pos.y + orange[3].pos.y) /
      4;
  double rightDist = util::euclidianDistance2D(xRight, xMid, yRight, yMid);
  double leftDist = util::euclidianDistance2D(xMid, xLeft, yMid, yLeft);
  double totalDist = util::euclidianDistance2D(xRight, xLeft, yRight, yLeft);
  if (abs(rightDist - centre_distance_) > 0.25) { // improper right centre
    return {NAN, NAN, NAN, NAN};
  }
  // improper left centre, extrapolate it
  if (abs(totalDist - 2 * centre_distance_) > 0.5 ||
      abs(leftDist - centre_distance_) > 0.25) {
    xLeft = xMid * 2 - xRight;
    yLeft = yMid * 2 - yRight;
  }
  return {xLeft, yLeft, xRight, yRight};
}

std::tuple<double, double, double, double>
CenterPathNode::skidpadCircleCentres() {
  using geometry_msgs::msg::PolygonStamped;
  static rclcpp::Publisher<PolygonStamped>::SharedPtr small_blue_pub =
      this->create_publisher<PolygonStamped>("SmallBlue", 1);
  static rclcpp::Publisher<PolygonStamped>::SharedPtr small_yellow_pub =
      this->create_publisher<PolygonStamped>("SmallYellow", 1);
  static rclcpp::Publisher<PolygonStamped>::SharedPtr large_blue_pub =
      this->create_publisher<PolygonStamped>("LargeBlue", 1);
  static rclcpp::Publisher<PolygonStamped>::SharedPtr large_yellow_pub =
      this->create_publisher<PolygonStamped>("LargeYellow", 1);

  std::vector<utfr_msgs::msg::Cone> &blue = cone_map_global_->left_cones;
  std::vector<utfr_msgs::msg::Cone> &yellow = cone_map_global_->right_cones;

  // auto print = [&](auto &out, auto &c){
  //   auto [x,y,r,t] = c;
  //   out << "(" << x << "," << y << "," << r << "," << t << ")" << std::endl;
  // };

  auto smallBlue =
      this->circleCentre(blue, small_radius_, small_circle_cones_ / 2);
  // static std::ofstream a("SmallBlue.txt");
  // print(a, smallBlue);
  auto smallYellow =
      this->circleCentre(yellow, small_radius_, small_circle_cones_ / 2);
  // static std::ofstream b("SmallYellow.txt");
  // print(b, smallYellow);
  auto largeBlue = this->circleCentre(blue, big_radius_, big_circle_cones_ / 2);
  // static std::ofstream c("LargeBlue.txt");
  // print(c, largeBlue);
  auto largeYellow =
      this->circleCentre(yellow, big_radius_, big_circle_cones_ / 2);
  // static std::ofstream d("LargeYellow.txt");
  // print(d, largeYellow);

  auto drawCircle = [this](auto publisher, auto &cord, double radius) {
    auto [xc, yc, r, t] = cord;
    PolygonStamped circle_stamped_global;
    circle_stamped_global.header.frame_id = "map";
    circle_stamped_global.header.stamp = this->get_clock()->now();
    PolygonStamped circle_stamped_local;
    circle_stamped_local.header.frame_id = "ground";
    circle_stamped_local.header.stamp = this->get_clock()->now();

    double carX = ego_state_->pose.pose.position.x;
    double carY = ego_state_->pose.pose.position.y;
    double yaw = -util::quaternionToYaw(ego_state_->pose.pose.orientation);

    for (int i = 0; i < 360; i++) {
      geometry_msgs::msg::Point32 point;
      double angle = 2.0 * M_PI * i / 360;
      point.x = xc + radius * cos(angle);
      point.y = -yc + radius * sin(angle);
      point.z = 0;
      circle_stamped_global.polygon.points.push_back(point);

      double localX =
          (sin(yaw) * (-point.y - carY)) + (cos(yaw) * (point.x - carX));
      double localY =
          (cos(yaw) * (-point.y - carY)) - (sin(yaw) * (point.x - carX));
      point.x = localX;
      point.y = localY;
      circle_stamped_local.polygon.points.push_back(point);
    }
    publisher->publish(circle_stamped_global);
    publisher->publish(circle_stamped_local);
  };

  drawCircle(small_blue_pub, smallBlue, 0.1);
  drawCircle(small_yellow_pub, smallYellow, 0.1);
  drawCircle(large_blue_pub, largeBlue, 0.1);
  drawCircle(large_yellow_pub, largeYellow, 0.1);

  auto getBetterCentre = [](auto &first, auto &second) {
    auto [x1, y1, r1, t1] = first;
    auto [x2, y2, r2, t2] = second;
    double x, y;
    if (r1 < r2)
      x = x1, y = y1;
    else if (r2 < r1)
      x = x2, y = y2;
    else {
      if (t1 < t2)
        x = x1, y = y1;
      else
        x = x2, y = y2;
    }
    return std::make_pair(x, y);
  };
  auto [xLeft, yLeft] = getBetterCentre(smallBlue, largeYellow);
  auto [xRight, yRight] = getBetterCentre(smallYellow, largeBlue);
  return {xLeft, yLeft, xRight, yRight};
}

std::tuple<double, double, double, double>
CenterPathNode::circleCentre(std::vector<utfr_msgs::msg::Cone> &cones,
                             double target_radius, int inlier_count) {
  int n = cones.size();
  if (n < inlier_count)
    return {NAN, NAN, DBL_MAX, DBL_MAX};
  auto get_threshold = [&](double xc, double yc, double radius) {
    // get the distances from the circle's centre
    std::vector<double> distances(n);
    for (int i = 0; i < n; i++) {
      auto &pos = cones[i].pos;
      distances[i] = util::euclidianDistance2D(pos.x, xc, pos.y, yc);
    }
    // sort the distances by closeness to the radius
    auto cmp = [&](double a, double b) {
      return abs(a - radius) < abs(b - radius);
    };
    sort(distances.begin(), distances.end(), cmp);
    // find the threshold that contains inlier_count points
    double threshold;
    if (n < inlier_count)
      threshold = abs(radius - distances[n - 1]);
    else
      threshold = abs(radius - distances[inlier_count - 1]);
    return threshold;
  };

  auto circle = [](std::vector<utfr_msgs::msg::Cone> &cones) {
    MatrixXd A(2, 2); // A -> [X Y]
    VectorXd B(2);
    auto x = [&](int i) { return cones[i].pos.x; };
    auto y = [&](int i) { return cones[i].pos.y; };
    // Solve 3 circle equations (x-a)^2 + (y-b)^2 = r^2
    // equation 1 - equation 2
    A(0, 0) = 2 * (x(0) - x(1));
    A(0, 1) = 2 * (y(0) - y(1));
    B(0) = (x(0) * x(0) + y(0) * y(0)) - (x(1) * x(1) + y(1) * y(1));
    // equation 1 - equation 3
    A(1, 0) = 2 * (x(0) - x(2));
    A(1, 1) = 2 * (y(0) - y(2));
    B(1) = (x(0) * x(0) + y(0) * y(0)) - (x(2) * x(2) + y(2) * y(2));
    // Solve for X & Y
    VectorXd ans = A.fullPivLu().solve(B);
    double xc = ans(0), yc = ans(1);
    double r = sqrt(pow(x(0) - xc, 2) + pow(y(0) - yc, 2));
    return std::make_tuple(xc, yc, r);
  };

  // find the circle with the lowest threshold
  double best_x = NAN, best_y = NAN, best_radius = DBL_MAX,
         best_threshold = DBL_MAX;
  std::vector<utfr_msgs::msg::Cone> ransacCones(3);
  for (int i = 0; i < n; i++) {
    ransacCones[0] = cones[i];
    for (int j = i + 1; j < n; j++) {
      ransacCones[1] = cones[j];
      for (int k = j + 1; k < n; k++) {
        ransacCones[2] = cones[k];
        auto [xc, yc, radius] = circle(ransacCones);
        double threshold = get_threshold(xc, yc, target_radius);
        if (threshold < best_threshold) {
          best_threshold = threshold;
          best_x = xc;
          best_y = yc;
          best_radius = abs(radius - target_radius);
        } else if (threshold == best_threshold && radius < best_radius) {
          best_x = xc;
          best_y = yc;
          best_radius = abs(radius - target_radius);
        }
      }
    }
  }
  return {best_x, best_y, best_radius, best_threshold};
}

std::tuple<double, double, double, double> CenterPathNode::getCentresColourblind() {
  auto [xLeft, yLeft, xRight, yRight] = this->getSkidpadCircleCentresColourblind();
  if (isnan(xLeft) || isnan(xRight) || isnan(yLeft) || isnan(yRight)) {
    return {NAN, NAN, NAN, NAN};
  }
  if(abs(util::euclidianDistance2D(xLeft, xRight, yLeft, yRight)-18.25) > 0.5){
    return {NAN, NAN, NAN, NAN};
  }
  using geometry_msgs::msg::PolygonStamped;
  static rclcpp::Publisher<PolygonStamped>::SharedPtr left_circle_pub =
      this->create_publisher<PolygonStamped>("LeftCircle", 1);
  static rclcpp::Publisher<PolygonStamped>::SharedPtr right_circle_pub =
      this->create_publisher<PolygonStamped>("RightCircle", 1);
  auto drawCircle = [this](auto publisher, double xc, double yc) {
    PolygonStamped circle_stamped_global;
    circle_stamped_global.header.frame_id = "map";
    circle_stamped_global.header.stamp = this->get_clock()->now();
    PolygonStamped circle_stamped_local;
    circle_stamped_local.header.frame_id = "ground";
    circle_stamped_local.header.stamp = this->get_clock()->now();

    double carX = ego_state_->pose.pose.position.x;
    double carY = ego_state_->pose.pose.position.y;
    double yaw = -util::quaternionToYaw(ego_state_->pose.pose.orientation);

    for (int i = 0; i < 360; i++) {
      geometry_msgs::msg::Point32 point;
      double angle = 2.0 * M_PI * i / 360;
      point.x = xc + 0.1 * cos(angle);
      point.y = -yc + 0.1 * sin(angle);
      point.z = 0;
      circle_stamped_global.polygon.points.push_back(point);

      double localX =
          (sin(yaw) * (-point.y - carY)) + (cos(yaw) * (point.x - carX));
      double localY =
          (cos(yaw) * (-point.y - carY)) - (sin(yaw) * (point.x - carX));
      point.x = localX;
      point.y = localY;
      circle_stamped_local.polygon.points.push_back(point);
    }
    publisher->publish(circle_stamped_global);
    publisher->publish(circle_stamped_local);
  };
  drawCircle(left_circle_pub, xLeft, yLeft);
  drawCircle(right_circle_pub, xRight, yRight);
  return {xLeft, yLeft, xRight, yRight};
}

std::tuple<double,double,double> circle(std::vector<utfr_msgs::msg::Cone> &cones) {
    MatrixXd A(2, 2); // A -> [X Y]
    VectorXd B(2);
    auto x = [&](int i) { return cones[i].pos.x; };
    auto y = [&](int i) { return cones[i].pos.y; };
    // Solve 3 circle equations (x-a)^2 + (y-b)^2 = r^2
    // equation 1 - equation 2
    A(0, 0) = 2 * (x(0) - x(1));
    A(0, 1) = 2 * (y(0) - y(1));
    B(0) = (x(0) * x(0) + y(0) * y(0)) - (x(1) * x(1) + y(1) * y(1));
    // equation 1 - equation 3
    A(1, 0) = 2 * (x(0) - x(2));
    A(1, 1) = 2 * (y(0) - y(2));
    B(1) = (x(0) * x(0) + y(0) * y(0)) - (x(2) * x(2) + y(2) * y(2));
    // Solve for X & Y
    VectorXd ans = A.fullPivLu().solve(B);
    double xc = ans(0), yc = ans(1);
    double r = sqrt(pow(x(0) - xc, 2) + pow(y(0) - yc, 2));
    return std::make_tuple(xc, yc, r);
}

std::tuple<double,double,double,double> CenterPathNode::getSkidpadCircleCentresColourblind(){
    std::vector<utfr_msgs::msg::Cone> cones;
    cones.insert(cones.end(), cone_map_global_->left_cones.begin(),
                    cone_map_global_->left_cones.end());
    cones.insert(cones.end(), cone_map_global_->right_cones.begin(),
                    cone_map_global_->right_cones.end());
    cones.insert(cones.end(), cone_map_global_->small_orange_cones.begin(),
                    cone_map_global_->small_orange_cones.end());
    cones.insert(cones.end(), cone_map_global_->large_orange_cones.begin(),
                    cone_map_global_->large_orange_cones.end());
    int n = cones.size();
    std::vector<utfr_msgs::msg::Cone> cur(3);
    int best_threshold = 0;
    double best_radius = NAN, xc = NAN, yc = NAN;
    std::vector<std::tuple<int,double,double,double>> vals;
    for(int i = 0; i < n; i++){
        cur[0] = cones[i];
        for(int j = i+1; j < n; j++){
            cur[1] = cones[j];
            for(int k = j+1; k < n; k++){
                cur[2] = cones[k];
                auto [x,y,r] = circle(cur);
                int cur_threshold = 0;
                for(const auto &cone : cones){
                    double distance = std::sqrt((cone.pos.x-x) * (cone.pos.x-x) + (cone.pos.y-y) * (cone.pos.y-y));
                    if(abs(distance-r) < 0.1) cur_threshold++;
                }
                if(cur_threshold > best_threshold){
                    xc = x;
                    yc = y;
                    best_radius = r;
                    best_threshold = cur_threshold;
                }
                vals.emplace_back(cur_threshold, x, y, r);
            }
        }
    }
    int best_threshold2 = 0;
    double best_radius2 = NAN, xc2 = NAN, yc2 = NAN;
    for(auto [t,x,y,r] : vals){
        if(t > best_threshold2 && std::abs(std::sqrt((x-xc) * (x-xc) + (y-yc) * (y-yc))-18.25) < 0.5){
            xc2 = x;
            yc2 = y;
            best_radius2 = r;
            best_threshold2 = t;
        }
    }
    if(best_threshold < small_circle_cones_ /2 || best_threshold2 < small_circle_cones_ /2){
      return {NAN,NAN,NAN,NAN};
    }
    if(xc*yc2-xc2*yc < 0){
        std::swap(xc,xc2);
        std::swap(yc,yc2);
    }
    return {xc,yc,xc2,yc2};
}

} // namespace center_path
} // namespace utfr_dv