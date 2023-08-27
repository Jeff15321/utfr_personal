/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: pure_pursuit_controller.cpp
* auth: Youssef Elhadad
* desc: pure pursuit lateral controller
*
*/

#include "controller/pure_pursuit_controller.hpp"

namespace utfr_dv {
namespace controls {

void PurePursuitController::initController(double wheelbase, double ld_sf) {
  wheelbase_ = wheelbase;
  lookahead_distance_scaling_factor_ = ld_sf;
}

double PurePursuitController::getSteeringAngle(
    utfr_msgs::msg::TargetState::SharedPtr target,
    utfr_msgs::msg::EgoState::SharedPtr ego) {

  geometry_msgs::msg::Vector3 ego_pos = ego->pos;
  geometry_msgs::msg::Vector3 target_pos = target->trajectory.pos;

  double ego_velocity = utfr_dv::util::vectorMagnitude(ego->vel);

  // Check if velocity is zero or null
  if (isnan(ego_velocity) || ego_velocity == 0 || target == nullptr) {
    return 0;
  }

  auto node = std::make_shared<rclcpp::Node>("a_test_node");

  // PurePursuit Algorithim

  // Step 1: Calculate the pos at the rear of the car
  geometry_msgs::msg::Vector3 rear_pos;

  // using rot from North
  rear_pos.x = ego_pos.x - cos((ego->yaw)) * wheelbase_ / 2;
  rear_pos.y = ego_pos.y - sin((ego->yaw)) * wheelbase_ / 2;

  // Step 2: find vector from rear to target
  geometry_msgs::msg::Vector3 rear_to_target;
  geometry_msgs::msg::Vector3 rear_to_center;
  rear_to_target.x = target_pos.x - rear_pos.x;
  rear_to_target.y = target_pos.y - rear_pos.y;
  rear_to_center.x = ego_pos.x - rear_pos.x;
  rear_to_center.y = ego_pos.y - rear_pos.y;

  // Step 3: Find dot and cross product
  double dot_product;
  double cross_product;

  dot_product = (rear_to_target.x * rear_to_center.x +
                 rear_to_target.y * rear_to_center.y);

  cross_product = (rear_to_center.x * rear_to_target.y -
                   rear_to_target.x * rear_to_center.y);

  double lookahead_distance_from_center =
      utfr_dv::util::euclidianDistance2D(target_pos, ego_pos);

  double lookahead_distance_from_rear =
      utfr_dv::util::vectorMagnitude(rear_to_target);

  // Step 4: Calculate alpha
  double alpha;
  if (isnan(dot_product) &&
      (lookahead_distance_from_rear < lookahead_distance_from_center))
    return PI / 2;
  else if (isnan(dot_product) &&
           (lookahead_distance_from_rear > lookahead_distance_from_center))
    return 0;

  alpha = acos(dot_product / (lookahead_distance_from_rear * wheelbase_ / 2));

  double steering_angle;

  steering_angle =
      abs(atan((2 * wheelbase_ * sin(alpha)) /
               (lookahead_distance_scaling_factor_ * ego_velocity)));

  steering_angle = cross_product < 0 ? steering_angle : -steering_angle;

  if (isnan(steering_angle)) {
    return 0;
  }

  return steering_angle;
}
} // namespace controls
} // namespace utfr_dv