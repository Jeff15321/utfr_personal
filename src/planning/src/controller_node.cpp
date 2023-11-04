/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: controller_node.cpp
* auth: Justin Lim
* desc: controller node class
*/

#include <controller_node.hpp>

namespace utfr_dv {
namespace controller {

ControllerNode::ControllerNode() : Node("controller_node") {
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

void ControllerNode::initParams() {
  this->declare_parameter("update_rate", 33.33);
  this->declare_parameter("event", "accel");
  this->declare_parameter("controller", "stanley");
  this->declare_parameter("stanley_gain", 1.0);
  this->declare_parameter("softening_constant", 5.0);
  this->declare_parameter("k_yaw_rate", 0.0);
  this->declare_parameter("k_damp_steer", 0.0);
  this->declare_parameter("discretized_points", 100);
  this->declare_parameter("cte_error", 0.01);
  this->declare_parameter("cte_angle_error", 1.0);
  this->declare_parameter("ds", 0.01);
  this->declare_parameter("max_velocity", 3.0);
  this->declare_parameter("max_steering_angle", 0.34);
  this->declare_parameter("max_steering_rate", 0.2);
  this->declare_parameter("max_tire", 1.5);
  this->declare_parameter("baselink_location", 0.79);
  this->declare_parameter("wheel_base", 1.58);
  this->declare_parameter("num_points", 1000);
  this->declare_parameter("lookahead_distance_pp", 1.0);

  update_rate_ = this->get_parameter("update_rate").as_double();
  event_ = this->get_parameter("event").as_string();
  controller_ = this->get_parameter("controller").as_string();
  stanley_gain_ = this->get_parameter("stanley_gain").as_double();
  softening_constant_ = this->get_parameter("softening_constant").as_double();
  k_yaw_rate_ = this->get_parameter("k_yaw_rate").as_double();
  k_damp_steer_ = this->get_parameter("k_damp_steer").as_double();
  discretized_points_ = this->get_parameter("discretized_points").as_int();
  cte_error_ = this->get_parameter("cte_error").as_double();
  cte_angle_error_ = this->get_parameter("cte_angle_error").as_double();
  ds_ = this->get_parameter("ds").as_double();
  max_velocity_ = this->get_parameter("max_velocity").as_double();
  max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
  max_steering_rate_ = this->get_parameter("max_steering_rate").as_double();
  max_tire_ = this->get_parameter("max_tire").as_double();
  baselink_location_ = this->get_parameter("baselink_location").as_double();
  wheel_base_ = this->get_parameter("wheel_base").as_double();
  num_points_ = this->get_parameter("num_points").as_int();
  lookahead_distance_ =
      this->get_parameter("lookahead_distance_pp").as_double();
}

void ControllerNode::initSubscribers() {
  ego_state_subscriber_ = this->create_subscription<utfr_msgs::msg::EgoState>(
      topics::kEgoState, 10, std::bind(&ControllerNode::egoStateCB, this, _1));

  cone_map_subscriber_ = this->create_subscription<utfr_msgs::msg::ConeMap>(
      topics::kConeMap, 10, std::bind(&ControllerNode::coneMapCB, this, _1));

  path_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ParametricSpline>(
          topics::kOptimizedCenterPath, 10,
          std::bind(&ControllerNode::pathCB, this, _1));

  velocity_profile_subscriber_ =
      this->create_subscription<utfr_msgs::msg::VelocityProfile>(
          topics::kVelocityProfile, 10,
          std::bind(&ControllerNode::velocityProfileCB, this, _1));
}

void ControllerNode::initPublishers() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kControllerHeartbeat, 10);

  target_state_publisher_ = this->create_publisher<utfr_msgs::msg::TargetState>(
      topics::kTargetState, 10);
}

void ControllerNode::initTimers() {
  if (event_ == "accel") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBAccel, this));
  } else if (event_ == "skidpad") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBSkidpad, this));
  } else if (event_ == "autocross") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBAutocross, this));
  } else if (event_ == "trackdrive") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBTrackdrive, this));
  }
}

void ControllerNode::initHeartbeat() {
  heartbeat_.module.data = "controller_node";
  heartbeat_.update_rate = update_rate_;
}

void ControllerNode::publishHeartbeat(const int status) {
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->now();
  heartbeat_publisher_->publish(heartbeat_);
}

void ControllerNode::egoStateCB(const utfr_msgs::msg::EgoState &msg) {
  if (ego_state_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::EgoState template_ego;
    ego_state_ = std::make_shared<utfr_msgs::msg::EgoState>(template_ego);
  }

  ego_state_->header = msg.header;
  ego_state_->pose = msg.pose;
  ego_state_->vel = msg.vel;
  ego_state_->accel = msg.accel;
  ego_state_->steering_angle = msg.steering_angle;
  ego_state_->lap_count = msg.lap_count;
  ego_state_->finished = msg.finished;
}

void ControllerNode::coneMapCB(const utfr_msgs::msg::ConeMap &msg) {
  if (cone_map_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::ConeMap template_cone_map;
    cone_map_ = std::make_shared<utfr_msgs::msg::ConeMap>(template_cone_map);
  }

  cone_map_->header = msg.header;
  cone_map_->left_cones = msg.left_cones;
  cone_map_->right_cones = msg.right_cones;
  cone_map_->large_orange_cones = msg.large_orange_cones;
  cone_map_->small_orange_cones = msg.small_orange_cones;
}

void ControllerNode::pathCB(const utfr_msgs::msg::ParametricSpline &msg) {
  if (path_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::ParametricSpline template_path;
    path_ = std::make_shared<utfr_msgs::msg::ParametricSpline>(template_path);
  }

  std::vector<double> template_vector = {0.0, 0.0, 0.0};
  path_->header = msg.header;
  path_->x_params = msg.x_params;
  path_->y_params = msg.y_params;
}

void ControllerNode::velocityProfileCB(
    const utfr_msgs::msg::VelocityProfile &msg) {
  if (velocity_profile_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::VelocityProfile template_velocity_profile;
    velocity_profile_ = std::make_shared<utfr_msgs::msg::VelocityProfile>(
        template_velocity_profile);
  }

  velocity_profile_->header = msg.header;
  velocity_profile_->velocities = msg.velocities;
}

void ControllerNode::timerCBAccel() {
  const std::string function_name{"controller_timerCB:"};

  if (!path_ || !velocity_profile_) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                "Data not published or initialized yet. Using defaults.");
    return;
  }

  // CODE GOES HERE
  double cur_s_ = 0;

  if (controller_ == "stanley") {
    utfr_msgs::msg::TargetState target =
        stanleyController(stanley_gain_, max_velocity_, max_steering_angle_,
                          max_steering_rate_, *path_, cur_s_, ds_,
                          *velocity_profile_, baselink_location_, *ego_state_);
    target_ = target;
  } else if (controller_ == "pure_pursuit") {
    utfr_msgs::msg::TargetState target = purePursuitController(
        max_velocity_, max_steering_angle_, *path_, cur_s_, ds_,
        *velocity_profile_, baselink_location_, *ego_state_,
        lookahead_distance_);
    target_ = target;
  }

  // publish target state
  target_state_publisher_->publish(target_);
}

void ControllerNode::timerCBSkidpad() {
  const std::string function_name{"controller_timerCB:"};

  if (!path_ || !velocity_profile_) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                "Data not published or initialized yet. Using defaults.");
    return;
  }

  // CODE GOES HERE

  // publish target state
  target_state_publisher_->publish(target_);
}

void ControllerNode::timerCBAutocross() {
  const std::string function_name{"controller_timerCB:"};

  if (!path_ || !velocity_profile_) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                "Data not published or initialized yet. Using defaults.");
    return;
  }

  // CODE GOES HERE

  // publish target state
  target_state_publisher_->publish(target_);
}

void ControllerNode::timerCBTrackdrive() {
  const std::string function_name{"controller_timerCB:"};

  if (!path_ || !velocity_profile_) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                "Data not published or initialized yet. Using defaults.");
    return;
  }

  // CODE GOES HERE

  // publish target state
  target_state_publisher_->publish(target_);
}

geometry_msgs::msg::Pose
ControllerNode::discretizePoint(const utfr_msgs::msg::ParametricSpline &spline,
                                const double s, const double delta) {
  std::vector<double> x_params = spline.x_params;
  std::vector<double> y_params = spline.y_params;

  double x = x_params[0] * pow(s, 5) + x_params[1] * pow(s, 4) +
             x_params[2] * pow(s, 3) + x_params[3] * pow(s, 2) +
             x_params[4] * s + x_params[5];

  double y = y_params[0] * pow(s, 5) + y_params[1] * pow(s, 4) +
             y_params[2] * pow(s, 3) + y_params[3] * pow(s, 2) +
             y_params[4] * s + y_params[5];

  double x_before =
      x_params[0] * pow(s - delta, 5) + x_params[1] * pow(s - delta, 4) +
      x_params[2] * pow(s - delta, 3) + x_params[3] * pow(s - delta, 2) +
      x_params[4] * (s - delta) + x_params[5];

  double y_before =
      y_params[0] * pow(s - delta, 5) + y_params[1] * pow(s - delta, 4) +
      y_params[2] * pow(s - delta, 3) + y_params[3] * pow(s - delta, 2) +
      y_params[4] * (s - delta) + y_params[5];

  double x_after =
      x_params[0] * pow(s + delta, 5) + x_params[1] * pow(s + delta, 4) +
      x_params[2] * pow(s + delta, 3) + x_params[3] * pow(s + delta, 2) +
      x_params[4] * (s + delta) + x_params[5];

  double y_after =
      y_params[0] * pow(s + delta, 5) + y_params[1] * pow(s + delta, 4) +
      y_params[2] * pow(s + delta, 3) + y_params[3] * pow(s + delta, 2) +
      y_params[4] * (s + delta) + y_params[5];

  double heading = atan2(y_after - y_before, x_after - x_before);

  geometry_msgs::msg::Pose res;
  res.position.x = x;
  res.position.y = y;
  res.orientation = util::yawToQuaternion(heading);

  return res;
}

std::vector<geometry_msgs::msg::Pose> ControllerNode::discretizeParametric(
    const utfr_msgs::msg::ParametricSpline &spline_params, double cur_s,
    double ds, int num_points) {
  std::vector<geometry_msgs::msg::Pose> discretized_points;

  double s = cur_s;
  int count = 0;
  double delta = 0.001;
  while (count++ < num_points) {
    geometry_msgs::msg::Pose point = discretizePoint(spline_params, s, delta);
    discretized_points.push_back(point);
    s += ds;
  }

  return discretized_points;
}

geometry_msgs::msg::Pose
ControllerNode::closestPoint(utfr_msgs::msg::EgoState ego_state,
                             std::vector<geometry_msgs::msg::Pose> waypoints) {
  double closest_distance = 100000.0;
  geometry_msgs::msg::Pose closest_wp;

  for (int i = 0; i < waypoints.size(); i++) {
    geometry_msgs::msg::Pose wp = waypoints[i];
    double dx = wp.position.x - ego_state.pose.pose.position.x;
    double dy = wp.position.y - ego_state.pose.pose.position.y;
    double distance = sqrt(dx * dx + dy * dy);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_wp = wp;
    }
  }

  return closest_wp;
}

double ControllerNode::sign(double x) { return x > 0 ? 1 : x < 0 ? -1 : 0; }

utfr_msgs::msg::TargetState ControllerNode::stanleyController(
    double k, double max_speed, double max_steering_angle,
    double max_steering_rate, utfr_msgs::msg::ParametricSpline spline_params,
    double cur_s, double ds, utfr_msgs::msg::VelocityProfile velocity_profile,
    double baselink_location, utfr_msgs::msg::EgoState ego_state) {
  const std::string function_name{"stanleyController:"};

  double vehicle_theta = ego_state.pose.pose.position.z;

  double prev_steering = ego_state.steering_angle;

  double vehicle_x =
      ego_state.pose.pose.position.x + baselink_location * cos(vehicle_theta);
  double vehicle_y = ego_state.pose.pose.position.y +
                     baselink_location * sin(vehicle_theta); // FIX LATER

  std::vector<geometry_msgs::msg::Pose> discretized_points =
      discretizeParametric(spline_params, cur_s, ds, num_points_);

  geometry_msgs::msg::Pose closest_wp =
      closestPoint(ego_state, discretized_points);

  double vehicle_velocity = ego_state.vel.twist.linear.x;

  // Compute the errors
  double dx = closest_wp.position.x - vehicle_x;
  double dy = closest_wp.position.y - vehicle_y;
  // Crosstrack error i.e. the distance from the vehicle to the racing line
  double cte = sqrt(dx * dx + dy * dy);
  // Heading error i.e. the difference between the vehicle's heading and the
  // racing line's heading
  if (vehicle_theta > M_PI) {
    vehicle_theta = vehicle_theta - 2 * M_PI;
  } else if (vehicle_theta < -M_PI) {
    vehicle_theta = vehicle_theta + 2 * M_PI;
  }
  double psi = util::quaternionToYaw(closest_wp.orientation) - vehicle_theta;

  if (psi > M_PI) {
    psi = psi - 2 * M_PI;
  } else if (psi < -M_PI) {
    psi = psi + 2 * M_PI;
  }

  // Compute vector from vehicle to target point
  double vecToTarget[2] = {dx, dy};

  // Compute path direction vector at target point
  double theta = util::quaternionToYaw(closest_wp.orientation);
  double pathDir[2] = {cos(theta), sin(theta)};

  // Compute the magnitudes of the vectors
  double magnitudeVecToTarget =
      sqrt(vecToTarget[0] * vecToTarget[0] + vecToTarget[1] * vecToTarget[1]);
  double magnitudePathDir =
      sqrt(pathDir[0] * pathDir[0] + pathDir[1] * pathDir[1]);

  // Compute the dot product
  double dotProduct = vecToTarget[0] * pathDir[0] + vecToTarget[1] * pathDir[1];

  // Compute the angle in radians
  double angleRad =
      acos(dotProduct / (magnitudeVecToTarget * magnitudePathDir));

  // Convert the angle to degrees
  double angleDeg = angleRad * 180.0 / M_PI;
  if (cte < cte_error_ || abs(angleDeg) < cte_angle_error_)
    cte = 0.0;

  // Compute cross product
  double crossProduct =
      vecToTarget[0] * pathDir[1] - vecToTarget[1] * pathDir[0];

  // Check the side
  if (crossProduct > 0) {
    cte = -cte;
  }

  if (vehicle_velocity < 1.0) {
    vehicle_velocity = 1.0; // to prevent division by zero
  }

  // Control law for the Stanley steering controller
  double stanley_term = atan2((k * cte), vehicle_velocity);

  double desired_velocity;

  double yaw_rate_damping =
      k_yaw_rate_ * (-desired_velocity * sin(prev_steering)) / wheel_base_;

  double desired_steering_angle = psi + yaw_rate_damping + stanley_term;

  double steering_delay =
      k_damp_steer_ * (desired_steering_angle - prev_steering);

  double unclipped_delta = desired_steering_angle + steering_delay;

  // Clip the steering angle to the maximum allowed
  double delta =
      std::clamp(unclipped_delta, -max_steering_angle, max_steering_angle);

  // If we're at the maximum steering angle, slow down
  if (abs(delta) == max_steering_angle ||
      (cte > cte_error_ && abs(angleDeg) > cte_angle_error_ &&
       abs(psi) > 0.2 * max_steering_angle)) {
    desired_velocity = 1.0; // or some other speed lower than max_speed
  } else {
    // Use the velocity profile to determine the desired velocity
    desired_velocity = velocity_profile.velocities[0];
  }

  RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
              "Target steering: %f \n Target velocity: %f", delta,
              desired_velocity);

  utfr_msgs::msg::TargetState target;
  target.speed = desired_velocity;
  target.steering_angle = delta;

  return target;
}

utfr_msgs::msg::TargetState ControllerNode::purePursuitController(
    double max_speed, double max_steering_angle,
    utfr_msgs::msg::ParametricSpline spline_params, double cur_s, double ds,
    utfr_msgs::msg::VelocityProfile velocity_profile, double baselink_location,
    utfr_msgs::msg::EgoState ego_state, double lookahead_distance) {
  double vehicle_theta = ego_state.pose.pose.position.z;
  double vehicle_x =
      ego_state.pose.pose.position.x + baselink_location * cos(vehicle_theta);
  double vehicle_y =
      ego_state.pose.pose.position.y + baselink_location * sin(vehicle_theta);

  std::vector<geometry_msgs::msg::Pose> discretized_points =
      discretizeParametric(spline_params, cur_s, ds, num_points_);

  geometry_msgs::msg::Pose lookahead_point;
  bool found_lookahead = false;
  for (const auto &wp : discretized_points) {
    double dx = wp.position.x - vehicle_x;
    double dy = wp.position.y - vehicle_y;
    double distance = sqrt(dx * dx + dy * dy);
    if (distance > lookahead_distance) {
      lookahead_point = wp;
      found_lookahead = true;
      break;
    }
  }

  if (!found_lookahead) {
    // Handle scenario where lookahead point isn't found
    // Return some default state or do some error handling
    // Figure out later when testing / if neccesary
    // Can do things such as return the last point in the discretized path
  }

  // Compute the angle between the current vehicle orientation and the direction
  // towards the lookahead point
  double alpha = atan2(lookahead_point.position.y - vehicle_y,
                       lookahead_point.position.x - vehicle_x) -
                 vehicle_theta;

  // Steering law for Pure Pursuit controller
  double delta = atan2(2.0 * wheel_base_ * sin(alpha), lookahead_distance);

  // Clamp steering angle to its limits
  delta = std::clamp(delta, -max_steering_angle, max_steering_angle);

  // Get desired velocity from the velocity profile
  double desired_velocity = velocity_profile.velocities[0];

  utfr_msgs::msg::TargetState target;
  target.speed = desired_velocity;
  target.steering_angle = delta;

  return target;
}

} // namespace controller
} // namespace utfr_dv
