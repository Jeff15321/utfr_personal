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
  this->declare_parameter("controller", "pure_pursuit");
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
  this->declare_parameter("base_lookahead_distance", 1.0);
  this->declare_parameter("lookahead_scaling_factor", 0.15);

  update_rate_ = this->get_parameter("update_rate").as_double();
  event_ = this->get_parameter("event").as_string();
  controller_ = this->get_parameter("controller").as_string();
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

  RCLCPP_INFO(this->get_logger(), "Finished Initializing Params");
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

  lap_counter_subscriber_ =
      this->create_subscription<utfr_msgs::msg::Heartbeat>(
          topics::kCenterPathHeartbeat, 10,
          std::bind(&ControllerNode::lapCounterCB, this, _1));
  
  RCLCPP_INFO(this->get_logger(), "Finished Initializing Subscribers");
}

void ControllerNode::initPublishers() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kControllerHeartbeat, 10);

  target_state_publisher_ = this->create_publisher<utfr_msgs::msg::TargetState>(
      topics::kTargetState, 10);

  path_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      topics::kControllerPath, 10);

  pure_pursuit_point_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>(
          topics::kPurePursuitPoint, 10);
      
  RCLCPP_INFO(this->get_logger(), "Finished Initializing Publishers");
}

void ControllerNode::initTimers() {
  if (event_ == "accel") {
    last_lap_count_ = 2;
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBAccel, this));
  } 
  /*else if (event_ == "skidpad") {
    last_lap_count_ = 17;
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBSkidpad, this));
  } else if (event_ == "autocross") {
    last_lap_count_ = 21;
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBAutocross, this));
  } else if (event_ == "trackdrive") {
    last_lap_count_ = 40;
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBTrackdrive, this));
  }*/
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

void ControllerNode::lapCounterCB(const utfr_msgs::msg::Heartbeat &msg) {
  lap_count_ = msg.lap_count;
  RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
              " Controller Lap count: %d", lap_count_);
}

void ControllerNode::timerCBAccel() {
  const std::string function_name{"controller_timerCB:"};

  if (!path_ || !velocity_profile_) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                "Data not published or initialized yet. Using defaults.");
    return;
  }
  if(ego_state_ == nullptr){
    return;
  }
  // CODE GOES HERE
  double cur_s_ = 0;

  // if (controller_ == "stanley") {
  //   utfr_msgs::msg::TargetState target =
  //       stanleyController(stanley_gain_, max_velocity_, max_steering_angle_,
  //                         max_steering_rate_, *path_, cur_s_, ds_,
  //                         *velocity_profile_, baselink_location_, *ego_state_);
  //   target_ = target;
  // } else if (controller_ == "pure_pursuit") {
  //   utfr_msgs::msg::TargetState target = purePursuitController(
  //       max_velocity_, max_steering_angle_, *path_, cur_s_, ds_,
  //       *velocity_profile_, baselink_location_, *ego_state_,
  //       lookahead_distance_);
  //   target_ = target;
  // }

  utfr_msgs::msg::TargetState target;
  target.speed = 5.0;
  target.steering_angle = 0.0;

  if (!max_vel && (ego_state_->vel.twist.linear.x > target.speed)) {
    max_vel = true;
  }

  if (max_vel) {
    target.speed = 0.0;
    if (ego_state_->vel.twist.linear.x == target.speed) {
      max_vel = false;
    }
  }

  target_ = target;

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
  double cur_s_ = 0;

  // Controller
  utfr_msgs::msg::TargetState target = purePursuitController(
      max_steering_angle_, *path_, cur_s_, ds_, *velocity_profile_,
      baselink_location_, *ego_state_, base_lookahead_distance_,
      lookahead_distance_scaling_factor_);

  target_ = target;

  // print target state
  RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
              "Target steering: %f \n Target velocity: %f",
              target.steering_angle, target.speed);

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
  double cur_s_ = 0;

  // Controller
  utfr_msgs::msg::TargetState target = purePursuitController(
      max_steering_angle_, *path_, cur_s_, ds_, *velocity_profile_,
      baselink_location_, *ego_state_, base_lookahead_distance_,
      lookahead_distance_scaling_factor_);

  target_ = target;

  // print target state
  RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
              "Target steering: %f \n Target velocity: %f",
              target.steering_angle, target.speed);

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
  double cur_s_ = 0;

  // Controller
  utfr_msgs::msg::TargetState target = purePursuitController(
      max_steering_angle_, *path_, cur_s_, ds_, *velocity_profile_,
      baselink_location_, *ego_state_, base_lookahead_distance_,
      lookahead_distance_scaling_factor_);

  target_ = target;

  // print target state
  RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
              "Target steering: %f \n Target velocity: %f",
              target.steering_angle, target.speed);

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

utfr_msgs::msg::TargetState ControllerNode::purePursuitController(
    double max_steering_angle, utfr_msgs::msg::ParametricSpline spline_params,
    double cur_s, double ds, utfr_msgs::msg::VelocityProfile velocity_profile,
    double baselink_location, utfr_msgs::msg::EgoState ego_state,
    double base_lookahead_distance, double lookahead_distance_scaling_factor) {
  std::vector<geometry_msgs::msg::Pose> discretized_points =
      discretizeParametric(spline_params, cur_s, ds, num_points_);

  geometry_msgs::msg::PolygonStamped path_stamped;

  path_stamped.header.frame_id = "base_footprint";

  path_stamped.header.stamp = this->get_clock()->now();

  for (int i = 0; i < static_cast<int>(discretized_points.size()); i++) {
    geometry_msgs::msg::Point32 point;
    point.x = discretized_points[i].position.x;
    point.y = -discretized_points[i].position.y;
    point.z = 0.0;
    path_stamped.polygon.points.push_back(point);
  }

  path_publisher_->publish(path_stamped);

  double lookahead_distance;
  geometry_msgs::msg::Pose lookahead_point;
  bool found_lookahead = false;
  // Get desired velocity from the velocity profile
  double desired_velocity = velocity_profile.velocities[20];

  // Get dynamic lookahead distance
  for (const auto &wp : discretized_points) {
    double dx = wp.position.x - baselink_location;
    double dy = wp.position.y;

    lookahead_distance =
        std::min(base_lookahead_distance + lookahead_distance_scaling_factor *
                                               velocity_profile.velocities[1],
                 5.0);
    double distance = sqrt(dx * dx + dy * dy);
    if (distance > lookahead_distance) {
      lookahead_point = wp;
      found_lookahead = true;
      break;
    }
  }

  if (!found_lookahead) {
    lookahead_point = discretized_points[discretized_points.size() - 1];
  }

  // Plotting pure pursuit lookahead point
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_footprint";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "lookahead";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = lookahead_point.position.x;
  marker.pose.position.y = -lookahead_point.position.y;
  marker.pose.position.z = 0.0;

  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  pure_pursuit_point_publisher_->publish(marker);

  // Calculate angle to the lookahead point.
  double alpha = atan2(lookahead_point.position.y, lookahead_point.position.x);

  // Compute steering angle using Pure Pursuit.
  double delta = atan2(2.0 * wheel_base_ * sin(alpha), lookahead_distance);

  // Limit steering angle within bounds.
  delta = std::clamp(delta, -max_steering_angle, max_steering_angle);

  // Reduce speed if turning sharply or near max steering.
  if (abs(util::quaternionToYaw(discretized_points[5].orientation)) > 0.2 ||
      abs(delta) > max_steering_angle) {
    desired_velocity = 1.0;
  }

  rclcpp::Time curr_time = this->get_clock()->now();

  if (lap_count_ == last_lap_count_ || finished_event_) {
    if (start_finish_time) {
      start_time_ == curr_time;
      start_finish_time = false;
    }
    finished_event_ = true;
    desired_velocity = 1.0;
  }

  if (abs((curr_time - start_time_).seconds()) > 5.0 && finished_event_) {
    desired_velocity = 0.0;
  }

  if (desired_velocity != 0.0 && desired_velocity < 2.0) {
    desired_velocity = 2.0;
  }

  // Set target state values.
  utfr_msgs::msg::TargetState target;
  target.speed = desired_velocity;
  target.steering_angle = -delta;

  return target; // Return the target state.
}

} // namespace controller
} // namespace utfr_dv
