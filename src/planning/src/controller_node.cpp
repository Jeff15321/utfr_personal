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
  // RCLCPP_INFO(this->get_logger(), "Center Path Node Launched");
  this->initParams();
  this->initEvent();
  this->initHeartbeat();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::NOT_READY);
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initGGV(ament_index_cpp::get_package_share_directory("planning") +
                "/GGV.csv");
  publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
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
  this->declare_parameter("max_steering_angle", 1.0);
  this->declare_parameter("max_steering_rate", 0.4);
  this->declare_parameter("max_tire", 1.5);
  this->declare_parameter("baselink_location", 0.79);
  this->declare_parameter("wheel_base", 1.58);
  this->declare_parameter("num_points", 1000);
  this->declare_parameter("base_lookahead_distance", 1.0);
  this->declare_parameter("lookahead_scaling_factor", 0.15);

  this->declare_parameter("skip_path_opt", false);
  this->declare_parameter("lookahead_distance", 5.0);
  this->declare_parameter("a_lateral", 1.0);

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
  base_lookahead_distance_ =
      this->get_parameter("base_lookahead_distance").as_double();
  lookahead_distance_scaling_factor_ =
      this->get_parameter("lookahead_scaling_factor").as_double();

  skip_path_opt_ = this->get_parameter("skip_path_opt").as_bool();
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  a_lateral_max_ = this->get_parameter("a_lateral").as_double();

  start_time_ = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "Event: %s", event_.c_str());
}

void ControllerNode::initSubscribers() {
  ego_state_subscriber_ = this->create_subscription<utfr_msgs::msg::EgoState>(
      topics::kEgoState, 10, std::bind(&ControllerNode::egoStateCB, this, _1));

  path_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ParametricSpline>(
          topics::kCenterPath, 10,
          std::bind(&ControllerNode::pathCB, this, _1));

  point_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
      topics::kSkidpadCenterPoint, 10,
      std::bind(&ControllerNode::pointCB, this, _1));

  // mission_subscriber_ =
  // this->create_subscription<utfr_msgs::msg::SystemStatus>(
  //     topics::kSystemStatus, 10,
  //     std::bind(&ControllerNode::missionCB, this, std::placeholders::_1));
}

void ControllerNode::initPublishers() {
  target_state_publisher_ = this->create_publisher<utfr_msgs::msg::TargetState>(
      topics::kTargetState, 10);

  path_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      topics::kControllerPath, 10);

  pure_pursuit_point_publisher_ =
      this->create_publisher<visualization_msgs::msg::Marker>(
          topics::kPurePursuitPoint, 10);
}

void ControllerNode::initTimers() {
  main_timer_.reset();

  if (!event_set_) {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::homeScreenCB, this));

    return;
  }

  if (event_ == "accel") {
    last_lap_count_ = 2;
    use_mapping_ = false;
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBAccel, this));
  } else if (event_ == "skidpad") {
    last_lap_count_ = 17;
    use_mapping_ = false;
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBSkidpad, this));
  } else if (event_ == "autocross") {
    last_lap_count_ = 21;
    use_mapping_ = true;
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBAutocross, this));
  } else if (event_ == "trackdrive") {
    last_lap_count_ = 40;
    use_mapping_ = true;
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBTrackdrive, this));
  } else if (event_ == "EBSTest") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBEBS, this));
  } else if (event_ == "ASTest") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&ControllerNode::timerCBAS, this));
  }
}

void ControllerNode::initEvent() { // TODO: TEST
  if (event_ == "read") {
    mission_subscriber_ =
        this->create_subscription<utfr_msgs::msg::SystemStatus>(
            topics::kSystemStatus, 10,
            std::bind(&ControllerNode::missionCB, this, std::placeholders::_1));
  }
}

void ControllerNode::missionCB(const utfr_msgs::msg::SystemStatus &msg) {
  switch (msg.ami_state) {
  case utfr_msgs::msg::SystemStatus::AMI_STATE_ACCELERATION: {
    event_ = "accel";
    event_set_ = true;
    break;
  }
  case utfr_msgs::msg::SystemStatus::AMI_STATE_SKIDPAD: {
    event_ = "skidpad";
    event_set_ = true;
    break;
  }
  case utfr_msgs::msg::SystemStatus::AMI_STATE_TRACKDRIVE: {
    event_ = "trackdrive";
    event_set_ = true;
    break;
  }
  case utfr_msgs::msg::SystemStatus::AMI_STATE_EBSTEST: {
    event_ = "EBSTest";
    event_set_ = true;
    break;
  }
  case utfr_msgs::msg::SystemStatus::AMI_STATE_TESTING:
  case utfr_msgs::msg::SystemStatus::AMI_STATE_INSPECTION: {
    event_ = "ASTest";
    event_set_ = true;
    break;
  }
  case utfr_msgs::msg::SystemStatus::AMI_STATE_AUTOCROSS: {
    event_ = "autocross";
    event_set_ = true;
    break;
  }
  }
  if (as_state == 2 && msg.as_state == 3) {
    start_time_ = this->get_clock()->now();
  }

  as_state = msg.as_state;

  std::cout << "PLANNING AS STATE: " << as_state << std::endl;
  // std::cout << "PLANNING AMI: " << msg.ami_state << std::endl;
}

void ControllerNode::initGGV(std::string filename) {
  std::ifstream GGV(filename);
  if (!GGV.is_open()) {
    std::cerr << "Error opening file" << filename << std::endl;
  }

  if (!GGV.eof()) {
    std::string line[3];
    std::getline(GGV, line[0], ',');
    std::getline(GGV, line[1], ',');
    std::getline(GGV, line[2]);
  }

  // Parse CSV
  std::string line;
  while (std::getline(GGV, line)) {
    if (line == "" || line == "\n" || line == "\r") {
      continue;
    }

    // parse each element of line into a vector
    std::vector<std::string> data;
    std::string temp = "";
    for (size_t i = 0; i < line.length(); i++) {
      if (line[i] == ',') {
        data.push_back(temp);
        temp = "";
      } else {
        temp += line[i];
      }
    }
    data.push_back(temp);

    // if the line is not formatted correctly, skip it
    if (data.size() < 3) {
      continue;
    }

    double vel = ((int)(std::stod(data[2]) * 100)) / 100.0;
    double lat_accel = std::stod(data[1]);
    double long_accel = std::stod(data[0]);

    // Ignore negative accel values. We only care about positive values
    if (long_accel < 0) {
      continue;
    }

    GGV_velocities_.insert(vel);

    // Add the values into vectors in the map
    if (GGV_vel_to_lat_accel_.find(vel) == GGV_vel_to_lat_accel_.end()) {
      GGV_vel_to_lat_accel_[vel] = std::vector<double>();
    }
    if (GGV_vel_to_long_accel_.find(vel) == GGV_vel_to_long_accel_.end()) {
      GGV_vel_to_long_accel_[vel] = std::vector<double>();
    }

    GGV_vel_to_lat_accel_[vel].push_back(lat_accel);
    GGV_vel_to_long_accel_[vel].push_back(long_accel);
  }
  GGV.close();

  // Reverse the arrays in the hashmap because they are sorted high to low
  for (auto it = GGV_vel_to_lat_accel_.begin();
       it != GGV_vel_to_lat_accel_.end(); it++) {
    std::reverse(it->second.begin(), it->second.end());
  }
  for (auto it = GGV_vel_to_long_accel_.begin();
       it != GGV_vel_to_long_accel_.end(); it++) {
    std::reverse(it->second.begin(), it->second.end());
  }
}

void ControllerNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kControllerHeartbeat, 10);
  heartbeat_.module.data = "planning_controller";
  heartbeat_.update_rate = update_rate_;
}

void ControllerNode::publishHeartbeat(const int status) {
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  if (lap_count_ <= 25) {
    heartbeat_.lap_count = 0;
  } else {
    heartbeat_.lap_count = lap_count_ - 29;
  }
  heartbeat_publisher_->publish(heartbeat_);
}

void ControllerNode::egoStateCB(const utfr_msgs::msg::EgoState &msg) {
  if (ego_state_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::EgoState template_ego;
    ego_state_ = std::make_shared<utfr_msgs::msg::EgoState>(template_ego);
  }
  if (use_mapping_) {
    ego_state_->header = msg.header;
    ego_state_->pose = msg.pose;
    ego_state_->vel = msg.vel;
    ego_state_->accel = msg.accel;
    ego_state_->steering_angle = msg.steering_angle;
  } else if (!use_mapping_) {
    ego_state_->header = msg.header;
    ego_state_->pose = msg.pose;
    ego_state_->pose.pose.position.x = 0.0;
    ego_state_->pose.pose.position.y = 0.0;
    ego_state_->pose.pose.position.z = 0.0;
    ego_state_->vel = msg.vel;
    ego_state_->accel = msg.accel;
    ego_state_->steering_angle = msg.steering_angle;
  }
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
  path_->skidpad_params = msg.skidpad_params;
  path_->lap_count = msg.lap_count;
  lap_count_ = msg.lap_count;
}

void ControllerNode::pointCB(const geometry_msgs::msg::Pose &msg) {
  point_ = std::make_shared<geometry_msgs::msg::Pose>(msg);
}

void ControllerNode::homeScreenCB() {
  if (event_set_)
  {
    initTimers();
  }
  if (as_state != 3) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }
}

void ControllerNode::timerCBAccel() {
  const std::string function_name{"controller_timerCB:"};

  if (as_state != 3) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }

  try {
    if (!path_ || !ego_state_) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                  "Data not published or initialized yet. Using defaults.");
      return;
    }
    if (velocity_profile_ == nullptr) {
      // first initialization:
      utfr_msgs::msg::VelocityProfile template_velocity_profile;
      velocity_profile_ = std::make_shared<utfr_msgs::msg::VelocityProfile>(
          template_velocity_profile);
    }

    double cur_s_ = 0;

    std::vector<double> velocities = calculateVelocities(
        *path_, lookahead_distance_, num_points_, a_lateral_max_);
    std::vector<double> filtered_velocities =
        filterVelocities(velocities, ego_state_->vel.twist.linear.x,
                         lookahead_distance_, max_velocity_, 10, -10);

    velocity_profile_->velocities = filtered_velocities;
    velocity_profile_->header.stamp = this->get_clock()->now();

    // Controller
    utfr_msgs::msg::TargetState target = purePursuitController(
        max_steering_angle_, *path_, cur_s_, ds_, *velocity_profile_,
        baselink_location_, base_lookahead_distance_,
        lookahead_distance_scaling_factor_);

    target_ = target;

    // print target state
    // RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
    //             "Target steering: %f \n Target velocity: %f",
    //             target.steering_angle, target.speed);

    // publish target state
    target_state_publisher_->publish(target_);

    if (!finished_and_stopped_)
      publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
    else
      publishHeartbeat(utfr_msgs::msg::Heartbeat::FINISH);
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void ControllerNode::timerCBSkidpad() {
  const std::string function_name{"controller_timerCB:"};

  if (as_state != 3) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }

  try {
    if (!(path_ || point_) || !ego_state_) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                  "Data not published or initialized yet. Using defaults.");
      return;
    }
    if (!point_) {
      if (velocity_profile_ == nullptr) {
        // first initialization:
        utfr_msgs::msg::VelocityProfile template_velocity_profile;
        velocity_profile_ = std::make_shared<utfr_msgs::msg::VelocityProfile>(
            template_velocity_profile);
      }

      std::vector<double> velocities;
      if (lap_count_ > 11 && lap_count_ < 16) {
        velocities =
            calculateSkidpadVelocities(*path_, num_points_, a_lateral_max_);
      } else {
        velocities = calculateVelocities(*path_, lookahead_distance_,
                                         num_points_, a_lateral_max_);
      }

      std::vector<double> filtered_velocities =
          filterVelocities(velocities, ego_state_->vel.twist.linear.x,
                           lookahead_distance_, max_velocity_, 10.0, -2.5);

      velocity_profile_->velocities = filtered_velocities;
      velocity_profile_->header.stamp = this->get_clock()->now();

      double cur_s_ = 0;

      // Controller
      target_ = purePursuitController(max_steering_angle_, *path_, cur_s_, ds_,
                                      *velocity_profile_, baselink_location_,
                                      base_lookahead_distance_,
                                      lookahead_distance_scaling_factor_);
    } else {
      target_ =
          purePursuitController(max_steering_angle_, *point_, max_velocity_);
      point_ = nullptr;
    }

    // print target state
    // RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
    //             "Target steering: %f \n Target velocity: %f",
    //             target.steering_angle, target.speed);

    // publish target state
    target_state_publisher_->publish(target_);
    if (!finished_and_stopped_)
      publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
    else
      publishHeartbeat(utfr_msgs::msg::Heartbeat::FINISH);
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void ControllerNode::timerCBAutocross() {
  const std::string function_name{"controller_timerCB:"};

  if (as_state != 3) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }

  try {
    if (!path_ || !ego_state_) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                  "Data not published or initialized yet. Using defaults.");
      return;
    }

    if (velocity_profile_ == nullptr) {
      // first initialization:
      utfr_msgs::msg::VelocityProfile template_velocity_profile;
      velocity_profile_ = std::make_shared<utfr_msgs::msg::VelocityProfile>(
          template_velocity_profile);
    }

    std::vector<double> velocities = calculateVelocities(
        *path_, lookahead_distance_, num_points_, a_lateral_max_);
    std::vector<double> filtered_velocities =
        filterVelocities(velocities, ego_state_->vel.twist.linear.x,
                         lookahead_distance_, max_velocity_, 10, -10);

    velocity_profile_->velocities = filtered_velocities;
    velocity_profile_->header.stamp = this->get_clock()->now();

    double cur_s_ = 0;

    // Controller
    utfr_msgs::msg::TargetState target = purePursuitController(
        max_steering_angle_, *path_, cur_s_, ds_, *velocity_profile_,
        baselink_location_, base_lookahead_distance_,
        lookahead_distance_scaling_factor_);

    target_ = target;

    // print target state
    // RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
    //             "Target steering: %f \n Target velocity: %f",
    //             target.steering_angle, target.speed);

    // publish target state
    target_state_publisher_->publish(target_);
    if (!finished_and_stopped_)
      publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
    else
      publishHeartbeat(utfr_msgs::msg::Heartbeat::FINISH);
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void ControllerNode::timerCBTrackdrive() {
  const std::string function_name{"controller_timerCB:"};

  if (as_state != 3) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }

  try {

    if (!path_ || !ego_state_) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                  "Data not published or initialized yet. Using defaults.");
      return;
    }

    if (velocity_profile_ == nullptr) {
      // first initialization:
      utfr_msgs::msg::VelocityProfile template_velocity_profile;
      velocity_profile_ = std::make_shared<utfr_msgs::msg::VelocityProfile>(
          template_velocity_profile);
    }

    std::vector<double> velocities = calculateVelocities(
        *path_, lookahead_distance_, num_points_, a_lateral_max_);
    std::vector<double> filtered_velocities =
        filterVelocities(velocities, ego_state_->vel.twist.linear.x,
                         lookahead_distance_, max_velocity_, 10, -10);

    velocity_profile_->velocities = filtered_velocities;
    velocity_profile_->header.stamp = this->get_clock()->now();

    double cur_s_ = 0;

    // Controller
    utfr_msgs::msg::TargetState target = purePursuitController(
        max_steering_angle_, *path_, cur_s_, ds_, *velocity_profile_,
        baselink_location_, base_lookahead_distance_,
        lookahead_distance_scaling_factor_);

    target_ = target;

    // print target state
    // RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
    //             "Target steering: %f \n Target velocity: %f",
    //             target.steering_angle, target.speed);

    // publish target state
    target_state_publisher_->publish(target_);

    if (!finished_and_stopped_)
      publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
    else
      publishHeartbeat(utfr_msgs::msg::Heartbeat::FINISH);
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void ControllerNode::timerCBEBS() {
  const std::string function_name{"controller_timerCB:"};

  if (as_state != 3) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }

  try {
    if (!path_) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                  "Data not published or initialized yet. Using defaults.");
      return;
    }
    if (velocity_profile_ == nullptr) {
      // first initialization:
      utfr_msgs::msg::VelocityProfile template_velocity_profile;
      velocity_profile_ = std::make_shared<utfr_msgs::msg::VelocityProfile>(
          template_velocity_profile);
    }

    double cur_s_ = 0;

    std::vector<double> velocities = calculateVelocities(
        *path_, lookahead_distance_, num_points_, a_lateral_max_);
    std::vector<double> filtered_velocities =
        filterVelocities(velocities, ego_state_->vel.twist.linear.x,
                         lookahead_distance_, max_velocity_, 10, -10);

    velocity_profile_->velocities = filtered_velocities;
    velocity_profile_->header.stamp = this->get_clock()->now();

    // Controller
    utfr_msgs::msg::TargetState target = purePursuitController(
        max_steering_angle_, *path_, cur_s_, ds_, *velocity_profile_,
        baselink_location_, base_lookahead_distance_,
        lookahead_distance_scaling_factor_);

    target_ = target;

    double time = this->get_clock_now();
    double time_diff = (time - start_time_).seconds();

    if (time_diff > 10.0){
      target_.speed = 0.0;
      target_.steering_angle = 0.0;
      publishHeartbeat(utfr_msgs::msg::Heartbeat::FINISH);
      return;
    }

    // print target state
    // RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
    //             "Target steering: %f \n Target velocity: %f",
    //             target.steering_angle, target.speed);

    // publish target state
    target_state_publisher_->publish(target_);

    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void ControllerNode::timerCBAS() {
  if (as_state != 3) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"), "here lol");
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }
  double curr_time = this->now().seconds();
  double time_diff = curr_time - start_time_.seconds();

  if (as_state == 3 && time_diff < 30.0) {
    target_.speed = 1.0;
    target_.steering_angle = sin(time_diff * 3.1415 / 3) * max_steering_angle_;
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
  } else if (as_state == 3) {
    target_.speed = 0.0;
    target_.steering_angle = 0.0;
    publishHeartbeat(utfr_msgs::msg::Heartbeat::FINISH);
  } else {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
    target_.speed = 0.0;
    target_.steering_angle = 0.0;
  }

  std::cout << "Speed: " << target_.speed << " STR: " << target_.steering_angle
            << std::endl;

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

std::vector<geometry_msgs::msg::Pose> ControllerNode::discretizeCircle(
    const utfr_msgs::msg::ParametricSpline &spline_params, int num_points) {
  std::vector<geometry_msgs::msg::Pose> discretized_points;

  double x0 = spline_params.skidpad_params[0];
  double y0 = spline_params.skidpad_params[1];
  double r = spline_params.skidpad_params[2];

  if (lap_count_ == 12 || lap_count_ == 13) {
    for (int i = num_points - 1; i > -1; i--) {
      double cur_ang = static_cast<double>(i) / num_points * 3.1415 / 2;
      double x = x0 + r * cos(cur_ang);
      double y = y0 - r * sin(cur_ang);
      double next_cur_ang = cur_ang + 0.00000001;
      double next_x = x0 + r * cos(next_cur_ang);
      double next_y = y0 - r * sin(next_cur_ang);
      double last_cur_ang = cur_ang - 0.00000001;
      double last_x = x0 + r * cos(last_cur_ang);
      double last_y = y0 - r * sin(last_cur_ang);
      double heading = atan2(next_y - last_y, next_x - last_x);
      geometry_msgs::msg::Pose point;
      point.position.x = x;
      point.position.y = y;
      point.orientation = util::yawToQuaternion(heading);
      discretized_points.push_back(point);
    }
  } else if (lap_count_ == 14 || lap_count_ == 15) {
    for (int i = num_points - 1; i > -1; i--) {
      double cur_ang = static_cast<double>(i) / num_points * 3.1415 / 2;
      double x = x0 + r * cos(cur_ang);
      double y = y0 + r * sin(cur_ang);
      double next_cur_ang = cur_ang + 0.00000001;
      double next_x = x0 + r * cos(next_cur_ang);
      double next_y = y0 - r * sin(next_cur_ang);
      double last_cur_ang = cur_ang - 0.00000001;
      double last_x = x0 + r * cos(last_cur_ang);
      double last_y = y0 - r * sin(last_cur_ang);
      double heading = atan2(next_y - last_y, next_x - last_x);
      geometry_msgs::msg::Pose point;
      point.position.x = x;
      point.position.y = y;
      point.orientation = util::yawToQuaternion(heading);
      discretized_points.push_back(point);
    }
  }
  return discretized_points;
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
    double baselink_location, double base_lookahead_distance,
    double lookahead_distance_scaling_factor) {

  std::vector<geometry_msgs::msg::Pose> discretized_points;
  if (lap_count_ < 16 && lap_count_ > 11) {
    discretized_points = discretizeCircle(spline_params, num_points_);
  } else {
    discretized_points =
        discretizeParametric(spline_params, cur_s, ds, num_points_);
  }

  geometry_msgs::msg::PolygonStamped path_stamped;

  path_stamped.header.frame_id = "ground";

  path_stamped.header.stamp = this->get_clock()->now();

  for (int i = 0; i < static_cast<int>(discretized_points.size()); i++) {
    geometry_msgs::msg::Point32 point;
    point.x = discretized_points[i].position.x;
    point.y = discretized_points[i].position.y;
    point.z = 0.0;
    path_stamped.polygon.points.push_back(point);
  }

  for (int i = discretized_points.size() - 1; i > -1; i--) {
    geometry_msgs::msg::Point32 point;
    point.x = discretized_points[i].position.x;
    point.y = discretized_points[i].position.y;
    point.z = 0.0;
    path_stamped.polygon.points.push_back(point);
  }

  path_publisher_->publish(path_stamped);

  double lookahead_distance =
      std::min(base_lookahead_distance + lookahead_distance_scaling_factor *
                                             velocity_profile.velocities[1],
               5.0);
  geometry_msgs::msg::Pose lookahead_point;
  bool found_lookahead = false;
  // Get desired velocity from the velocity profile
  double desired_velocity = velocity_profile.velocities[20];

  // Get dynamic lookahead distance
  for (const auto &wp : discretized_points) {
    double dx = wp.position.x + (wheel_base_ - baselink_location);
    double dy = wp.position.y;
    double distance = sqrt(dx * dx + dy * dy);
    if (distance > lookahead_distance && dx > 0.0) {
      lookahead_point = wp;
      found_lookahead = true;
      break;
    }
  }

  if (!found_lookahead) {
    lookahead_point = discretized_points.back();
  }

  return purePursuitController(max_steering_angle, lookahead_point,
                               desired_velocity);
}

utfr_msgs::msg::TargetState
ControllerNode::purePursuitController(double max_steering_angle,
                                      geometry_msgs::msg::Pose lookahead_point,
                                      double desired_velocity) {

  // Plotting pure pursuit lookahead point
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "ground";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "lookahead";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = lookahead_point.position.x;
  marker.pose.position.y = lookahead_point.position.y;
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
  double dx = lookahead_point.position.x + (wheel_base_ - baselink_location_);
  double dy = lookahead_point.position.y;
  double lookahead_distance = sqrt(dx * dx + dy * dy);
  double alpha = atan2(dy, dx);

  // Compute steering angle using Pure Pursuit.
  double delta = atan2(2.0 * wheel_base_ * sin(alpha), lookahead_distance);

  // Limit steering angle within bounds.
  delta = std::clamp(delta, -max_steering_angle, max_steering_angle);

  double max_steering_rate =
      static_cast<double>(109 * 100 / 22.0 / 60.0 / update_rate_);
  if (abs(delta - last_steering_angle_) > max_steering_rate) {
    if (delta > last_steering_angle_) {
      delta = last_steering_angle_ + max_steering_rate;
    } else {
      delta = last_steering_angle_ - max_steering_rate;
    }
  }
  last_steering_angle_ = delta;

  // Reduce speed if turning sharply or near max steering.
  // if ((abs(util::quaternionToYaw(discretized_points[5].orientation)) > 0.2 ||
  //      abs(delta) > max_steering_angle) &&
  //     (lap_count_ > 15 || lap_count_ < 12)) {
  //   desired_velocity = std::max(desired_velocity - 2, 1.0);
  // }

  rclcpp::Time curr_time = this->get_clock()->now();

  if (lap_count_ == last_lap_count_ || finished_event_) {
    if (start_finish_time) {
      start_time_ == curr_time;
      start_finish_time = false;
    }
    finished_event_ = true;
    desired_velocity = 1.0;
  }

  // if (abs((curr_time - start_time_).seconds()) > 5.0 && finished_event_) {
  //   desired_velocity = 0.0;
  //   finished_and_stopped_ = true;
  // }

  if (finished_event_) {
    desired_velocity = 0.0;
    finished_and_stopped_ = true;
  }

  if (desired_velocity != 0.0 && desired_velocity < 2.0) {
    desired_velocity = 2.0;
  }

  if (desired_velocity > max_velocity_){
    desired_velocity = max_velocity_;
  }

  // Set target state values.
  utfr_msgs::msg::TargetState target;
  target.speed = desired_velocity;
  target.steering_angle = delta;

  return target; // Return the target state.
}

double ControllerNode::k(std::vector<double> c) { return 1 / c[2]; }

std::vector<double> ControllerNode::calculateSkidpadVelocities(
    utfr_msgs::msg::ParametricSpline &spline, int n, double a_lateral) {
  if (n <= 1)
    return {};

  std::vector<double> params = spline.skidpad_params;
  std::vector<double> velocities;

  for (int s = 0; s <= num_points_; s++) {
    try {
      velocities.push_back(sqrt(a_lateral / k(params)));
    } catch (int e) {
      velocities.push_back(5.0);
    }
  }
  return velocities;
}

std::vector<double>
ControllerNode::calculateVelocities(utfr_msgs::msg::ParametricSpline &spline,
                                    double L, int n, double a_lateral) {
  if (n <= 1)
    return {};
  auto first_derivative = [](std::vector<double> &c, double s, double s2,
                             double s3, double s4) {
    return 5 * c[0] * s4 + 4 * c[1] * s3 + 3 * c[2] * s2 + 2 * c[3] * s + c[4];
  };
  auto second_derivative = [](std::vector<double> &c, double s, double s2,
                              double s3) {
    return 20 * c[0] * s3 + 12 * c[1] * s2 + 6 * c[2] * s + 2 * c[3];
  };
  std::vector<double> &x = spline.x_params;
  std::vector<double> &y = spline.y_params;
  auto k = [&x, &y, &first_derivative, &second_derivative](double s) {
    double s2 = s * s, s3 = s2 * s, s4 = s3 * s;
    double x_first_derivative = first_derivative(x, s, s2, s3, s4);
    double x_second_derivative = second_derivative(x, s, s2, s3);
    double y_first_derivative = first_derivative(y, s, s2, s3, s4);
    double y_second_derivative = second_derivative(y, s, s2, s3);
    double numerator = x_first_derivative * y_second_derivative -
                       x_second_derivative * y_first_derivative;
    double val = x_first_derivative * x_first_derivative +
                 y_first_derivative * y_first_derivative;
    double denominator = val * sqrt(val);
    return abs(numerator / denominator);
  };
  std::vector<double> velocities;
  for (double s = 0; s <= L; s += L / (n - 1)) {
    velocities.push_back(sqrt(a_lateral / k(s)));
  }
  return velocities;
}

std::vector<double>
ControllerNode::filterVelocities(std::vector<double> &max_velocities,
                                 double current_velocity, double distance,
                                 double max_velocity, double max_acceleration,
                                 double min_acceleration) {
  if (max_velocities.empty()) { // Not possible to filter
    return {};
  }
  // make sure all velocities are reasonable
  for (double &v : max_velocities) {
    if (std::isnan(v)) { // straight line
      v = max_velocity;
    }
    v = std::min(abs(v), max_velocity);
  }

  int n = max_velocities.size();

  if (n < 2) { // Can't filter this
    return {current_velocity};
  }

  auto accel = [](double vf, double vi, double d) {
    // vf^2 = vi^2 + 2ad
    return (vf * vf - vi * vi) / (2 * d);
  };
  auto velo = [](double vi, double a, double d) {
    // vf^2 = vi^2 + 2ad
    return sqrt(vi * vi + 2 * a * d);
  };

  std::vector<double> velocities = max_velocities;
  double ds = distance / (n - 1);

  // Backward pass (make min_acceleration isn't exceeded)
  for (int i = n - 1; i > 0; i--) {
    double a = accel(velocities[i], velocities[i - 1], ds);
    if (a < min_acceleration) { // limit deceleration
      velocities[i - 1] = velo(velocities[i], -min_acceleration, ds);
    }
  }

  // Forward Pass (find the max possible velocities starting at current)
  velocities[0] = current_velocity;
  for (int i = 1; i < n; i++) {
    double a = accel(velocities[i], velocities[i - 1], ds);
    a = std::max(a, min_acceleration); // limit deceleration
    a = std::min(a, max_acceleration); // limit acceleration
    velocities[i] = velo(velocities[i - 1], a, ds);
  }
  return velocities;
}

double ControllerNode::getMaxLongAccelGGV(double velocity, double a_lateral) {
  velocity = std::max(velocity, 0.0);
  double rounded_vel = velocity;
  // get the closest velocity in the GGV data
  if (velocity >=
      (*GGV_velocities_.rbegin() + *--GGV_velocities_.rbegin()) / 2.0) {
    rounded_vel = *GGV_velocities_.rbegin();
  } else {
    // round velocity to nearest even number
    rounded_vel = std::round(velocity);
    if ((int)rounded_vel % 2 != 0) {
      rounded_vel += (velocity >= rounded_vel) ? 1 : -1;
    }
  }

  std::vector<double> lat_accels = GGV_vel_to_lat_accel_[rounded_vel];
  std::vector<double> long_accels = GGV_vel_to_long_accel_[rounded_vel];

  // binary search for the index of the closest lat_accel to the given lat_accel
  int idx = std::lower_bound(lat_accels.begin(), lat_accels.end(), a_lateral) -
            lat_accels.begin();
  if (idx > 0) {
    if (std::abs(lat_accels[idx - 1] - a_lateral) <
        std::abs(lat_accels[idx] - a_lateral)) {
      idx--;
    }
  }
  if (idx > (int)lat_accels.size() - 1) {
    idx = lat_accels.size() - 1;
  }

  return long_accels[idx];
}

} // namespace controller
} // namespace utfr_dv