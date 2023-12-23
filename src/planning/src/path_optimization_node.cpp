/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: path_optimization_node.cpp
* auth: Justin Lim, Richard Li
* desc: path optimization node class
*/

#include <path_optimization_node.hpp>

namespace utfr_dv {
namespace path_optimization {

PathOptimizationNode::PathOptimizationNode() : Node("path_optimization_node") {
  RCLCPP_INFO(this->get_logger(), "Path Optimization Node Launched");
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

void PathOptimizationNode::initParams() {
  this->declare_parameter("update_rate", 33.33);
  this->declare_parameter("event", "accel");
  this->declare_parameter("skip_path_opt", false);
  this->declare_parameter("lookahead_distance", 5.0);
  this->declare_parameter("num_points", 100);
  this->declare_parameter("a_lateral", 1.0);
  this->declare_parameter("max_velocity", 2.0);

  update_rate_ = this->get_parameter("update_rate").as_double();
  event_ = this->get_parameter("event").as_string();
  skip_path_opt_ = this->get_parameter("skip_path_opt").as_bool();
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  num_points_ = this->get_parameter("num_points").as_int();
  a_lateral_max_ = this->get_parameter("a_lateral").as_double();
  max_velocity_ = this->get_parameter("max_velocity").as_double();
}

void PathOptimizationNode::initSubscribers() {
  ego_state_subscriber_ = this->create_subscription<utfr_msgs::msg::EgoState>(
      topics::kEgoState, 10,
      std::bind(&PathOptimizationNode::egoStateCB, this, _1));

  cone_map_subscriber_ = this->create_subscription<utfr_msgs::msg::ConeMap>(
      topics::kConeMap, 10,
      std::bind(&PathOptimizationNode::coneMapCB, this, _1));

  center_path_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ParametricSpline>(
          topics::kCenterPath, 10,
          std::bind(&PathOptimizationNode::centerPathCB, this, _1));
}

void PathOptimizationNode::initPublishers() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kPathOptimizationHeartbeat, 10);

  center_path_publisher_ =
      this->create_publisher<utfr_msgs::msg::ParametricSpline>(
          topics::kOptimizedCenterPath, 10);

  velocity_profile_publisher_ =
      this->create_publisher<utfr_msgs::msg::VelocityProfile>(
          topics::kVelocityProfile, 10);
}

void PathOptimizationNode::initTimers() {
  if (event_ == "accel") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&PathOptimizationNode::timerCBAccel, this));
  } else if (event_ == "skidpad") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&PathOptimizationNode::timerCBSkidpad, this));
  } else if (event_ == "autocross") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&PathOptimizationNode::timerCBAutocross, this));
  } else if (event_ == "trackdrive") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&PathOptimizationNode::timerCBTrackdrive, this));
  }
}

void PathOptimizationNode::initGGV(std::string filename) {
  std::ifstream GGV(filename);
  if (!GGV.is_open()) {
      std::cout << "Error opening file" << std::endl;
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
      for (int i = 0; i < line.length(); i++) {
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
      
      int vel = std::stod(data[2]);
      double latAccel = std::stod(data[1]);
      double longAccel = std::stod(data[0]);

      // Ignore negative accel values. We only care about positive values
      if (longAccel < 0) {
          continue;
      }

      GGV_velocities.insert(vel);

      // Add the values into vectors in the map
      if (GGV_vel_to_lat_accel.find(vel) == GGV_vel_to_lat_accel.end()) {
          GGV_vel_to_lat_accel[vel] = std::vector<double>();
      }
      if (GGV_vel_to_long_accel.find(vel) == GGV_vel_to_long_accel.end()) {
          GGV_vel_to_long_accel[vel] = std::vector<double>();
      }

      GGV_vel_to_lat_accel[vel].push_back(latAccel);
      GGV_vel_to_long_accel[vel].push_back(longAccel);
  }
  GGV.close();

  // Reverse the arrays in the hashmap because they are sorted high to low
  for (auto it = GGV_vel_to_lat_accel.begin(); it != GGV_vel_to_lat_accel.end(); it++) {
      std::reverse(it->second.begin(), it->second.end());
  }
  for (auto it = GGV_vel_to_long_accel.begin(); it != GGV_vel_to_long_accel.end(); it++) {
      std::reverse(it->second.begin(), it->second.end());
  }
}

void PathOptimizationNode::initHeartbeat() {
  heartbeat_.module.data = "path_optimization_node";
  heartbeat_.update_rate = update_rate_;
}

void PathOptimizationNode::publishHeartbeat(const int status) {
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_publisher_->publish(heartbeat_);
}

void PathOptimizationNode::egoStateCB(const utfr_msgs::msg::EgoState &msg) {
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

void PathOptimizationNode::coneMapCB(const utfr_msgs::msg::ConeMap &msg) {
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

void PathOptimizationNode::centerPathCB(
    const utfr_msgs::msg::ParametricSpline &msg) {
  if (center_path_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::ParametricSpline template_center_path;
    center_path_ = std::make_shared<utfr_msgs::msg::ParametricSpline>(
        template_center_path);
  }
  std::vector<double> template_vector = {0.0, 0.0, 0.0};
  center_path_->header = msg.header;
  center_path_->x_params = msg.x_params;
  center_path_->y_params = msg.y_params;
}

void PathOptimizationNode::timerCBAccel() {
  const std::string function_name{"path_opt_timerCB:"};

  if (!center_path_) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"), "No center path");
    return;
  }
  if (skip_path_opt_) {
    if (center_path_ != nullptr) {
      center_path_publisher_->publish(*center_path_);
    } else {
      RCLCPP_WARN(this->get_logger(), "%s no center path to publish",
                  function_name.c_str());
    }
  } else {
    // CODE GOES HERE TO OPTIMIZE PATH
  }

  std::vector<double> velocities = calculateVelocities(
      *center_path_, lookahead_distance_, num_points_, a_lateral_max_);
  std::vector<double> filtered_velocities =
      filterVelocities(velocities, ego_state_->vel.twist.linear.x,
                       lookahead_distance_, max_velocity_, 10, -10);
  utfr_msgs::msg::VelocityProfile velocity_profile_msg;
  velocity_profile_msg.velocities = filtered_velocities;
  velocity_profile_msg.header.stamp = this->get_clock()->now();
  velocity_profile_publisher_->publish(velocity_profile_msg);

  this->publishHeartbeat(2);
  return;
}

void PathOptimizationNode::timerCBSkidpad() {
  const std::string function_name{"path_opt_timerCB:"};

  if (!center_path_) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"), "No center path");
    return;
  }
  if (skip_path_opt_) {
    if (center_path_ != nullptr) {
      center_path_publisher_->publish(*center_path_);
    } else {
      RCLCPP_WARN(this->get_logger(), "%s no center path to publish",
                  function_name.c_str());
    }
  } else {
    // CODE GOES HERE TO OPTIMIZE PATH
  }

  std::vector<double> velocities = calculateVelocities(
      *center_path_, lookahead_distance_, num_points_, a_lateral_max_);
  std::vector<double> filtered_velocities =
      filterVelocities(velocities, ego_state_->vel.twist.linear.x,
                       lookahead_distance_, max_velocity_, 10, -10);
  utfr_msgs::msg::VelocityProfile velocity_profile_msg;
  velocity_profile_msg.velocities = filtered_velocities;
  velocity_profile_msg.header.stamp = this->get_clock()->now();
  velocity_profile_publisher_->publish(velocity_profile_msg);

  this->publishHeartbeat(2);
  return;
}

void PathOptimizationNode::timerCBAutocross() {
  const std::string function_name{"path_opt_timerCB:"};

  if (!center_path_) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"), "No center path");
    return;
  }
  if (skip_path_opt_) {
    if (center_path_ != nullptr) {
      center_path_publisher_->publish(*center_path_);
    } else {
      RCLCPP_WARN(this->get_logger(), "%s no center path to publish",
                  function_name.c_str());
    }
  } else {
    // CODE GOES HERE TO OPTIMIZE PATH
  }

  std::vector<double> velocities = calculateVelocities(
      *center_path_, lookahead_distance_, num_points_, a_lateral_max_);
  std::vector<double> filtered_velocities =
      filterVelocities(velocities, ego_state_->vel.twist.linear.x,
                       lookahead_distance_, max_velocity_, 10, -10);
  utfr_msgs::msg::VelocityProfile velocity_profile_msg;
  velocity_profile_msg.velocities = filtered_velocities;
  velocity_profile_msg.header.stamp = this->get_clock()->now();
  velocity_profile_publisher_->publish(velocity_profile_msg);

  this->publishHeartbeat(2);
  return;
}

void PathOptimizationNode::timerCBTrackdrive() {
  const std::string function_name{"path_opt_timerCB:"};

  if (!center_path_) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"), "No center path");
    return;
  }
  if (skip_path_opt_) {
    if (center_path_ != nullptr) {
      center_path_publisher_->publish(*center_path_);
    } else {
      RCLCPP_WARN(this->get_logger(), "%s no center path to publish",
                  function_name.c_str());
    }
  } else {
    // CODE GOES HERE TO OPTIMIZE PATH
  }

  std::vector<double> velocities = calculateVelocities(
      *center_path_, lookahead_distance_, num_points_, a_lateral_max_);
  std::vector<double> filtered_velocities =
      filterVelocities(velocities, ego_state_->vel.twist.linear.x,
                       lookahead_distance_, max_velocity_, 10, -10);
  utfr_msgs::msg::VelocityProfile velocity_profile_msg;
  velocity_profile_msg.velocities = filtered_velocities;
  velocity_profile_msg.header.stamp = this->get_clock()->now();
  velocity_profile_publisher_->publish(velocity_profile_msg);

  this->publishHeartbeat(2);
  return;
}

std::vector<double> PathOptimizationNode::calculateVelocities(
    utfr_msgs::msg::ParametricSpline &spline, double L, int n,
    double a_lateral) {
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

std::vector<double> PathOptimizationNode::filterVelocities(
    std::vector<double> &max_velocities, double current_velocity,
    double distance, double max_velocity, double max_acceleration,
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

double PathOptimizationNode::calculateLongitAccel(double velocity,
                                                  double a_lateral) {
  // get the closest velocity in the GGV data
  if (velocity >= *GGV_velocities.rbegin()) {
    velocity = *GGV_velocities.rbegin();
  }
  if (velocity >= (*GGV_velocities.rbegin() + *--GGV_velocities.rbegin())/2.0) {
      velocity = *GGV_velocities.rbegin();
  } else {
    // round velocity to nearest even number
    int roundedVel = std::round(vel);
    if (roundedVel % 2 != 0) {
        roundedVel += (vel >= roundedVel) ? 1 : -1;
    }
  }

  std::vector<double> latAccels = GGV_vel_to_lat_accel[roundedVel];
  std::vector<double> longAccels = GGV_vel_to_long_accel[roundedVel];

  for (int i = 0; i < latAccels.size(); i++) {
      std::cout << latAccels[i] << ", " << longAccels[i] << std::endl;
  }

  // binary search for the index of the closest latAccel to the given latAccel
  int idx = std::lower_bound(latAccels.begin(), latAccels.end(), latAccel)-latAccels.begin();
  if (idx > 0) {
      if (std::abs(latAccels[idx-1] - latAccel) < std::abs(latAccels[idx] - latAccel)) {
          idx--;
      }
  }
  if (idx > latAccels.size() - 1) {
      idx = latAccels.size() - 1;
  }

  return longAccels[idx];
}

} // namespace path_optimization
} // namespace utfr_dv
