/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: path_optimization_node.cpp
* auth: Justin Lim
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

  update_rate_ = this->get_parameter("update_rate").as_double();
  event_ = this->get_parameter("event").as_string();
  skip_path_opt_ = this->get_parameter("skip_path_opt").as_bool();
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

  if (skip_path_opt_) {
    RCLCPP_WARN(this->get_logger(), "%s skipping path optimization",
                function_name.c_str());
    if (center_path_ != nullptr) {
      center_path_publisher_->publish(*center_path_);
      RCLCPP_INFO(this->get_logger(), "%s published original center path",
                  function_name.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "%s no center path to publish",
                  function_name.c_str());
    }
    this->publishHeartbeat(2);
    return;
  }

  // CODE GOES HERE
}

void PathOptimizationNode::timerCBSkidpad() {
  const std::string function_name{"path_opt_timerCB:"};

  if (skip_path_opt_) {
    RCLCPP_WARN(this->get_logger(), "%s skipping path optimization",
                function_name.c_str());
    if (center_path_ != nullptr) {
      center_path_publisher_->publish(*center_path_);
      RCLCPP_INFO(this->get_logger(), "%s published original center path",
                  function_name.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "%s no center path to publish",
                  function_name.c_str());
    }
    this->publishHeartbeat(2);
    return;
  }

  // CODE GOES HERE
}

void PathOptimizationNode::timerCBAutocross() {
  const std::string function_name{"path_opt_timerCB:"};

  if (skip_path_opt_) {
    RCLCPP_WARN(this->get_logger(), "%s skipping path optimization",
                function_name.c_str());
    if (center_path_ != nullptr) {
      center_path_publisher_->publish(*center_path_);
      RCLCPP_INFO(this->get_logger(), "%s published original center path",
                  function_name.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "%s no center path to publish",
                  function_name.c_str());
    }
    this->publishHeartbeat(2);
    return;
  }

  // CODE GOES HERE
}

void PathOptimizationNode::timerCBTrackdrive() {
  const std::string function_name{"path_opt_timerCB:"};

  if (skip_path_opt_) {
    RCLCPP_WARN(this->get_logger(), "%s skipping path optimization",
                function_name.c_str());
    if (center_path_ != nullptr) {
      center_path_publisher_->publish(*center_path_);
      RCLCPP_INFO(this->get_logger(), "%s published original center path",
                  function_name.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "%s no center path to publish",
                  function_name.c_str());
    }
    this->publishHeartbeat(2);
    return;
  }

  // CODE GOES HERE
}

std::vector<double> PathOptimizationNode::calculateVelocities(
    utfr_msgs::msg::ParametricSpline &spline, double L, int n,
    double a_lateral) {
  if (n <= 1)
    return {};
  auto first_derivative = [](std::vector<double> &c, double t, double t2,
                             double t3, double t4) {
    return 5 * c[0] * t4 + 4 * c[1] * t3 + 3 * c[2] * t2 + 2 * c[3] * t + c[4];
  };
  auto second_derivative = [](std::vector<double> &c, double t, double t2,
                              double t3) {
    return 20 * c[0] * t3 + 12 * c[1] * t2 + 6 * c[2] * t + 2 * c[3];
  };
  std::vector<double> &x = spline.x_params;
  std::vector<double> &y = spline.y_params;
  auto k = [&x, &y, &first_derivative, &second_derivative](double t) {
    double t2 = t * t, t3 = t2 * t, t4 = t3 * t;
    double x_first_derivative = first_derivative(x, t, t2, t3, t4);
    double x_second_derivative = second_derivative(x, t, t2, t3);
    double y_first_derivative = first_derivative(y, t, t2, t3, t4);
    double y_second_derivative = second_derivative(y, t, t2, t3);
    double numerator = x_first_derivative * y_second_derivative -
                       x_second_derivative * y_first_derivative;
    double val = x_first_derivative * x_first_derivative +
                 y_first_derivative * y_first_derivative;
    double denominator = val * sqrt(val);
    return abs(numerator / denominator);
  };
  std::vector<double> velocities;
  for (double t = 0; t <= L; t += L / (n - 1)) {
    velocities.push_back(sqrt(a_lateral / k(t)));
  }
  return velocities;
}

} // namespace path_optimization
} // namespace utfr_dv
