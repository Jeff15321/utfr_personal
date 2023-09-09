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

  update_rate_ = this->get_parameter("update_rate").as_double();
  event_ = this->get_parameter("event").as_string();
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

  // CODE GOES HERE
}

void ControllerNode::timerCBSkidpad() {
  const std::string function_name{"controller_timerCB:"};

  // CODE GOES HERE
}

void ControllerNode::timerCBAutocross() {
  const std::string function_name{"controller_timerCB:"};

  // CODE GOES HERE
}

void ControllerNode::timerCBTrackdrive() {
  const std::string function_name{"controller_timerCB:"};

  // CODE GOES HERE
}

} // namespace controller
} // namespace utfr_dv
