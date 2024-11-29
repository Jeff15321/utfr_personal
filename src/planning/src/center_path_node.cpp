/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: center_path_node.cpp
* auth: Justin Lim
* desc: center path node class
*/

#include <center_path_node.hpp>

namespace utfr_dv {
namespace center_path {

CenterPathNode::CenterPathNode() : Node("center_path_node") {
  // RCLCPP_INFO(this->get_logger(), "Center Path Node Launched");
  this->initParams();
  this->initHeartbeat();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::NOT_READY);
  this->initSubscribers();
  this->initPublishers();
  this->initTransforms();
  this->initEvent();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
}

void CenterPathNode::initParams() {
  this->declare_parameter("update_rate", 33.33);
  this->declare_parameter("event", "read");
  this->declare_parameter("big_radius", 10.625);
  this->declare_parameter("small_radius", 7.625);
  this->declare_parameter("threshold_radius", 0.8);
  this->declare_parameter("threshold_cones", 3);
  this->declare_parameter("global_path", 0);
  this->declare_parameter("colourblind", 0);
  this->declare_parameter("max_velocity", 5.0);
  this->declare_parameter("lookahead_scaling_factor", 0.6);
  this->declare_parameter("base_lookahead_distance", 3.0);
  this->declare_parameter("use_mapping", true);
  this->declare_parameter("use_autocross_for_accel", true);

  update_rate_ = this->get_parameter("update_rate").as_double();
  event_ = this->get_parameter("event").as_string();
  small_radius_ = this->get_parameter("small_radius").as_double();
  big_radius_ = this->get_parameter("big_radius").as_double();
  threshold_radius_ = this->get_parameter("threshold_radius").as_double();
  threshold_cones_ = this->get_parameter("threshold_cones").as_int();
  max_velocity_ = this->get_parameter("max_velocity").as_double();
  use_mapping_ = this->get_parameter("use_mapping").as_bool();
  use_autocross_for_accel_ = this->get_parameter("use_autocross_for_accel").as_bool();

  waypoints = this->getWaypoints("src/planning/global_waypoints/Waypoints.csv");
  global_path_ = this->get_parameter("global_path").as_int();
  colourblind_ = this->get_parameter("colourblind").as_int();
  lookahead_scaling_factor_ =
      this->get_parameter("lookahead_scaling_factor").as_double();
  base_lookahead_distance_ =
      this->get_parameter("base_lookahead_distance").as_double();

  RCLCPP_INFO(this->get_logger(), "Event: %s", event_.c_str());

  last_time = this->get_clock()->now();
}

void CenterPathNode::initSubscribers() {
  ego_state_subscriber_ = this->create_subscription<utfr_msgs::msg::EgoState>(
      topics::kEgoState, 10, std::bind(&CenterPathNode::egoStateCB, this, _1));

  cone_map_subscriber_ = this->create_subscription<utfr_msgs::msg::ConeMap>(
      topics::kConeMap, 10, std::bind(&CenterPathNode::coneMapCB, this, _1));

  cone_detection_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ConeDetections>(
          topics::kConeDetections, 10,
          std::bind(&CenterPathNode::coneDetectionsCB, this, _1));

  cone_map_closure_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      topics::kLoopClosed, 10,
      std::bind(&CenterPathNode::coneMapClosureCB, this, _1));

  if(!use_mapping_){
    vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/filter/velocity",
      10,
      std::bind(&CenterPathNode::velocityCB, this, _1));
  }
}

void CenterPathNode::initPublishers() {
  center_path_publisher_ =
      this->create_publisher<utfr_msgs::msg::ParametricSpline>(
          topics::kCenterPath, 10);

  center_point_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
      topics::kSkidpadCenterPoint, 10);

  accel_path_publisher_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>(
          topics::kAccelPath, 10);

  delauny_midpoint_path_publisher_ = 
      this->create_publisher<visualization_msgs::msg::Marker>(
          topics::kDelaunayMidpoints, 10);
  
  delaunay_path_publisher_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>(
          topics::kDelaunayWaypoints, 10);

  first_midpoint_path_publisher_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>(
          topics::kDelaunayWaypoints, 11);

  skidpad_path_publisher_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>(
          topics::kSkidpadFitting, 10);

  skidpad_path_publisher_2_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>(
          topics::kSkidpadFitting2, 10);

  skidpad_path_publisher_avg_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>(
          topics::kSkidpadFittingavg, 10);

  lap_time_publisher_ = 
      this->create_publisher<utfr_msgs::msg::LapTime>(
          topics::kLapTime, 10);

  lap_datum_publisher_ = 
      this->create_publisher<visualization_msgs::msg::Marker>(
          topics::kLapDatum, 10);
}

void CenterPathNode::initTransforms() {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void CenterPathNode::initEvent() {
  if (event_ == "read") {
    mission_subscriber_ =
        this->create_subscription<utfr_msgs::msg::SystemStatus>(
            topics::kSystemStatus, 10,
            std::bind(&CenterPathNode::missionCB, this, std::placeholders::_1));
  }
  else{ // for sim
    as_state = utfr_msgs::msg::SystemStatus::AS_STATE_DRIVING;
    this->initTimers();
    this->initSector();
  }
}

void CenterPathNode::velocityCB(const geometry_msgs::msg::Vector3Stamped &msg){
    if (ego_state_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::EgoState template_ego;
    ego_state_ = std::make_shared<utfr_msgs::msg::EgoState>(template_ego);
  }
  ego_state_->header = msg.header;
  ego_state_->pose.pose.position.x = 0.0;
  ego_state_->pose.pose.position.y = 0.0;
  ego_state_->pose.pose.position.z = 0.0;
  ego_state_->pose.pose.orientation = util::yawToQuaternion(0.0);
  ego_state_->vel.twist.linear = msg.vector;
  ego_state_->steering_angle = 0;
}

void CenterPathNode::missionCB(const utfr_msgs::msg::SystemStatus &msg) {
  if(event_ == "read"){
    switch (msg.ami_state) {
      case utfr_msgs::msg::SystemStatus::AMI_STATE_ACCELERATION: {
        event_ = "accel";
        break;
      }
      case utfr_msgs::msg::SystemStatus::AMI_STATE_SKIDPAD: {
        event_ = "skidpad";
        break;
      }
      case utfr_msgs::msg::SystemStatus::AMI_STATE_TRACKDRIVE: {
        event_ = "trackdrive";
        break;
      }
      case utfr_msgs::msg::SystemStatus::AMI_STATE_EBSTEST: {
        event_ = "EBSTest";
        break;
      }
      case utfr_msgs::msg::SystemStatus::AMI_STATE_TESTING:
      case utfr_msgs::msg::SystemStatus::AMI_STATE_INSPECTION: {
        event_ = "ASTest";
        break;
      }
      case utfr_msgs::msg::SystemStatus::AMI_STATE_AUTOCROSS: {
        event_ = "autocross";
        break;
      }
    }
    if(event_ != "read"){
      this->initTimers();
      this->initSector();
    }
  }

  if (as_state == utfr_msgs::msg::SystemStatus::AS_STATE_READY && msg.as_state == utfr_msgs::msg::SystemStatus::AS_STATE_DRIVING) {
    last_time = this->get_clock()->now();
  }

  as_state = msg.as_state;
}

void CenterPathNode::initTimers() {
  if (event_ == "accel") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&CenterPathNode::timerCBAccel, this));
  } else if (event_ == "skidpad") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&CenterPathNode::timerCBSkidpad, this));
  } else if (event_ == "autocross") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&CenterPathNode::timerCBAutocross, this));
  } else if (event_ == "trackdrive") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&CenterPathNode::timerCBTrackdrive, this));
  } else if (event_ == "EBSTest") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&CenterPathNode::timerCBEBS, this));
  } else if (event_ == "ASTest") {
    main_timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(update_rate_),
        std::bind(&CenterPathNode::timerCBAS, this));
  }
}

void CenterPathNode::initSector() {
  if (event_ == "accel") {
    curr_sector_ = EventState::ACCEL_STRAIGHT;
  } else if (event_ == "skidpad") {
    curr_sector_ = EventState::SMALL_ORANGE_STRAIGHT;
  } else if (event_ == "autocross") {
    curr_sector_ = EventState::AUTOCROSS_LAP_1;
  } else if (event_ == "trackdrive") {
    curr_sector_ = EventState::TRACKDRIVE_LAP_1;
  } else if (event_ == "EBSTest"){
    curr_sector_ = EventState::ACCEL_STRAIGHT;
  } else {
    curr_sector_ = EventState::ERROR;
  }
}

void CenterPathNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kCenterPathHeartbeat, 10);
  heartbeat_.module.data = "planning_cp";
  heartbeat_.update_rate = update_rate_;
}

void CenterPathNode::publishHeartbeat(const int status) {
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_.lap_count = curr_sector_;
  heartbeat_publisher_->publish(heartbeat_);
}

void CenterPathNode::egoStateCB(const utfr_msgs::msg::EgoState &msg) {
  if (ego_state_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::EgoState template_ego;
    ego_state_ = std::make_shared<utfr_msgs::msg::EgoState>(template_ego);
  } else {
    total_distance_traveled_ +=
        msg.vel.twist.linear.x *
        ((((double)msg.header.stamp.sec) + msg.header.stamp.nanosec * 1e-9) -
         (((double)ego_state_->header.stamp.sec) +
          ego_state_->header.stamp.nanosec * 1e-9));

    // RCLCPP_INFO(this->get_logger(), "Total Distance Traveled: %f",
    // total_distance_traveled_);
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
    ego_state_->pose.pose.orientation = util::yawToQuaternion(0.0);
    ego_state_->vel = msg.vel;
    ego_state_->accel = msg.accel;
    ego_state_->steering_angle = msg.steering_angle;
  }

  if (event_ == "skidpad" && global_path_) {
    // add points to the visited array for global skidpad path finding
    double curX = ego_state_->pose.pose.position.x;
    double curY = ego_state_->pose.pose.position.y;
    double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);
    if (visited.empty()) {
      visited.push_back({curX, curY, yaw});
    } else {
      auto [lastX, lastY, _] = visited.back();
      if (util::euclidianDistance2D(lastX, curX, lastY, curY) >= 5) {
        visited.push_back({curX, curY, yaw});
      }
    }
  }
}

void CenterPathNode::coneMapCB(const utfr_msgs::msg::ConeMap &msg) {
  if(!use_mapping_) return;
  if (cone_map_local_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::ConeMap template_cone_map;
    cone_map_local_ =
        std::make_shared<utfr_msgs::msg::ConeMap>(template_cone_map);
  }
  if (cone_map_global_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::ConeMap template_cone_map;
    cone_map_global_ =
        std::make_shared<utfr_msgs::msg::ConeMap>(template_cone_map);
  }
  cone_map_global_->header = msg.header;
  cone_map_global_->left_cones = msg.left_cones;
  cone_map_global_->right_cones = msg.right_cones;
  cone_map_global_->large_orange_cones = msg.large_orange_cones;
  cone_map_global_->small_orange_cones = msg.small_orange_cones;

  cone_map_local_->header = msg.header;
  cone_map_local_->left_cones = getConesInHemisphere(msg.left_cones, 15.0);
  cone_map_local_->right_cones = getConesInHemisphere(msg.right_cones, 15.0);
  cone_map_local_->large_orange_cones =
      getConesInHemisphere(msg.large_orange_cones, 15.0);
  cone_map_local_->small_orange_cones =
      getConesInHemisphere(msg.small_orange_cones, 15.0);
  
  if (event_ == "skidpad") {
    this->createTransform(); // generate transform for global path
    // this->GlobalWaypoints(); // visualize the transformed points in map
    // view
  }
}

void CenterPathNode::coneDetectionsCB(
    const utfr_msgs::msg::ConeDetections &msg) {
  if (cone_detections_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::ConeDetections template_cone_detections;
    cone_detections_ = std::make_shared<utfr_msgs::msg::ConeDetections>(
        template_cone_detections);
  }

  cone_detections_->header = msg.header;
  cone_detections_->left_cones = msg.left_cones;
  cone_detections_->right_cones = msg.right_cones;
  cone_detections_->large_orange_cones = msg.large_orange_cones;
  cone_detections_->small_orange_cones = msg.small_orange_cones;

  if (!use_mapping_) {
    if (cone_map_local_ == nullptr) {
      // first initialization:
      utfr_msgs::msg::ConeMap template_cone_map;
      cone_map_local_ = std::make_shared<utfr_msgs::msg::ConeMap>(template_cone_map);
    }
    cone_map_local_->header = msg.header;
    cone_map_local_->left_cones = cone_detections_->left_cones;
    cone_map_local_->right_cones = cone_detections_->right_cones;
    cone_map_local_->large_orange_cones = cone_detections_->large_orange_cones;
    cone_map_local_->small_orange_cones = cone_detections_->small_orange_cones;
    if (cone_map_global_ == nullptr) {
      // first initialization:
      utfr_msgs::msg::ConeMap template_cone_map;
      cone_map_global_ = std::make_shared<utfr_msgs::msg::ConeMap>(template_cone_map);
    }
    cone_map_global_->header = msg.header;
    cone_map_global_->left_cones = cone_detections_->left_cones;
    cone_map_global_->right_cones = cone_detections_->right_cones;
    cone_map_global_->large_orange_cones = cone_detections_->large_orange_cones;
    cone_map_global_->small_orange_cones = cone_detections_->small_orange_cones;
  }
}

void CenterPathNode::coneMapClosureCB(const std_msgs::msg::Bool &msg) {
  // RCLCPP_WARN(this->get_logger(), "Cone Map Closure Callback");
  loop_closed_ = msg.data;
  // RCLCPP_INFO(this->get_logger(), "Loop Closed: %d", loop_closed_);
}

bool CenterPathNode::coneDistComparitor(const utfr_msgs::msg::Cone &a,
                                        const utfr_msgs::msg::Cone &b) {
  double dist_a = util::euclidianDistance2D(a.pos.x, 0.0, a.pos.y, 0.0);
  double dist_b = util::euclidianDistance2D(b.pos.x, 0.0, b.pos.y, 0.0);

  return dist_a < dist_b;
}

bool CenterPathNode::checkPassedDatum(const utfr_msgs::msg::EgoState reference,
                                      const utfr_msgs::msg::EgoState &current) {
  double ref_x = reference.pose.pose.position.x;
  double ref_y = reference.pose.pose.position.y;
  double ref_yaw = util::quaternionToYaw(reference.pose.pose.orientation);

  double cur_x = current.pose.pose.position.x +
                 2.0; // offset forward to represent nose of car
  double cur_y = current.pose.pose.position.y;
  double cur_yaw = util::quaternionToYaw(current.pose.pose.orientation);

  // for testing
  ref_yaw = cur_yaw;

  double dx = ref_x - cur_x;
  double dy = ref_y - cur_y;

  double tdist = sqrt(dx * dx + dy * dy);

  double dx_local = dx * cos(-ref_yaw) - dy * sin(-ref_yaw);
  double dy_local = dx * sin(-ref_yaw) + dy * cos(-ref_yaw);
  RCLCPP_INFO(this->get_logger(), "Y distance: %f", abs(dy_local));

  if (abs(ref_yaw - cur_yaw) < 3.1415 / 2 && abs(dy_local) < 2.5 &&
      dx_local < 0.0 && datum_last_local_x_ >= 0.0) {
    //if alignment within 90 deg, distance less than 2.5m
    datum_last_local_x_ = dx_local;
    return true;
  }
  datum_last_local_x_ = dx_local;
  return false;
}

void CenterPathNode::publishLine(double m_left, double m_right, double c_left,
                                 double c_right, double x_min, double x_max) {

  utfr_msgs::msg::ParametricSpline avg_line_msg;

  geometry_msgs::msg::PolygonStamped line1_stamped, line2_stamped, lineavg;

  line1_stamped.header.frame_id = "ground";
  line2_stamped.header.frame_id = "ground";
  lineavg.header.frame_id = "ground";

  line1_stamped.header.stamp = this->get_clock()->now();
  line2_stamped.header.stamp = this->get_clock()->now();
  lineavg.header.stamp = this->get_clock()->now();

  geometry_msgs::msg::Point32 point;
  point.x = x_min;
  point.y = -(x_min * m_left + c_left);
  point.z = 0.0;
  line1_stamped.polygon.points.push_back(point);

  point.x = x_max;
  point.y = -(x_max * m_left + c_left);
  point.z = 0.0;
  line1_stamped.polygon.points.push_back(point);

  point.x = x_min;
  point.y = -(x_min * m_right + c_right);
  point.z = 0.0;
  line2_stamped.polygon.points.push_back(point);

  point.x = x_max;
  point.y = -(x_max * m_right + c_right);
  point.z = 0.0;
  line2_stamped.polygon.points.push_back(point);

  point.x = x_min;
  point.y = -(x_min * ((m_left + m_right) / 2.0) + ((c_left + c_right) / 2.0));
  point.z = 0.0;
  lineavg.polygon.points.push_back(point);

  point.x = x_max;
  point.y = -(x_max * ((m_left + m_right) / 2.0) + ((c_left + c_right) / 2.0));
  point.z = 0.0;
  lineavg.polygon.points.push_back(point);

  skidpad_path_publisher_->publish(line1_stamped);
  skidpad_path_publisher_2_->publish(line2_stamped);
  skidpad_path_publisher_avg_->publish(lineavg);

  avg_line_msg.header.frame_id = "ground";
  avg_line_msg.header.stamp = this->get_clock()->now();
  avg_line_msg.x_params = {0, 0, 0, 0, 1, 0};
  avg_line_msg.y_params = {
      0, 0, 0, 0, (m_left + m_right) / 2.0, (c_left + c_right) / 2.0};
  avg_line_msg.skidpad_params = {0, 0, 0};
  avg_line_msg.lap_count = curr_sector_;

  center_path_publisher_->publish(avg_line_msg);
}

void CenterPathNode::publishLapTime() {
  utfr_msgs::msg::LapTime lap_time_msg;
  lap_time_msg.header.stamp = this->get_clock()->now();
  if (!last_sector) {
    return;
  }
  rclcpp::Time curr_time = this->get_clock()->now();
  float curr_lap_time = (curr_time - last_switch_time).seconds();

  if (curr_sector_ != last_sector) {
    last_lap_time = curr_time.seconds();
    last_switch_time = curr_time;
    if (curr_time.seconds() < best_lap_time) {
      best_lap_time = curr_time.seconds();
    }
  }

  lap_time_msg.best_time = best_lap_time;
  lap_time_msg.last_time = last_lap_time;
  lap_time_msg.curr_time = curr_time.seconds();

  lap_time_publisher_->publish(lap_time_msg);

  last_sector = curr_sector_;
}

std::vector<double> CenterPathNode::globalToLocal(double x, double y) {
  double translated_x = x - ego_state_->pose.pose.position.x;
  double translated_y = y - ego_state_->pose.pose.position.y;
  double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);
  double local_x = translated_x * cos(-yaw) - translated_y * sin(-yaw);
  double local_y = translated_x * sin(-yaw) + translated_y * cos(-yaw);
  return {local_x, local_y};
}

bool CenterPathNode::hempishere(double x, double y, double r) {
  std::vector<double> local_coord = globalToLocal(x, y);
  double local_x = local_coord[0];
  double local_y = local_coord[1];
  if (local_x * local_x + local_y * local_y < r * r && local_x > 0) {
    return true;
  } else
    return false;
}

std::vector<utfr_msgs::msg::Cone>
CenterPathNode::getConesInHemisphere(std::vector<utfr_msgs::msg::Cone> cones,
                                     double r) {
  std::vector<utfr_msgs::msg::Cone> cones_in_hemisphere;
  for (int i = 0; i < static_cast<int>(cones.size()); i++) {
    if (hempishere(cones[i].pos.x, cones[i].pos.y, r)) {
      utfr_msgs::msg::Cone cone;
      std::vector<double> local_coord =
          globalToLocal(cones[i].pos.x, cones[i].pos.y);
      cone.pos.x = local_coord[0];
      cone.pos.y = local_coord[1];
      cone.type = cones[i].type;
      cones_in_hemisphere.push_back(cone);
    }
  }
  return cones_in_hemisphere;
}

} // namespace center_path
} // namespace utfr_dv
