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
using Point32 = geometry_msgs::msg::Point32;
using PointTuple = std::tuple<Point32, Point32, Point32, Point32>;

CenterPathNode::CenterPathNode() : Node("center_path_node") {
  // RCLCPP_INFO(this->get_logger(), "Center Path Node Launched");
  this->initParams();
  this->initEvent();
  this->initHeartbeat();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::NOT_READY);
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initSector();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
}

void CenterPathNode::initParams() {
  this->declare_parameter("update_rate", 33.33);
  this->declare_parameter("event", "read");
  this->declare_parameter("big_radius", 10.625);
  this->declare_parameter("small_radius", 7.625);
  this->declare_parameter("threshold_radius", 0.8);
  this->declare_parameter("threshold_cones", 3);

  update_rate_ = this->get_parameter("update_rate").as_double();
  event_ = this->get_parameter("event").as_string();
  small_radius_ = this->get_parameter("small_radius").as_double();
  big_radius_ = this->get_parameter("big_radius").as_double();
  threshold_radius_ = this->get_parameter("threshold_radius").as_double();
  threshold_cones_ = this->get_parameter("threshold_cones").as_int();

  RCLCPP_INFO(this->get_logger(), "Event: %s", event_.c_str());
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

  cone_map_closure_subscriber_ =
      this->create_subscription<std_msgs::msg::Bool>(
          topics::kLoopClosed, 10,
          std::bind(&CenterPathNode::coneMapClosureCB, this, _1));
}

void CenterPathNode::initPublishers() {
  center_path_publisher_ =
      this->create_publisher<utfr_msgs::msg::ParametricSpline>(
          topics::kCenterPath, 10);

  accel_path_publisher_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>(
          topics::kAccelPath, 10);

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

  lap_time_publisher_ = this->create_publisher<utfr_msgs::msg::LapTime>(
      topics::kLapTime, 10);

  lap_datum_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/lap_datum", 10);
}

void CenterPathNode::initEvent() {
  if (event_ == "read") {
    mission_subscriber_ =
        this->create_subscription<utfr_msgs::msg::SystemStatus>(
            topics::kSystemStatus, 10,
            std::bind(&CenterPathNode::missionCB, this, std::placeholders::_1));
  }
}

void CenterPathNode::missionCB(const utfr_msgs::msg::SystemStatus &msg) {
  if (msg.ami_state == 1) {
    event_ = "accel";
    mission_subscriber_.reset();
  } else if (msg.ami_state == 2) {
    event_ = "skidpad";
    mission_subscriber_.reset();
  } else if (msg.ami_state == 3) {
    event_ = "trackdrive";
    mission_subscriber_.reset();
  } else if (msg.ami_state == 4) {
    event_ = "EBSTest";
    mission_subscriber_.reset();
  } else if (msg.ami_state == 5) {
    event_ = "ASTest";
    mission_subscriber_.reset();
  } else if (msg.ami_state == 6) {
    event_ = "autocross";
    mission_subscriber_.reset();
  }
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
    curr_sector_ = 1;
    use_mapping_ = true;
  } else if (event_ == "skidpad") {
    curr_sector_ = 10;
    use_mapping_ = true;
  } else if (event_ == "autocross") {
    curr_sector_ = 20;
    use_mapping_ = true;
  } else if (event_ == "trackdrive") {
    curr_sector_ = 30;
    use_mapping_ = true;
  } else {
    curr_sector_ = 0;
  }
  last_time = this->get_clock()->now();
  last_switch_time = this->get_clock()->now();
  lock_sector_ = true;
  found_4_large_orange = false;
}

void CenterPathNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kCenterPathHeartbeat, 10);
  heartbeat_.module.data = "center_path_node";
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
    total_distance_traveled_ += msg.vel.twist.linear.x * 
        ((((double) msg.header.stamp.sec) + msg.header.stamp.nanosec * 1e-9) - 
        (((double)ego_state_->header.stamp.sec) + ego_state_->header.stamp.nanosec * 1e-9));
    
    // RCLCPP_INFO(this->get_logger(), "Total Distance Traveled: %f", total_distance_traveled_);
  }
  if (use_mapping_){
    ego_state_->header = msg.header;
    ego_state_->pose = msg.pose;
    ego_state_->vel = msg.vel;
    ego_state_->accel = msg.accel;
    ego_state_->steering_angle = msg.steering_angle;
  }
  else if (!use_mapping_){
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

void CenterPathNode::coneMapCB(const utfr_msgs::msg::ConeMap &msg) {
  if (cone_map_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::ConeMap template_cone_map;
    cone_map_ = std::make_shared<utfr_msgs::msg::ConeMap>(template_cone_map);
  }
  if (cone_map_raw_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::ConeMap template_cone_map;
    cone_map_raw_ = std::make_shared<utfr_msgs::msg::ConeMap>(template_cone_map);
  }
  if (use_mapping_ && (event_ == "autocross" || event_ == "trackdrive")) {
    cone_map_->header = msg.header;
    cone_map_->left_cones = msg.left_cones;
    cone_map_->right_cones = msg.right_cones;
    cone_map_->large_orange_cones = msg.large_orange_cones;
    cone_map_->small_orange_cones = msg.small_orange_cones;
    (*cone_map_raw_) = msg;
  }
  else if (use_mapping_ && (event_ == "skidpad" || event_ == "accel")){
    cone_map_->header = msg.header;
    cone_map_->left_cones = getConesInHemisphere(msg.left_cones, 15.0);
    cone_map_->right_cones = getConesInHemisphere(msg.right_cones, 15.0);
    cone_map_->large_orange_cones = getConesInHemisphere(msg.large_orange_cones, 15.0);
    cone_map_->small_orange_cones = getConesInHemisphere(msg.small_orange_cones, 15.0);
    (*cone_map_raw_) = msg;
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

  if (!use_mapping_ && cone_map_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::ConeMap template_cone_map;
    cone_map_ = std::make_shared<utfr_msgs::msg::ConeMap>(template_cone_map);
  }

  cone_detections_->header = msg.header;
  cone_detections_->left_cones = msg.left_cones;
  cone_detections_->right_cones = msg.right_cones;
  cone_detections_->large_orange_cones = msg.large_orange_cones;
  cone_detections_->small_orange_cones = msg.small_orange_cones;

  if (!use_mapping_){
    cone_map_->header = msg.header;
    cone_map_->left_cones = cone_detections_->left_cones;
    cone_map_->right_cones = cone_detections_->right_cones;
    cone_map_->large_orange_cones = cone_detections_->large_orange_cones;
    cone_map_->small_orange_cones = cone_detections_->small_orange_cones;
  }
}

void CenterPathNode::coneMapClosureCB(const std_msgs::msg::Bool &msg) {
  // RCLCPP_WARN(this->get_logger(), "Cone Map Closure Callback");
  loop_closed_ = msg.data;
}

void CenterPathNode::timerCBAccel() {
  try {
    const std::string function_name{"center_path_timerCB:"};

    if (!cone_detections_) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                  "Data not published or initialized yet. Using defaults.");
      return;
    }
    if (cone_map_ == nullptr) {
      RCLCPP_WARN(get_logger(), "%s Cone Map is empty", function_name.c_str());
      return;
    }

    std::vector<double> accel_path = getAccelPath();

    utfr_msgs::msg::ParametricSpline center_path_msg;

    double m = accel_path[0];
    double c = accel_path[1];

    std::vector<double> x = {0, 0, 0, 0, 1, 0};
    std::vector<double> y = {0, 0, 0, 0, m, c};

    center_path_msg.x_params = x;
    center_path_msg.y_params = y;
    center_path_msg.lap_count = curr_sector_;
    center_path_publisher_->publish(center_path_msg);

    int left_size = cone_detections_->left_cones.size();
    int right_size = cone_detections_->right_cones.size();
    int large_orange_size = cone_detections_->large_orange_cones.size();

    if (!accel_sector_increase && left_size == 0 && right_size == 0 &&
        large_orange_size == 0) {
      accel_sector_increase = true;
      curr_sector_ += 1;
      RCLCPP_INFO(this->get_logger(), "Accel ended due to cone detections.");
    }

    if (!accel_sector_increase && total_distance_traveled_ > 80) { // accel length 75 m
      accel_sector_increase = true;
      curr_sector_ += 1;
      RCLCPP_INFO(this->get_logger(), "Accel ended due to distance traveled.");
    }

    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
    publishLapTime();
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void CenterPathNode::timerCBSkidpad() {
  const std::string function_name{"center_path_timerCB:"};
  try {
    if (cone_detections_ == nullptr || ego_state_ == nullptr) {
      RCLCPP_WARN(get_logger(),
                  "%s Either cone detections or ego state is empty.",
                  function_name.c_str());
      return;
    }
    if (cone_map_ == nullptr) {
      RCLCPP_WARN(get_logger(), "%s Cone Map is empty", function_name.c_str());
      return;
    }
    skidPadFit();
    skidpadLapCounter();
    publishLapTime();
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void CenterPathNode::timerCBAutocross() {
  const std::string function_name{"center_path_timerCB:"};

  try {
    if (cone_detections_ == nullptr || cone_map_ == nullptr ||
        ego_state_ == nullptr) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                  "Data not published or initialized yet. Using defaults.");
      return;
    }
    if (cone_map_ == nullptr) {
      RCLCPP_WARN(get_logger(), "%s Cone Map is empty", function_name.c_str());
      return;
    }

    std::vector<Point> midpoints = getBestPath();
    std::tuple<std::vector<Point>, std::vector<double>, std::vector<double>>
        result = BezierPoints(midpoints);
    std::vector<Point> bezier_curve = std::get<0>(result);
    std::vector<double> xoft = std::get<1>(result);
    std::vector<double> yoft = std::get<2>(result);

    // Bezier Curve Drawing
    geometry_msgs::msg::PolygonStamped delaunay_midpoints_stamped;
    geometry_msgs::msg::PolygonStamped first_midpoint_stamped;

    delaunay_midpoints_stamped.header.frame_id = "base_footprint";
    delaunay_midpoints_stamped.header.stamp = this->get_clock()->now();
    first_midpoint_stamped.header.frame_id = "base_footprint";
    first_midpoint_stamped.header.stamp = this->get_clock()->now();
    geometry_msgs::msg::Point32 midpoint_pt32;

    for (int i = 0; i < static_cast<int>(bezier_curve.size()); i++) {
      midpoint_pt32.x = bezier_curve[i].x();
      midpoint_pt32.y = bezier_curve[i].y() * -1;
      midpoint_pt32.z = 0.0;

      delaunay_midpoints_stamped.polygon.points.push_back(midpoint_pt32);
    }

    for (int i = static_cast<int>(bezier_curve.size()) - 1; i > 0; i--) {
      midpoint_pt32.x = bezier_curve[i].x();
      midpoint_pt32.y = bezier_curve[i].y() * -1;
      midpoint_pt32.z = 0.0;

      delaunay_midpoints_stamped.polygon.points.push_back(midpoint_pt32);
    }

    delaunay_path_publisher_->publish(delaunay_midpoints_stamped);

    utfr_msgs::msg::ParametricSpline center_path;
    center_path.header.stamp = this->get_clock()->now();
    center_path.x_params = xoft;
    center_path.y_params = yoft;
    center_path.lap_count = curr_sector_;
    center_path_publisher_->publish(center_path);

    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);

    trackdriveLapCounter();
    publishLapTime();
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void CenterPathNode::timerCBTrackdrive() {
  const std::string function_name{"center_path_timerCB:"};

  try {
    if (!cone_detections_ || cone_map_ == nullptr || ego_state_ == nullptr) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                  "Data not published or initialized yet. Using defaults.");
      return;
    }
    if (cone_map_ == nullptr) {
      RCLCPP_WARN(get_logger(), "%s Cone Map is empty", function_name.c_str());
      return;
    }

    std::vector<Point> midpoints = getBestPath();
    std::tuple<std::vector<Point>, std::vector<double>, std::vector<double>>
        result = BezierPoints(midpoints);
    std::vector<Point> bezier_curve = std::get<0>(result);
    std::vector<double> xoft = std::get<1>(result);
    std::vector<double> yoft = std::get<2>(result);

    // Bezier Curve Drawing
    geometry_msgs::msg::PolygonStamped delaunay_midpoints_stamped;
    geometry_msgs::msg::PolygonStamped first_midpoint_stamped;

    delaunay_midpoints_stamped.header.frame_id = "base_footprint";
    delaunay_midpoints_stamped.header.stamp = this->get_clock()->now();
    first_midpoint_stamped.header.frame_id = "base_footprint";
    first_midpoint_stamped.header.stamp = this->get_clock()->now();
    geometry_msgs::msg::Point32 midpoint_pt32;

    for (int i = 0; i < static_cast<int>(bezier_curve.size()); i++) {
      midpoint_pt32.x = bezier_curve[i].x();
      midpoint_pt32.y = bezier_curve[i].y() * -1;
      midpoint_pt32.z = 0.0;

      delaunay_midpoints_stamped.polygon.points.push_back(midpoint_pt32);
    }

    for (int i = static_cast<int>(bezier_curve.size()) - 1; i > 0; i--) {
      midpoint_pt32.x = bezier_curve[i].x();
      midpoint_pt32.y = bezier_curve[i].y() * -1;
      midpoint_pt32.z = 0.0;

      delaunay_midpoints_stamped.polygon.points.push_back(midpoint_pt32);
    }

    delaunay_path_publisher_->publish(delaunay_midpoints_stamped);

    utfr_msgs::msg::ParametricSpline center_path;
    center_path.header.stamp = this->get_clock()->now();
    center_path.x_params = xoft;
    center_path.y_params = yoft;
    center_path.lap_count = curr_sector_;
    center_path_publisher_->publish(center_path);

    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);

    trackdriveLapCounter();
    publishLapTime();
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void CenterPathNode::timerCBEBS() {
  try {
    const std::string function_name{"center_path_timerCB:"};

    if (!cone_detections_) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                  "Data not published or initialized yet. Using defaults.");
      return;
    }

    std::vector<double> accel_path = getAccelPath();

    utfr_msgs::msg::ParametricSpline center_path_msg;

    double m = accel_path[0];
    double c = accel_path[1];

    std::vector<double> x = {0, 0, 0, 0, 1, 0};
    std::vector<double> y = {0, 0, 0, 0, m, c};

    center_path_msg.x_params = x;
    center_path_msg.y_params = y;
    center_path_msg.lap_count = curr_sector_;

    center_path_publisher_->publish(center_path_msg);

    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void CenterPathNode::timerCBAS() {
  publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
}

bool CenterPathNode::coneDistComparitor(const utfr_msgs::msg::Cone &a,
                                        const utfr_msgs::msg::Cone &b) {
  double dist_a = util::euclidianDistance2D(a.pos.x, 0.0, a.pos.y, 0.0);
  double dist_b = util::euclidianDistance2D(b.pos.x, 0.0, b.pos.y, 0.0);

  return dist_a < dist_b;
}

std::vector<double> CenterPathNode::getAccelPath() {
  std::vector<utfr_msgs::msg::Cone> all_cones;

  if (cone_map_ == nullptr) {
    RCLCPP_WARN(this->get_logger(), "Cone Map is empty");
    return std::vector<double>();
  }
  
  if (curr_sector_ < 5) {
    all_cones.insert(all_cones.end(), cone_map_->left_cones.begin(),
                    cone_map_->left_cones.end());
    all_cones.insert(all_cones.end(), cone_map_->right_cones.begin(),
                    cone_map_->right_cones.end());
  }
  all_cones.insert(all_cones.end(),
                   cone_map_->large_orange_cones.begin(),
                   cone_map_->large_orange_cones.end());
  all_cones.insert(all_cones.end(),
                   cone_map_->small_orange_cones.begin(),
                   cone_map_->small_orange_cones.end());

  std::sort(
      all_cones.begin(), all_cones.end(),
      [this](const utfr_msgs::msg::Cone &a, const utfr_msgs::msg::Cone &b) {
        return this->coneDistComparitor(a, b);
      });

  bool found_1 = false;

  int ind_1, ind_2;
  double highest_1 = 0.0, highest_2 = 0.0;
  double best_m_1, best_m_2, best_c_1, best_c_2;

  for (int i = 0; i < static_cast<int>(all_cones.size()) - 1; i++) {
    for (int j = i + 1; j < static_cast<int>(all_cones.size()); j++) {
      if (i != j) {
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

        if (abs(m) < 1.0) {
          int count = 0;
          int cone_type;
          if (all_cones[i].type == all_cones[j].type) {
            count += 1;
            cone_type = all_cones[i].type;
          }
          for (int k = 0; k < static_cast<int>(all_cones.size()); k++) {
            double numerator =
                std::abs(m * all_cones[k].pos.x - all_cones[k].pos.y + c);
            double denominator = std::sqrt(m * m + 1);
            double dist = numerator / denominator;
            if (dist < 0.5) {
              count += 1;
              if (all_cones[k].type == cone_type) {
                count += 1;
              }
            }
          }

          if (highest_1 <= count) {
            highest_1 = count;
            best_m_1 = m;
            best_c_1 = c;
          }
          ind_1 = i;
          ind_2 = j;
          found_1 = true;
        }
      }
    }
  }

  bool found_2 = false;
  for (int i = 0; i < static_cast<int>(all_cones.size()) - 1; i++) {
    for (int j = i + 1; j < static_cast<int>(all_cones.size()); j++) {
      if (i != j && found_1 && i != ind_1 && j != ind_1 && i != ind_2 && j != ind_2) {
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

        if (abs(m - best_m_1) < 0.2 && abs(c - best_c_1) > 2.5) {
          int count = 0;
          int cone_type;
          if (all_cones[i].type == all_cones[j].type) {
            count += 1;
            cone_type = all_cones[i].type;
          }
          for (int k = 0; k < static_cast<int>(all_cones.size()); k++) {
            double numerator =
                std::abs(m * all_cones[k].pos.x - all_cones[k].pos.y + c);
            double denominator = std::sqrt(m * m + 1);
            double dist = numerator / denominator;
            if (dist < 0.5) {
              count += 1;
              if (all_cones[k].type == cone_type) {
                count += 0.2;
              }
            }
          }
          if (highest_2 <= count) {
            highest_2 = count;
            best_m_2 = m;
            best_c_2 = c;
          }
          found_2 = true;
        }
      }
    }
  }

  double final_m, final_c;

  if (found_1 == true && found_2 == true) {
    final_m = (best_m_1 + best_m_2) / 2;
    final_c = (best_c_1 + best_c_2) / 2;
  } else {
    if (found_1 == true) {
      final_m = best_m_1;
      if (best_c_1 > 0)
        final_c = best_c_1 - 1.5;
      else
        final_c = best_c_1 + 1.5;
    } else if (found_2 == true) {
      final_m = best_m_2;
      if (best_c_2 > 0)
        final_c = best_c_2 - 1.5;
      else
        final_c = best_c_2 + 1.5;
    } else {
      final_m = 0;
      final_c = 0;
    }
  }

  if (curr_sector_ < 5) {
    geometry_msgs::msg::PolygonStamped accel_path_msg;

    accel_path_msg.header.stamp = this->get_clock()->now();
    accel_path_msg.header.frame_id = "base_footprint";
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

  std::vector<double> accel_path;
  accel_path.push_back(final_m);
  accel_path.push_back(final_c);
  return accel_path;
}

double getTurnAngle(double u1, double u2, double v1, double v2) {
  if (sqrt(pow(u1, 2) + pow(u2, 2)) == 0 ||
      sqrt(pow(v1, 2) + pow(v2, 2)) == 0) {
    return 0;
  }
  double cos_theta = (u1 * v1 + u2 * v2) / (sqrt(pow(u1, 2) + pow(u2, 2)) *
                                            sqrt(pow(v1, 2) + pow(v2, 2)));
  double theta = acos(cos_theta);
  return theta;
}

std::vector<std::vector<double>>
calculateDistances(const std::vector<CGAL::Point_2<CGAL::Epick>> &midpoints,
                   const std::vector<utfr_msgs::msg::Cone> cones,
                   const std::vector<int> nodes) {
  std::vector<std::vector<double>> distances;
  for (int node : nodes) {
    std::vector<double> nodeDistances;
    for (const auto &cone : cones) {
      nodeDistances.push_back(sqrt(pow(cone.pos.x - midpoints[node].x(), 2) +
                                   pow(cone.pos.y - midpoints[node].y(), 2)));
    }
    distances.push_back(nodeDistances);
  }
  return distances;
}

double findMaxMinDistance(const std::vector<std::vector<double>> &distances) {
  double maxMinDistance = std::numeric_limits<double>::lowest();
  for (const auto &distArr : distances) {
    if (!distArr.empty()) {
      double minDistance = *std::min_element(distArr.begin(), distArr.end());
      maxMinDistance = std::max(maxMinDistance, minDistance);
    }
  }
  return maxMinDistance;
}

double CenterPathNode::midpointCostFunction(
    std::vector<int> nodes,
    const std::vector<CGAL::Point_2<CGAL::Epick>> &midpoints,
    std::vector<std::pair<CGAL::Point_2<CGAL::Epick>, unsigned int>> all_cones,
    std::vector<utfr_msgs::msg::Cone> yellow_cones,
    std::vector<utfr_msgs::msg::Cone> blue_cones,
    std::vector<std::pair<int, int>> midpoint_index_to_cone_indices) {
  double A = 0.7;
  double B = 2.0;
  double C = 2.0;
  double D = 3.0;
  double E = 1.0;
  double F = 0.3;
  double Z = 0.1;

  double MAX_MAX_ANGLE = 3.3141592653589793;
  double MAX_MAX_MIDPOINT_TO_CONE_DIST = 25.0;
  double MAX_SQUARED_RANGE_PATH_LENGTH_DIFF =
      pow(MAX_MAX_MIDPOINT_TO_CONE_DIST, 2);
  int MAX_POINT_COUNT_COST = 10;
  double MAX_INTERPOLATED_MIDPOINT_TO_CONE_DISTANCE_COST = 1.2 / 2.0 * 4.0;
  double MAX_STD_DEV = 1.2;
  double LENGTH_COST = 0.2;

  double car_tip_x =
      ego_state_->pose.pose.position.x +
      1.2 / 2.0 * cos(util::quaternionToYaw(ego_state_->pose.pose.orientation));
  double car_tip_y =
      ego_state_->pose.pose.position.y +
      1.2 / 2.0 * sin(util::quaternionToYaw(ego_state_->pose.pose.orientation));
  double initial_angle = getTurnAngle(
      car_tip_x - ego_state_->pose.pose.position.x,
      car_tip_y - ego_state_->pose.pose.position.y,
      midpoints[nodes[0]].x() - car_tip_x, midpoints[nodes[0]].y() - car_tip_y);
  double next_angle = getTurnAngle(
      midpoints[nodes[0]].x() - car_tip_x, midpoints[nodes[0]].y() - car_tip_y,
      midpoints[nodes[1]].x() - midpoints[nodes[0]].x(),
      midpoints[nodes[1]].y() - midpoints[nodes[0]].y());

  std::vector<double> midpoint_to_midpoint_angles;

  double max_angle = -100000.0;

  for (int i = 1; i < static_cast<int>(nodes.size()) - 1; i++) {
    double angle =
        getTurnAngle(midpoints[nodes[i]].x() - midpoints[nodes[i - 1]].x(),
                     midpoints[nodes[i]].y() - midpoints[nodes[i - 1]].y(),
                     midpoints[nodes[i + 1]].x() - midpoints[nodes[i]].x(),
                     midpoints[nodes[i + 1]].y() - midpoints[nodes[i]].y());
    midpoint_to_midpoint_angles.push_back(angle);
    if (angle > max_angle)
      max_angle = angle;
  }

  max_angle = std::max(std::max(initial_angle, next_angle), max_angle);

  double path_length = 0;

  for (int i = 0; i < static_cast<int>(nodes.size()) - 1; i++) {
    path_length +=
        sqrt(pow(midpoints[nodes[i]].x() - midpoints[nodes[i + 1]].x(), 2) +
             pow(midpoints[nodes[i]].y() - midpoints[nodes[i + 1]].y(), 2));
  }

  double squared_range_path_length_diff =
      pow(MAX_MAX_MIDPOINT_TO_CONE_DIST - path_length, 2);

  std::vector<std::vector<double>> midpoint_to_yellow_cone_distances;
  std::vector<std::vector<double>> midpoint_to_blue_cone_distances;

  if (!yellow_cones.empty()) {
    midpoint_to_yellow_cone_distances =
        calculateDistances(midpoints, cone_map_->right_cones, nodes);
  } else {
    midpoint_to_yellow_cone_distances = {{0}};
  }

  if (!blue_cones.empty()) {
    midpoint_to_blue_cone_distances =
        calculateDistances(midpoints, cone_map_->left_cones, nodes);
  } else {
    midpoint_to_blue_cone_distances = {{0}};
  }

  double max_midpoint_to_yellow_distance =
      findMaxMinDistance(midpoint_to_yellow_cone_distances);
  double max_midpoint_to_blue_distance =
      findMaxMinDistance(midpoint_to_blue_cone_distances);

  std::vector<Point> interpolated_midpoints;

  for (int i = 1; i < static_cast<int>(nodes.size()); i++) {
    double x1 = midpoints[nodes[i - 1]].x();
    double y1 = midpoints[nodes[i - 1]].y();
    double x2 = midpoints[nodes[i]].x();
    double y2 = midpoints[nodes[i]].y();
    double dx = x2 - x1;
    double dy = y2 - y1;

    Point interpolated_midpoint = Point(dx / 2.0, dy / 2.0);

    interpolated_midpoints.push_back(interpolated_midpoint);
  }

  std::vector<utfr_msgs::msg::Cone> combined_cones;
  combined_cones.insert(combined_cones.end(), cone_map_->left_cones.begin(),
                        cone_map_->left_cones.end());
  combined_cones.insert(combined_cones.end(), cone_map_->right_cones.begin(),
                        cone_map_->right_cones.end());

  double min_interpolated_midpoint_to_cone_distance = -10000.0;
  std::vector<double> interpolated_midpoints_to_cone_min_distances;
  for (size_t i = 0; i < nodes.size() - 1; ++i) {
    Point interpolatedMidpoint =
        Point((midpoints[nodes[i]].x() + midpoints[nodes[i + 1]].x()) / 2.0,
              (midpoints[nodes[i]].y() + midpoints[nodes[i + 1]].y()) / 2.0);

    double minDistance = std::numeric_limits<double>::max();
    for (const auto &cone : combined_cones) {
      double distance =
          std::sqrt(std::pow(cone.pos.x - interpolatedMidpoint.x(), 2) +
                    std::pow(cone.pos.y - interpolatedMidpoint.y(), 2));
      minDistance = std::min(minDistance, distance);
    }

    interpolated_midpoints_to_cone_min_distances.push_back(minDistance);

    if (minDistance > min_interpolated_midpoint_to_cone_distance)
      min_interpolated_midpoint_to_cone_distance = minDistance;
  }
  
  double length = 0.0;

  std::vector<double> track_widths;
  for (int node : nodes) {
    Point cone1 = all_cones[midpoint_index_to_cone_indices[node].first].first;
    Point cone2 = all_cones[midpoint_index_to_cone_indices[node].second].first;

    double width = std::sqrt(std::pow(cone1.x() - cone2.x(), 2) +
                             std::pow(cone1.y() - cone2.y(), 2));
    track_widths.push_back(width);
    length += F;
  }

  double mean = std::accumulate(track_widths.begin(), track_widths.end(), 0.0) /
                track_widths.size();
  double sq_sum = std::inner_product(track_widths.begin(), track_widths.end(),
                                     track_widths.begin(), 0.0);
  double std_dev = std::sqrt(sq_sum / track_widths.size() - mean * mean);

  double sum =
      A * pow((max_angle / MAX_MAX_ANGLE), 2) +
      B * pow((squared_range_path_length_diff /
               MAX_SQUARED_RANGE_PATH_LENGTH_DIFF),
              2) +
      C * pow((std::max(max_midpoint_to_blue_distance,
                        max_midpoint_to_yellow_distance) /
               MAX_MAX_MIDPOINT_TO_CONE_DIST),
              2) +
      D * pow((abs(1.2 / 2.0 - min_interpolated_midpoint_to_cone_distance) /
               MAX_INTERPOLATED_MIDPOINT_TO_CONE_DISTANCE_COST),
              2) +
      E * pow((std_dev / MAX_STD_DEV), 2) +
      Z * pow((abs(10.0 - nodes.size()) / MAX_POINT_COUNT_COST), 2) - 
      length * LENGTH_COST;

  return sum;
}

std::vector<CGAL::Point_2<CGAL::Epick>> CenterPathNode::getBestPath() {
  if (!cone_map_) {
    return std::vector<Point>();
  }

  std::vector<std::pair<Point, unsigned int>> points;

  for (int i = 0; i < static_cast<int>(cone_map_->left_cones.size()); i++) {
    points.push_back(std::make_pair(
        Point(cone_map_->left_cones[i].pos.x, cone_map_->left_cones[i].pos.y),
        i));
  }
  for (int i = 0; i < static_cast<int>(cone_map_->right_cones.size()); i++) {
    points.push_back(std::make_pair(
        Point(cone_map_->right_cones[i].pos.x, cone_map_->right_cones[i].pos.y),
        cone_map_->left_cones.size() + i));
  }

  Delaunay T;

  T.insert(points.begin(), points.end());
  // vertices located in an array with starting pointer
  // T.finite_vertices.begin()

  std::vector<std::pair<int, int>> midpoint_index_to_cone_indices;
  std::vector<std::vector<int>> cone_index_to_midpoint_indices(points.size());

  bool flag = false;

  std::vector<Point> midpoints;

  for (Delaunay::Finite_edges_iterator it = T.finite_edges_begin();
       it != T.finite_edges_end(); ++it) {
    Delaunay::Edge edge = *it;

    Delaunay::Vertex_handle vh1 =
        edge.first->vertex((edge.second + 1) % 3); // First vertex of the edge
    Delaunay::Vertex_handle vh2 =
        edge.first->vertex((edge.second + 2) % 3); // Second vertex of the edge

    int index1 = vh1->info();
    int index2 = vh2->info();
    if ((index1 < static_cast<int>(cone_map_->left_cones.size())) ==
        (index2 < static_cast<int>(cone_map_->left_cones.size()))) {
      continue;
    }

    Point p1 = vh1->point();
    Point p2 = vh2->point();

    Point midpoint = Point((p1.x() + p2.x()) / 2.0, (p1.y() + p2.y()) / 2.0);

    double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);
    double global_x = midpoint.x();
    double global_y = midpoint.y();
    double translated_x = global_x - ego_state_->pose.pose.position.x;
    double translated_y = global_y - ego_state_->pose.pose.position.y;
    double local_x = translated_x * cos(-yaw) - translated_y * sin(-yaw);

    if (local_x < 0) {
      continue;
    }

    midpoints.push_back(midpoint);
    int midpointIndex = midpoints.size() - 1;
    midpoint_index_to_cone_indices.push_back(std::make_pair(index1, index2));
    cone_index_to_midpoint_indices[index1].push_back(midpointIndex);
    cone_index_to_midpoint_indices[index2].push_back(midpointIndex);

    if (!flag) {
      //  RCLCPP_WARN(this->get_logger(), "Edge pair: (%d, %d)", index1,
      //  index2);
      flag = false;
    }
  }

  geometry_msgs::msg::PolygonStamped midpoint_viz;
  midpoint_viz.header.stamp = this->get_clock()->now();
  midpoint_viz.header.frame_id = "base_footprint";
  geometry_msgs::msg::Point32 point32;
  for (int i = 0; i < static_cast<int>(midpoints.size()); i++) {
    double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);
    double global_x = midpoints[i].x();
    double global_y = midpoints[i].y();
    double translated_x = global_x - ego_state_->pose.pose.position.x;
    double translated_y = global_y - ego_state_->pose.pose.position.y;
    double local_x = translated_x * cos(-yaw) - translated_y * sin(-yaw);
    double local_y = translated_x * sin(-yaw) + translated_y * cos(-yaw);

    point32.x = local_x;
    point32.y = -local_y;
    point32.z = 0.0;
    midpoint_viz.polygon.points.push_back(point32);
  }

  skidpad_path_publisher_->publish(midpoint_viz);

  std::vector<int> midpoint_indices_by_dist(midpoints.size());
  std::iota(midpoint_indices_by_dist.begin(), midpoint_indices_by_dist.end(),
            0);

  double car_tip_x =
      ego_state_->pose.pose.position.x +
      1.2 / 2.0 * cos(util::quaternionToYaw(ego_state_->pose.pose.orientation));
  double car_tip_y =
      ego_state_->pose.pose.position.y +
      1.2 / 2.0 * sin(util::quaternionToYaw(ego_state_->pose.pose.orientation));

  std::sort(midpoint_indices_by_dist.begin(), midpoint_indices_by_dist.end(),
            [&midpoints, &car_tip_x, &car_tip_y](int a, int b) {
              double distA =
                  std::sqrt(std::pow(midpoints[a].x() - car_tip_x, 2) +
                            std::pow(midpoints[a].y() - car_tip_y, 2));
              double distB =
                  std::sqrt(std::pow(midpoints[b].x() - car_tip_x, 2) +
                            std::pow(midpoints[b].y() - car_tip_y, 2));
              return distA < distB;
            });

  std::vector<std::vector<int>> all_paths;
  std::deque<std::vector<int>> q;

  for (size_t i = 0; i < std::min(static_cast<size_t>(4), midpoints.size());
       ++i) {
    q.push_back({midpoint_indices_by_dist[i]});
  }

  for (int i = 0; i < 3; ++i) {
    std::deque<std::vector<int>> nextQ;
    while (!q.empty()) {
      std::vector<int> path = q.front();
      q.pop_front();

      auto [a, b] = midpoint_index_to_cone_indices[path.back()];
      std::vector<int> neighbors = cone_index_to_midpoint_indices[a];
      neighbors.insert(neighbors.end(),
                       cone_index_to_midpoint_indices[b].begin(),
                       cone_index_to_midpoint_indices[b].end());

      for (int neighbor : neighbors) {
        if (std::find(path.begin(), path.end(), neighbor) == path.end()) {
          std::vector<int> newPath = path;
          newPath.push_back(neighbor);
          nextQ.push_back(newPath);
        }
      }
    }

    std::sort(nextQ.begin(), nextQ.end(),
              [&](const std::vector<int> &a, const std::vector<int> &b) {
                return midpointCostFunction(a, midpoints, points,
                                            cone_map_->right_cones,
                                            cone_map_->left_cones,
                                            midpoint_index_to_cone_indices) <
                       midpointCostFunction(b, midpoints, points,
                                            cone_map_->right_cones,
                                            cone_map_->left_cones,
                                            midpoint_index_to_cone_indices);
              });
    q.clear();
    for (size_t j = 0; j < std::min(static_cast<size_t>(4), nextQ.size());
         ++j) {
      q.push_back(nextQ[j]);
    }
    all_paths.insert(all_paths.end(), nextQ.begin(), nextQ.end());
    nextQ.clear();
  }

  std::sort(all_paths.begin(), all_paths.end(),
            [&](const std::vector<int> &a, const std::vector<int> &b) {
              return midpointCostFunction(a, midpoints, points,
                                          cone_map_->right_cones,
                                          cone_map_->left_cones,
                                          midpoint_index_to_cone_indices) <
                     midpointCostFunction(
                         b, midpoints, points, cone_map_->right_cones,
                         cone_map_->left_cones, midpoint_index_to_cone_indices);
            });

  if (!all_paths.empty()) {
    int best = 0;
    // double costO = midpointCostFunction(all_paths[0], midpoints, points,
    // cone_map_->right_cones, cone_map_->left_cones,
    // midpoint_index_to_cone_indices); for(int i = 1; i < all_paths.size();
    // i++){
    //   double costN = midpointCostFunction(all_paths[i], midpoints, points,
    //   cone_map_->right_cones, cone_map_->left_cones,
    //   midpoint_index_to_cone_indices); if(abs(1-costN/costO) <= 0.25 &&
    //   all_paths[best].size() < all_paths[i].size()){
    //     best = i;
    //   }
    // }
    const std::vector<int> &best_path_indices = all_paths[best];

    std::vector<Point> best_path_points;
    best_path_points.reserve(best_path_indices.size());
    double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);
    for (int index : best_path_indices) {
      double global_x = midpoints[index].x();
      double global_y = midpoints[index].y();
      double translated_x = global_x - ego_state_->pose.pose.position.x;
      double translated_y = global_y - ego_state_->pose.pose.position.y;
      double local_x = translated_x * cos(-yaw) - translated_y * sin(-yaw);
      double local_y = translated_x * sin(-yaw) + translated_y * cos(-yaw);
      best_path_points.push_back(Point(local_x, local_y));
      // RCLCPP_INFO(this->get_logger(), "Midpoint %d: %f, %f", index, local_x,
      //             local_y);
    }

    return best_path_points;
  } else {
    return std::vector<Point>();
  }
}

std::tuple<std::vector<CGAL::Point_2<CGAL::Epick>>, std::vector<double>,
           std::vector<double>>
CenterPathNode::BezierPoints(
    std::vector<CGAL::Point_2<CGAL::Epick>>
        midpoints) { // Creating functions x(t) and y(t) given midpoints
  std::vector<Point> bezier_points;

  unsigned long maxDegree = 5;
  int degree = std::min(maxDegree, midpoints.size());

  midpoints.push_back(Point(0, 0));

  for (int i = 0; i < static_cast<int>(maxDegree) + 1; i++) {
    bezier_points.push_back(Point(0, 0));
  }

  for (int i = 1; i <= degree;
       i++) { // std::min(bezier_points.size(), midpoints.size())
    bezier_points[i + maxDegree - degree] = midpoints[i - 1];
  }

  // std::vector<Point> a, b, c, d, e, f =
  // bezier_points[0],bezier_points[1],bezier_points[2],bezier_points[3],bezier_points[4],bezier_points[5];
  Point a = bezier_points[0];
  Point b = bezier_points[1];
  Point c = bezier_points[2];
  Point d = bezier_points[3];
  Point e = bezier_points[4];
  Point f = bezier_points[5];

  std::vector<double> xoft, yoft;

  xoft.push_back(-a.x() + 5 * b.x() - 10 * c.x() + 10 * d.x() - 5 * e.x() +
                 f.x());
  xoft.push_back(5 * a.x() - 20 * b.x() + 30 * c.x() - 20 * d.x() + 5 * e.x());
  xoft.push_back(-10 * a.x() + 30 * b.x() - 30 * c.x() + 10 * d.x());
  xoft.push_back(10 * a.x() - 20 * b.x() + 10 * c.x());
  xoft.push_back(-5 * a.x() + 5 * b.x());
  xoft.push_back(a.x());

  yoft.push_back(-a.y() + 5 * b.y() - 10 * c.y() + 10 * d.y() - 5 * e.y() +
                 f.y());
  yoft.push_back(5 * a.y() - 20 * b.y() + 30 * c.y() - 20 * d.y() + 5 * e.y());
  yoft.push_back(-10 * a.y() + 30 * b.y() - 30 * c.y() + 10 * d.y());
  yoft.push_back(10 * a.y() - 20 * b.y() + 10 * c.y());
  yoft.push_back(-5 * a.y() + 5 * b.y());
  yoft.push_back(a.y());

  std::vector<Point> bezier_curve;

  for (double t = 0; t < 1; t = t + 0.1) {
    double xval = xoft[0] * pow(t, 5) + xoft[1] * pow(t, 4) +
                  xoft[2] * pow(t, 3) + xoft[3] * pow(t, 2) + xoft[4] * t +
                  xoft[5];
    double yval = yoft[0] * pow(t, 5) + yoft[1] * pow(t, 4) +
                  yoft[2] * pow(t, 3) + yoft[3] * pow(t, 2) + yoft[4] * t +
                  yoft[5];
    Point temp = Point(xval, yval);
    // RCLCPP_WARN(this->get_logger(), "bezierpt: %f, %f", xval, yval);
    bezier_curve.push_back(temp);
  }
  return std::make_tuple(bezier_curve, xoft, yoft);
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
  case 10:
    if (large_orange_cones_size > 3 && small_orange_cones_size < 3) {
      curr_sector_ += 1;
    }
    break;

  case 11:
    if (large_orange_cones_size < 3) {
      curr_sector_ += 1;
    }
    break;

  case 12:
  case 13:
  case 14:
  case 15:
    if (time_diff > 10.0 && !lock_sector_) {
      if (loop_closed_) {
        if (checkPassedDatum(getSkidpadDatum(*cone_map_raw_), *ego_state_)) {
          last_time = curr_time;
          curr_sector_ += 1;
          lock_sector_ = true;
          RCLCPP_INFO(this->get_logger(), "Lap incremented: Global trigger");
        }
      } else {
        if (found_4_large_orange &&
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
    // if (loop_closed_ && checkPassedDatum(getSkidpadDatum(*cone_map_raw_), *ego_state_)) {
    //   RCLCPP_WARN(this->get_logger(), "Global lap incremented");
    // }
    break;
  case 16:
    if (left_size == 0 && right_size == 0) {
      curr_sector_ += 1;
    }
  }
}

bool CenterPathNode::checkPassedDatum(const utfr_msgs::msg::EgoState reference,
                      const utfr_msgs::msg::EgoState &current) {
  double ref_x = reference.pose.pose.position.x;
  double ref_y = reference.pose.pose.position.y;
  double ref_yaw = util::quaternionToYaw(reference.pose.pose.orientation);

  double cur_x = current.pose.pose.position.x + 2.0; // offset forward to represent nose of car
  double cur_y = current.pose.pose.position.y;
  double cur_yaw = util::quaternionToYaw(current.pose.pose.orientation);

  //for testing
  ref_yaw = cur_yaw;

  double dx = ref_x - cur_x;
  double dy = ref_y - cur_y;

  double tdist = sqrt(dx * dx + dy * dy);

  double dx_local = dx * cos(-ref_yaw) - dy * sin(-ref_yaw);

  if (abs(ref_yaw - cur_yaw) < 3.1415 / 2 && tdist < 3.0 && dx_local < 0.0 && datum_last_local_x_ >= 0.0) {
    //if alignment within 90 deg, distance less than 3m
    datum_last_local_x_ = dx_local;
    return true;
  }
  datum_last_local_x_ = dx_local;
  return false;

}

utfr_msgs::msg::EgoState CenterPathNode::getSkidpadDatum(const utfr_msgs::msg::ConeMap &cone_map) {
  utfr_msgs::msg::EgoState datum;

  if (cone_map.large_orange_cones.size() == 3) {
    double x = 0.0;
    double y = 0.0;
    //do x
    double baseX = cone_map.large_orange_cones[0].pos.x;
    for (int i = 0; i < 2; i++) {
      if (abs(baseX - cone_map.large_orange_cones[i].pos.x) > 0.5) {
        x += cone_map.large_orange_cones[i].pos.x;
        break;
      }
    }
    x = x / 2.0;
    //do y
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
  // RCLCPP_INFO(this->get_logger(), "Datum: %f, %f", datum.pose.pose.position.x, datum.pose.pose.position.y);
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = datum.pose.pose.position.x;
  marker.pose.position.y = -datum.pose.pose.position.y;
  marker.pose.position.z = 0;
  
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  lap_datum_publisher_->publish(marker);

  return datum;

}

void CenterPathNode::trackdriveLapCounter() {
  rclcpp::Time curr_time = this->get_clock()->now();
  double time_diff = (curr_time - last_time).seconds();

  int large_orange_cones_size = cone_detections_->large_orange_cones.size();

  double average_distance_to_cones = 0.0;

  for (int i = 0; i < static_cast<int>(large_orange_cones_size); i++) {
    average_distance_to_cones += util::euclidianDistance2D(
        cone_detections_->large_orange_cones[i].pos.x, 0.0,
        cone_detections_->large_orange_cones[i].pos.y, 0.0);
  }

  average_distance_to_cones =
      average_distance_to_cones / large_orange_cones_size;

  if (large_orange_cones_size == 4) {
    found_4_large_orange = true;
  }

  if (time_diff > 20.0 && !lock_sector_ && found_4_large_orange &&
      large_orange_cones_size < 4 && average_distance_to_cones < 5.0) {
    last_time = curr_time;
    curr_sector_ += 1;
    lock_sector_ = true;
  }

  if (found_4_large_orange && lock_sector_ && large_orange_cones_size == 0 &&
      time_diff > 5.0) {
    lock_sector_ = false;
    found_4_large_orange = false;
  }
}

void CenterPathNode::skidPadFit() {

  const std::string function_name{"skidPadFit:"};
  std::tuple<double, double, double, double> left_circle_s, left_circle_l,
      right_circle_s, right_circle_l, left_circle, right_circle;
  double m_left, m_right, c_left, c_right ;
  double xc1, yc1, xc2, yc2, r1, r2;
  if (curr_sector_ == 10 || curr_sector_ == 11 || curr_sector_ == 16 || curr_sector_ == 17) {
    std::vector<double> accel_path = getAccelPath();

    utfr_msgs::msg::ParametricSpline center_path_msg;

    double m = accel_path[0];
    double c = accel_path[1];

    std::vector<double> x = {0, 0, 0, 0, 1, 0};
    std::vector<double> y = {0, 0, 0, 0, m, c};

    center_path_msg.x_params = x;
    center_path_msg.y_params = y;
    center_path_msg.lap_count = curr_sector_;

    center_path_publisher_->publish(center_path_msg);

    geometry_msgs::msg::PolygonStamped circleavg;
    circleavg.header.frame_id = "base_footprint";
    circleavg.header.stamp = this->get_clock()->now();
    Point32 pointavg;

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

    circleavg.header.frame_id = "base_footprint";

    circleavg.header.stamp = this->get_clock()->now();

    double b = (xc1 + xc2) / 2.0;
    double k = (yc1 + yc2) / 2.0;
    double r = (small_radius_ + big_radius_) / 2.0;

    Point32 pointavg;

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
    avg_circle_msg.header.frame_id = "base_footprint";
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

    circleavg.header.frame_id = "base_footprint";
    circleavg.header.stamp = this->get_clock()->now();

    double b = (xc1 + xc2) / 2.0;
    double k = (yc1 + yc2) / 2.0;
    double r = (small_radius_ + big_radius_) / 2.0;

    Point32 pointavg;

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
    avg_circle_msg.header.frame_id = "base_footprint";
    avg_circle_msg.header.stamp = this->get_clock()->now();

    avg_circle_msg.skidpad_params = {b, k, r};
    avg_circle_msg.x_params = {0, 0, 0, 0, 0, 0};
    avg_circle_msg.y_params = {0, 0, 0, 0, 0, 0};
    avg_circle_msg.lap_count = curr_sector_;

    center_path_publisher_->publish(avg_circle_msg);
  }
}

void CenterPathNode::publishLine(double m_left, double m_right, double c_left,
                                 double c_right, double x_min, double x_max) {

  utfr_msgs::msg::ParametricSpline avg_line_msg;

  geometry_msgs::msg::PolygonStamped line1_stamped, line2_stamped, lineavg;

  line1_stamped.header.frame_id = "base_footprint";
  line2_stamped.header.frame_id = "base_footprint";
  lineavg.header.frame_id = "base_footprint";

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

  avg_line_msg.header.frame_id = "base_footprint";
  avg_line_msg.header.stamp = this->get_clock()->now();
  avg_line_msg.x_params = {0, 0, 0, 0, 1, 0};
  avg_line_msg.y_params = {
      0, 0, 0, 0, (m_left + m_right) / 2.0, (c_left + c_right) / 2.0};
  avg_line_msg.skidpad_params = {0, 0, 0};
  avg_line_msg.lap_count = curr_sector_;

  center_path_publisher_->publish(avg_line_msg);
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
  all_cones.insert(all_cones.end(), cone_map_->left_cones.begin(),
                  cone_map_->left_cones.end());
  all_cones.insert(all_cones.end(), cone_map_->right_cones.begin(),
                  cone_map_->right_cones.end());
  all_cones.insert(all_cones.end(), cone_map_->large_orange_cones.begin(),
                  cone_map_->large_orange_cones.end());
  std::sort(
      all_cones.begin(), all_cones.end(),
      [this](const utfr_msgs::msg::Cone &a, const utfr_msgs::msg::Cone &b) {
        return this->coneDistComparitor(a, b);
      });
  
  int all_size = all_cones.size();
  bool find = false;
  double best_xc_small, best_xc_large, best_yc_small, best_yc_large, best_r_small, best_r_large, best_xc, best_yc;
  int best = 0;

  for (int i = 0; i < all_size - 2; i++){
    for (int j = i + 1; j < all_size - 1; j++){
      for (int k = j + 1; k < all_size; k++){
        if (i != j && j != k && i != k){
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

          if (xc >= 5.0 || (turning == 1 && yc < 0) || (turning == 0 && yc > 0)){
            continue;
          }

          if (abs(r - small_radius_) < threshold_radius_){
            closest_radius = small_radius_;
            other_radius = big_radius_;
          }
          else if (abs(r - big_radius_) < threshold_radius_){
            closest_radius = big_radius_;
            other_radius = small_radius_;
          }
          else {
            closest_radius = r < (small_radius_ + big_radius_) / 2 ? small_radius_ : big_radius_;
            other_radius = r < (small_radius_ + big_radius_) / 2 ? big_radius_ : small_radius_;
          }
          double inner_threshold = closest_radius - threshold_radius_;
          double outer_threshold = closest_radius + threshold_radius_;
          double other_inner_threshold = other_radius - threshold_radius_;
          double other_outer_threshold = other_radius + threshold_radius_;
          int threshold = 0;
          for (int a = 0; a < all_size; a++) {
            if ((inner_threshold <
                    sqrt(
                        pow((yc - all_cones[a].pos.y), 2) +
                        pow((xc - all_cones[a].pos.x), 2)) &&
                outer_threshold >
                    sqrt(
                        pow((xc - all_cones[a].pos.x), 2) +
                        pow((yc - all_cones[a].pos.y), 2))) || 
                (other_inner_threshold <
                    sqrt(
                        pow((yc - all_cones[a].pos.y), 2) +
                        pow((xc - all_cones[a].pos.x), 2)) &&
                other_outer_threshold >
                    sqrt(
                        pow((xc - all_cones[a].pos.x), 2) +
                        pow((yc - all_cones[a].pos.y), 2)))) {
              threshold += 1;
            }
          }

          if (threshold >= 2 * threshold_cones_){
            find = true;
          }

          if (threshold > best){
            best_xc = xc;
            best_yc = yc;
            best = threshold;
          }       
        }
        if (find == true){
          break;
        }
      }
      if (find == true){
        break;
      }
    }
    if (find == true){
      break;
    }
  }

  if (!find){
    best_xc = best_xc_small;
    best_yc = best_yc_small;
  }
  best_xc_small = best_xc;
  best_yc_small = best_yc;
  best_r_small = small_radius_;
  best_xc_large = best_xc;
  best_yc_large = best_yc;
  best_r_large = big_radius_;

  return std::make_tuple(best_xc_small, best_yc_small, best_r_small, best_xc_large, best_yc_large, best_r_large);
}

std::tuple<double, double, double, double, double, double>
CenterPathNode::skidpadRight() {
  std::vector<utfr_msgs::msg::Cone> all_cones;
  all_cones.insert(all_cones.end(), cone_map_->left_cones.begin(),
                  cone_map_->left_cones.end());
  all_cones.insert(all_cones.end(), cone_map_->right_cones.begin(),
                  cone_map_->right_cones.end());
  all_cones.insert(all_cones.end(), cone_map_->large_orange_cones.begin(),
                  cone_map_->large_orange_cones.end());
  std::sort(
      all_cones.begin(), all_cones.end(),
      [this](const utfr_msgs::msg::Cone &a, const utfr_msgs::msg::Cone &b) {
        return this->coneDistComparitor(a, b);
      });
  
  int all_size = all_cones.size();
  bool find = false;
  double best_xc_small, best_xc_large, best_yc_small, best_yc_large, best_r_small, best_r_large, best_xc, best_yc;
  int best = 0;

  for (int i = 0; i < all_size - 2; i++){
    for (int j = i + 1; j < all_size - 1; j++){
      for (int k = j + 1; k < all_size; k++){
        if (i != j && j != k && i != k){
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

          if (xc >= 5.0 || yc < 0){
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
            if (inner_threshold <
                    sqrt(
                        pow((yc - all_cones[a].pos.y), 2) +
                        pow((xc - all_cones[a].pos.x), 2)) &&
                outer_threshold >
                    sqrt(
                        pow((xc - all_cones[a].pos.x), 2) +
                        pow((yc - all_cones[a].pos.y), 2))) {
              threshold += 1;
              if (all_cones[i].type == utfr_msgs::msg::Cone::YELLOW){
                threshold += 3;
              }
            }
          }

          if (threshold > best){
            best_xc = xc;
            best_yc = yc;
            best = threshold;
          }
        }
        if (find == true){
          break;
        }
      }
      if (find == true){
        break;
      }
    }
    if (find == true){
      break;
    }
  }

  best_xc_small = best_xc;
  best_yc_small = best_yc;
  best_r_small = small_radius_;
  best_xc_large = best_xc;
  best_yc_large = best_yc;
  best_r_large = big_radius_;

  return std::make_tuple(best_xc_small, best_yc_small, best_r_small, best_xc_large, best_yc_large, best_r_large);
}

std::tuple<double, double, double, double, double, double>
CenterPathNode::skidpadLeft() {
  std::vector<utfr_msgs::msg::Cone> all_cones;
  all_cones.insert(all_cones.end(), cone_map_->left_cones.begin(),
                  cone_map_->left_cones.end());
  all_cones.insert(all_cones.end(), cone_map_->right_cones.begin(),
                  cone_map_->right_cones.end());
  all_cones.insert(all_cones.end(), cone_map_->large_orange_cones.begin(),
                  cone_map_->large_orange_cones.end());
  std::sort(
      all_cones.begin(), all_cones.end(),
      [this](const utfr_msgs::msg::Cone &a, const utfr_msgs::msg::Cone &b) {
        return this->coneDistComparitor(a, b);
      });
  
  int all_size = all_cones.size();
  bool find = false;
  double best_xc_small, best_xc_large, best_yc_small, best_yc_large, best_r_small, best_r_large, best_xc, best_yc;
  int best = 0;

  for (int i = 0; i < all_size - 2; i++){
    for (int j = i + 1; j < all_size - 1; j++){
      for (int k = j + 1; k < all_size; k++){
        if (i != j && j != k && i != k){
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

          if (xc >= 5.0 || yc > 0){
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
            if (inner_threshold <
                    sqrt(
                        pow((yc - all_cones[a].pos.y), 2) +
                        pow((xc - all_cones[a].pos.x), 2)) &&
                outer_threshold >
                    sqrt(
                        pow((xc - all_cones[a].pos.x), 2) +
                        pow((yc - all_cones[a].pos.y), 2))) {
              threshold += 1;
              if (all_cones[i].type == utfr_msgs::msg::Cone::BLUE){
                threshold += 3;
              }
            }
          }

          if (threshold > best){
            best_xc = xc;
            best_yc = yc;
            best = threshold;
          }
        }
        if (find == true){
          break;
        }
      }
      if (find == true){
        break;
      }
    }
    if (find == true){
      break;
    }
  }

  best_xc_small = best_xc;
  best_yc_small = best_yc;
  best_r_small = small_radius_;
  best_xc_large = best_xc;
  best_yc_large = best_yc;
  best_r_large = big_radius_;

  return std::make_tuple(best_xc_small, best_yc_small, best_r_small, best_xc_large, best_yc_large, best_r_large);
}


void CenterPathNode::publishLapTime(){
  utfr_msgs::msg::LapTime lap_time_msg;
  lap_time_msg.header.stamp = this->get_clock()->now();
  if (!last_sector){
    return;
  }
  rclcpp::Time curr_time = this->get_clock()->now();
  float curr_lap_time = (curr_time - last_switch_time).seconds();

  if (curr_sector_ != last_sector){
    last_lap_time = curr_time.seconds();
    last_switch_time = curr_time;
    if (curr_time.seconds() < best_lap_time){
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

bool CenterPathNode::hempishere(double x, double y, double r){
  std::vector<double> local_coord = globalToLocal(x, y);
  double local_x = local_coord[0];
  double local_y = local_coord[1];
  if (local_x * local_x + local_y * local_y < r * r && local_x > 0){
    return true;
  }
  else return false;
}

std::vector<utfr_msgs::msg::Cone> CenterPathNode::getConesInHemisphere(std::vector<utfr_msgs::msg::Cone> cones, double r){
  std::vector<utfr_msgs::msg::Cone> cones_in_hemisphere;
  for (int i = 0; i < static_cast<int>(cones.size()); i++){
    if (hempishere(cones[i].pos.x, cones[i].pos.y, r)){
      utfr_msgs::msg::Cone cone;
      std::vector<double> local_coord = globalToLocal(cones[i].pos.x, cones[i].pos.y);
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
