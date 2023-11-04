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
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

void CenterPathNode::initParams() {
  this->declare_parameter("update_rate", 33.33);
  this->declare_parameter("event", "accel");

  update_rate_ = this->get_parameter("update_rate").as_double();
  event_ = this->get_parameter("event").as_string();
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
}

void CenterPathNode::initPublishers() {
  center_path_publisher_ =
      this->create_publisher<utfr_msgs::msg::ParametricSpline>(
          topics::kCenterPath, 10);

  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kCenterPathHeartbeat, 10);

  accel_path_publisher_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>(
          topics::kAccelPath, 10);
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
  }
}

void CenterPathNode::initHeartbeat() {
  heartbeat_.module.data = "center_path_node";
  heartbeat_.update_rate = update_rate_;
}

void CenterPathNode::publishHeartbeat(const int status) {
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_publisher_->publish(heartbeat_);
}

void CenterPathNode::egoStateCB(const utfr_msgs::msg::EgoState &msg) {
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

void CenterPathNode::coneMapCB(const utfr_msgs::msg::ConeMap &msg) {
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
}

void CenterPathNode::timerCBAccel() {
  const std::string function_name{"center_path_timerCB:"};

  std::vector<double> accel_path = getAccelPath();

  utfr_msgs::msg::ParametricSpline center_path_msg;

  double m = accel_path[0];
  double c = accel_path[1];

  std::vector<double> x = {0,0,0,0,1,0};
  std::vector<double> y = {0,0,0,0,m,c};

  center_path_msg.x_params = x;
  center_path_msg.y_params = y;

  center_path_publisher_->publish(center_path_msg);

  // CODE GOES HERE
}

void CenterPathNode::timerCBSkidpad() {
  const std::string function_name{"center_path_timerCB:"};

  // CODE GOES HERE
}

void CenterPathNode::timerCBAutocross() {
  const std::string function_name{"center_path_timerCB:"};

  // CODE GOES HERE
}

void CenterPathNode::timerCBTrackdrive() {
  const std::string function_name{"center_path_timerCB:"};

  // CODE GOES HERE
}

bool CenterPathNode::coneDistComparitor(const utfr_msgs::msg::Cone& a,
                                       const utfr_msgs::msg::Cone& b) {
  double dist_a = util::euclidianDistance2D(a.pos.x, 0.0, a.pos.y, 0.0);
  double dist_b = util::euclidianDistance2D(b.pos.x, 0.0, b.pos.y, 0.0);

  return dist_a < dist_b;
}

std::vector<double> CenterPathNode::getAccelPath() {
  std::vector<utfr_msgs::msg::Cone> all_cones;
  all_cones.insert(all_cones.end(), cone_detections_->left_cones.begin(), 
      cone_detections_->left_cones.end());
  all_cones.insert(all_cones.end(), cone_detections_->right_cones.begin(), 
      cone_detections_->right_cones.end());
  all_cones.insert(all_cones.end(), cone_detections_->large_orange_cones.begin(),
      cone_detections_->large_orange_cones.end());
  all_cones.insert(all_cones.end(), cone_detections_->small_orange_cones.begin(),
      cone_detections_->small_orange_cones.end());

  std::sort(all_cones.begin(), all_cones.end(), [this](const utfr_msgs::msg::Cone& 
      a, const utfr_msgs::msg::Cone& b) {
    return this->coneDistComparitor(a, b);
  });


  bool found_1 = false;

  int ind_1, ind_2;
  double highest_1 = 0.0, highest_2 = 0.0;
  double best_m_1, best_m_2, best_c_1, best_c_2;

  for (int i = 0; i < all_cones.size() - 1; i++){
    for (int j = i + 1; j < all_cones.size(); j++){
      if (i != j){
        utfr_msgs::msg::ConeMap test_cones;
        utfr_msgs::msg::Cone test_cone;
        test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;
        test_cone.pos.x = all_cones[i].pos.x;
        test_cone.pos.y = all_cones[i].pos.y;
        test_cones.left_cones.push_back(test_cone);
        test_cone.pos.x = all_cones[j].pos.x;
        test_cone.pos.y = all_cones[j].pos.y;
        test_cones.left_cones.push_back(test_cone);

        std::tuple<double, double> test_line = util::accelLLS(test_cones.left_cones);

        double m = std::get<0>(test_line);
        double c = std::get<1>(test_line);

        if (abs(m) < 1.0){
          int count = 0;
          int cone_type;
          if (all_cones[i].type == all_cones[j].type){
            count += 1;
            cone_type = all_cones[i].type;
          }
          for (int k = 0; k < all_cones.size(); k++){
            double numerator = std::abs(m * all_cones[k].pos.x - 
                all_cones[k].pos.y + c);
            double denominator = std::sqrt(m * m + 1);
            double dist = numerator / denominator;
            if (dist < 0.5){
              count += 1;
              if (all_cones[k].type == cone_type){
                count += 1;
              }
            }
          }
          
          if (highest_1 <= count){
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
  for (int i = 0; i < all_cones.size() - 1; i++){
    for (int j = i + 1; j < all_cones.size(); j++){
      if (i != j && i != ind_1 && j != ind_1 && i != ind_2 && j != ind_2){
        utfr_msgs::msg::ConeMap test_cones;
        utfr_msgs::msg::Cone test_cone;
        test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;
        test_cone.pos.x = all_cones[i].pos.x;
        test_cone.pos.y = all_cones[i].pos.y;
        test_cones.left_cones.push_back(test_cone);
        test_cone.pos.x = all_cones[j].pos.x;
        test_cone.pos.y = all_cones[j].pos.y;
        test_cones.left_cones.push_back(test_cone);

        std::tuple<double, double> test_line = util::accelLLS(test_cones.left_cones);

        double m = std::get<0>(test_line);
        double c = std::get<1>(test_line);

        if (abs(m - best_m_1) < 0.2 && abs(c - best_c_1) > 2.5){
          int count = 0;
          int cone_type;
          if (all_cones[i].type == all_cones[j].type){
            count += 1;
            cone_type = all_cones[i].type;
          }
          for (int k = 0; k < all_cones.size(); k++){
            double numerator = std::abs(m * all_cones[k].pos.x - 
                all_cones[k].pos.y + c);
            double denominator = std::sqrt(m * m + 1);
            double dist = numerator / denominator;
            if (dist < 0.5){
              count += 1;
              if (all_cones[k].type == cone_type){
                count += 0.2;
              }
            }
          }
          if (highest_2 <= count){
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

  if (found_1 == true && found_2 == true){
    final_m = (best_m_1 + best_m_2) / 2;
    final_c = (best_c_1 + best_c_2) / 2;
  }
  else{
    if (found_1 == true){
      final_m = best_m_1;
      if (best_c_1 > 0) final_c = best_c_1 - 1.5;
      else final_c = best_c_1 + 1.5;
    }
    else if (found_2 == true){
      final_m = best_m_2;
      if (best_c_2 > 0) final_c = best_c_2 - 1.5;
      else final_c = best_c_2 + 1.5;
    }
    else{
      final_m = 0;
      final_c = 0;
    }
  }

  geometry_msgs::msg::PolygonStamped accel_path_msg;

  accel_path_msg.header.stamp = this->get_clock()->now();
  accel_path_msg.header.frame_id = "base_footprint";
  geometry_msgs::msg::Point32 point;
  point.x = 0;
  point.y = -final_c;
  point.z = 0;
  accel_path_msg.polygon.points.push_back(point);
  point.x = 15;
  point.y = - (final_m * 15 + final_c);
  point.z = 0;
  accel_path_msg.polygon.points.push_back(point);

  accel_path_publisher_->publish(accel_path_msg);

  std::vector<double> accel_path;
  accel_path.push_back(final_m);
  accel_path.push_back(final_c);
  return accel_path;
}
} // namespace center_path
} // namespace utfr_dv
