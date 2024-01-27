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
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
  this->initSector();
}

void CenterPathNode::initParams() {
  this->declare_parameter("update_rate", 33.33);
  this->declare_parameter("event", "accel");
  this->declare_parameter("big_radius", 10.625);
  this->declare_parameter("small_radius", 7.625);
  this->declare_parameter("threshold_radius", 0.8);
  this->declare_parameter("threshold_cones", 3);
  this->declare_parameter("global_path", 0);

  update_rate_ = this->get_parameter("update_rate").as_double();
  event_ = this->get_parameter("event").as_string();
  small_radius_ = this->get_parameter("small_radius").as_double();
  big_radius_ = this->get_parameter("big_radius").as_double();
  threshold_radius_ = this->get_parameter("threshold_radius").as_double();
  threshold_cones_ = this->get_parameter("threshold_cones").as_int();

  waypoints = this->getWaypoints("src/planning/global_waypoints/Waypoints.csv");
  global_path_ = this->get_parameter("global_path").as_int();
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

void CenterPathNode::initSector() {
  if (event_ == "accel") {
    curr_sector_ = 1;
  } else if (event_ == "skidpad") {
    curr_sector_ = 10;
  } else if (event_ == "autocross") {
    curr_sector_ = 20;
  } else if (event_ == "trackdrive") {
    curr_sector_ = 30;
  }
  last_time = this->get_clock()->now();
  lock_sector_ = true;
  found_4_large_orange = false;
}

void CenterPathNode::initHeartbeat() {
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
  }

  ego_state_->header = msg.header;
  ego_state_->pose = msg.pose;
  ego_state_->vel = msg.vel;
  ego_state_->accel = msg.accel;
  ego_state_->steering_angle = msg.steering_angle;
  ego_state_->lap_count = msg.lap_count;
  ego_state_->finished = msg.finished;

  // add points to the visited array for global skidpad path finding
  double curX = ego_state_->pose.pose.position.x;
  double curY = ego_state_->pose.pose.position.y;
  double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);
  if(visited.empty()){
    visited.push_back({curX,curY,yaw});
  }
  else{
    auto [lastX,lastY,_] = visited.back();
    if(util::euclidianDistance2D(lastX, curX, lastY, curY) >= 5){
      visited.push_back({curX,curY,yaw});
    }
  }
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

  this->createTransform(); // generate transform for global path
  this->GlobalWaypoints(); // visualize the transformed points in map view
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

  center_path_publisher_->publish(center_path_msg);

  publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);

  int left_size = cone_detections_->left_cones.size();
  int right_size = cone_detections_->right_cones.size();
  int large_orange_size = cone_detections_->large_orange_cones.size();

  if (!accel_sector_increase && left_size == 0 && right_size == 0 &&
      large_orange_size == 0) {
    accel_sector_increase = true;
    curr_sector_ += 1;
  }
}

void CenterPathNode::timerCBSkidpad() {
  const std::string function_name{"center_path_timerCB:"};

  try {
      if (cone_detections_ == nullptr || ego_state_ == nullptr) {
          RCLCPP_WARN(get_logger(), "%s Either cone detections or ego state is empty.", function_name.c_str());
          return; 
      }
      if(global_path_ && skidpadTransform_ != nullptr){
        RCLCPP_INFO(this->get_logger(), "USING GLOBAL PATH");
        this->nextWaypoint();
      }
      else{
        RCLCPP_INFO(this->get_logger(), "USING LOCAL PATH");
        skidPadFit(*cone_detections_, *ego_state_);
      }

  } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "%s Exception: %s", function_name.c_str(), e.what());
  }

  skidpadLapCounter();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
  RCLCPP_WARN(this->get_logger(), "Skidpad lap count: %d", curr_sector_);
}

void CenterPathNode::timerCBAutocross() {
  const std::string function_name{"center_path_timerCB:"};

  if (!cone_detections_) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                "Data not published or initialized yet. Using defaults.");
    return;
  }

  std::vector<Point> midpoints = Midpoints(cone_detections_);
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
  center_path_publisher_->publish(center_path);

  publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);

  trackdriveLapCounter();
}

void CenterPathNode::timerCBTrackdrive() {
  const std::string function_name{"center_path_timerCB:"};

  if (!cone_detections_) {
    RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                "Data not published or initialized yet. Using defaults.");
    return;
  }

  std::vector<Point> midpoints = Midpoints(cone_detections_);
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
  center_path_publisher_->publish(center_path);

  publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);

  trackdriveLapCounter();
}

bool CenterPathNode::coneDistComparitor(const utfr_msgs::msg::Cone &a,
                                        const utfr_msgs::msg::Cone &b) {
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
  all_cones.insert(all_cones.end(),
                   cone_detections_->large_orange_cones.begin(),
                   cone_detections_->large_orange_cones.end());
  all_cones.insert(all_cones.end(),
                   cone_detections_->small_orange_cones.begin(),
                   cone_detections_->small_orange_cones.end());

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
      if (i != j && i != ind_1 && j != ind_1 && i != ind_2 && j != ind_2) {
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

  std::vector<double> accel_path;
  accel_path.push_back(final_m);
  accel_path.push_back(final_c);
  return accel_path;
}

bool MidpointCostFunction(float x1, float y1, float x2, float y2) {
  float d1_sq = pow(x1, 2) + pow(y1, 2);
  float d2_sq = pow(x2, 2) + pow(y2, 2);
  return d1_sq < d2_sq;
}

std::vector<CGAL::Point_2<CGAL::Epick>> CenterPathNode::Midpoints(
    utfr_msgs::msg::ConeDetections_<std::allocator<void>>::SharedPtr
        cone_detections_) {
  if (!cone_detections_) {
    return std::vector<Point>();
  }

  std::vector<std::pair<Point, unsigned int>> points;

  for (int i = 0; i < cone_detections_->left_cones.size(); i++) {
    points.push_back(
        std::make_pair(Point(cone_detections_->left_cones[i].pos.x,
                             cone_detections_->left_cones[i].pos.y),
                       i));
  }
  for (int i = 0; i < cone_detections_->right_cones.size(); i++) {
    points.push_back(
        std::make_pair(Point(cone_detections_->right_cones[i].pos.x,
                             cone_detections_->right_cones[i].pos.y),
                       cone_detections_->left_cones.size() + i));
  }

  Delaunay T;

  T.insert(points.begin(), points.end());
  // vertices located in an array with starting pointer
  // T.finite_vertices.begin()

  bool flag = false;

  std::vector<Point> midpoints;

  midpoints.push_back(Point(0, 0));

  for (Delaunay::Finite_edges_iterator it = T.finite_edges_begin();
       it != T.finite_edges_end(); ++it) {
    Delaunay::Edge edge = *it;

    Delaunay::Vertex_handle vh1 =
        edge.first->vertex((edge.second + 1) % 3); // First vertex of the edge
    Delaunay::Vertex_handle vh2 =
        edge.first->vertex((edge.second + 2) % 3); // Second vertex of the edge

    int index1 = vh1->info();
    int index2 = vh2->info();
    if ((index1 < cone_detections_->left_cones.size()) ==
        (index2 < cone_detections_->left_cones.size())) {
      continue;
    }

    Point p1 = vh1->point();
    Point p2 = vh2->point();

    Point midpoint = Point((p1.x() + p2.x()) / 2.0, (p1.y() + p2.y()) / 2.0);
    midpoints.push_back(midpoint);

    if (!flag) {
      //  RCLCPP_WARN(this->get_logger(), "Edge pair: (%d, %d)", index1,
      //  index2);
      flag = false;
    }
  }

  if (midpoints.empty()) {
    RCLCPP_WARN(this->get_logger(), "No midpoints found");
  }

  // first sort cones based on distance from car
  std::sort(midpoints.begin(), midpoints.end(), [](Point a, Point b) {
    return MidpointCostFunction(a.x(), a.y(), b.x(), b.y());
  });
  // second sort cones based on angle bewteen them
  for (int i = 0; i < midpoints.size() - 1; i++) {
    float max_angle = M_PI / 4;
    float dx = abs(midpoints[i].x() - midpoints[i + 1].x());
    float dy = abs(midpoints[i].y() - midpoints[i + 1].y());
    if (atan(dy / dx) > max_angle) {
      midpoints.erase(midpoints.begin() + (i + 1));
    }
  }
  return midpoints;
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

  for (int i = 0; i < maxDegree + 1; i++) {
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
    break;
  case 16:
    if (left_size == 0 && right_size == 0 && large_orange_cones_size == 0) {
      curr_sector_ += 1;
    }
  }
}

void CenterPathNode::trackdriveLapCounter() {
  rclcpp::Time curr_time = this->get_clock()->now();
  double time_diff = (curr_time - last_time).seconds();

  int large_orange_cones_size = cone_detections_->large_orange_cones.size();

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

void CenterPathNode::skidPadFit(
    const utfr_msgs::msg::ConeDetections &cone_detections,
    const utfr_msgs::msg::EgoState &msg) {

  const std::string function_name{"skidPadFit:"};
  std::tuple<double, double, double, double> left_circle_s, left_circle_l,
      right_circle_s, right_circle_l, left_circle, right_circle;
  double m_left, b_left, m_right, b_right, c_left, c_right;
  double xc1, yc1, xc2, yc2, r1, r2;
  geometry_msgs::msg::Polygon circle1, circle2;
  circle1.points.reserve(75);
  circle2.points.reserve(75);

  std::cout << "curr_sector_: " << curr_sector_ << std::endl;
  float y_0 = 0;
  if (curr_sector_ == 10) {
    bool find = false;
    int ind1, ind2;

    for (int i = 0; i < cone_detections_->small_orange_cones.size() - 1; i++) {
      for (int j = i + 1; j < cone_detections_->small_orange_cones.size(); j++) {
        if (i != j) {
          utfr_msgs::msg::ConeMap cur_test;
          cur_test.small_orange_cones.push_back(
              cone_detections_->small_orange_cones[i]);
          cur_test.small_orange_cones.push_back(
              cone_detections_->small_orange_cones[j]);

          std::tuple<double, double> test_line =
              util::accelLLSOccupancy(cur_test.small_orange_cones);
          m_left = std::get<0>(test_line);
          if (abs(m_left) < 0.6) { // unsure about bounds
            find = true;
            m_left = std::get<0>(test_line);
            c_left = std::get<1>(test_line);
            ind1 = i;
            ind2 = j;
          }
        }
        if (find == true) {
          break;
        }
      }
      if (find == true) {
        break;
      }
    }
    if (cone_detections_->small_orange_cones.size() == 4) {
      utfr_msgs::msg::ConeMap cur_test_other;
      int possibilities[4] = {0, 1, 2, 3};
      for (int a = 0; a < 4; a++) {
        if (possibilities[a] != ind1 && possibilities[a] != ind2) {
          cur_test_other.small_orange_cones.push_back(
              cone_detections_->small_orange_cones[a]);
        }
      }

      std::tuple<double, double> test_other_line =
          util::accelLLSOccupancy(cur_test_other.small_orange_cones);
      m_right = std::get<0>(test_other_line);
      c_right = std::get<1>(test_other_line);
    } else {
      m_right = m_left;
      if (c_left < 0) {
        c_right = c_left + 3;
      } else {
        c_right = c_left - 3;
      }
    }

    publishLine(m_left, m_right, c_left, c_right, 0, 5, 0.02);
  }

  else if (curr_sector_ == 11) {
    bool find = false;
    int ind1, ind2;

    for (int i = 0; i < cone_detections_->large_orange_cones.size() - 1; i++) {
      for (int j = i + 1; j < cone_detections_->large_orange_cones.size(); j++) {
        if (i != j) {
          utfr_msgs::msg::ConeMap cur_test;
          cur_test.large_orange_cones.push_back(
              cone_detections_->large_orange_cones[i]);
          cur_test.large_orange_cones.push_back(
              cone_detections_->large_orange_cones[j]);

          std::tuple<double, double> test_line =
              util::accelLLSOccupancy(cur_test.large_orange_cones);
          m_left = std::get<0>(test_line);
          if (abs(m_left) < 0.6) { // unsure about bounds
            find = true;
            m_left = std::get<0>(test_line);
            c_left = std::get<1>(test_line);
            ind1 = i;
            ind2 = j;
          }
        }
        if (find == true) {
          break;
        }
      }
      if (find == true) {
        break;
      }
    }
    if (cone_detections_->large_orange_cones.size() == 4) {
      utfr_msgs::msg::ConeMap cur_test_other;
      int possibilities[4] = {0, 1, 2, 3};
      for (int a = 0; a < 4; a++) {
        if (possibilities[a] != ind1 && possibilities[a] != ind2) {
          cur_test_other.large_orange_cones.push_back(
              cone_detections_->large_orange_cones[a]);
        }
      }

      std::tuple<double, double> test_other_line =
          util::accelLLSOccupancy(cur_test_other.large_orange_cones);
      m_right = std::get<0>(test_other_line);
      c_right = std::get<1>(test_other_line);
    } else {
      m_right = m_left;
      if (c_left < 0) {
        c_right = c_left + 3;
      } else {
        c_right = c_left - 3;
      }
    }
     
    publishLine(m_left, m_right, c_left, c_right, 0, 5, 0.02);

  } else if (curr_sector_ == 12 || curr_sector_ == 13) {
    std::tuple<double, double, double, double, double, double> circle;
    if (cone_detections_->large_orange_cones.size() > 0 || cone_detections_->small_orange_cones.size() > 0) {
      circle = skidpadRight();
    }
    else{
      circle = skidpadMain();
    }

    xc1 = std::get<0>(circle);
    yc1 = std::get<1>(circle);
    r1 = std::get<2>(circle);
    xc2 = std::get<3>(circle);
    yc2 = std::get<4>(circle);
    r2 = std::get<5>(circle);

    geometry_msgs::msg::PolygonStamped circle1_stamped, circle2_stamped, circleavg;

    circle1_stamped.header.frame_id = "base_footprint";
    circle2_stamped.header.frame_id = "base_footprint";
    circleavg.header.frame_id = "base_footprint";

    circle1_stamped.header.stamp = this->get_clock()->now();
    circle2_stamped.header.stamp = this->get_clock()->now();
    circleavg.header.stamp = this->get_clock()->now();

    for (int i = 0; i < 75; ++i) {
      Point32 point1;
      Point32 point2;
      Point32 pointavg;

      double angle = 2.0 * M_PI * static_cast<double>(i) / 75.0;
      point1.x = xc1 + small_radius_ * cos(angle);
      point1.y = (yc1 + small_radius_ * sin(angle)) * -1;
      point1.z = 0;
      pointavg.x = (xc1 + xc2) / 2.0 + (small_radius_ + big_radius_) / 2.0 * cos(angle);
      pointavg.y = ((yc1 + yc2) / 2.0 + (small_radius_ + big_radius_) / 2.0 * sin(angle)) * -1;
      pointavg.z = 0;

      point2.x = xc2 + big_radius_ * cos(angle);
      point2.y = (yc2 + big_radius_ * sin(angle)) * -1;
      point2.z = 0;

      circle1_stamped.polygon.points.push_back(point1);
      circle2_stamped.polygon.points.push_back(point2);
      circleavg.polygon.points.push_back(pointavg);
    }

    skidpad_path_publisher_->publish(circle1_stamped);
    skidpad_path_publisher_2_->publish(circle2_stamped);
    skidpad_path_publisher_avg_->publish(circleavg);

    double b = (xc1 + xc2) / 2.0;
    double k = (yc1 + yc2) / 2.0;
    double r = (small_radius_ + big_radius_) / 2.0;

    utfr_msgs::msg::ParametricSpline avg_circle_msg;
    avg_circle_msg.header.frame_id = "base_footprint";
    avg_circle_msg.header.stamp = this->get_clock()->now();

    avg_circle_msg.x_params = {0,0,0,0,1,0};
    avg_circle_msg.y_params = {0,0,0,1/(2*r),-b/r, k-r+b*b/(2*r)};

    center_path_publisher_->publish(avg_circle_msg);
  }

  else if (curr_sector_ == 14 || curr_sector_ == 15) {
    std::tuple<double, double, double, double, double, double> circle;
    if (cone_detections_->large_orange_cones.size() > 0 || cone_detections_->small_orange_cones.size() > 0) {
      circle = skidpadLeft();
    }
    else{
      circle = skidpadMain();
    }

    xc1 = std::get<0>(circle);
    yc1 = std::get<1>(circle);
    r1 = std::get<2>(circle);
    xc2 = std::get<3>(circle);
    yc2 = std::get<4>(circle);
    r2 = std::get<5>(circle);

    geometry_msgs::msg::PolygonStamped circle1_stamped, circle2_stamped, circleavg;

    circle1_stamped.header.frame_id = "base_footprint";
    circle2_stamped.header.frame_id = "base_footprint";
    circleavg.header.frame_id = "base_footprint";
    circle1_stamped.header.stamp = this->get_clock()->now();
    circle2_stamped.header.stamp = this->get_clock()->now();
    circleavg.header.stamp = this->get_clock()->now();

    for (int i = 0; i < 75; ++i) {
      Point32 point1;
      Point32 point2;
      Point32 pointavg;

      double angle = 2.0 * M_PI * static_cast<double>(i) / 75.0;
      point1.x = xc1 + small_radius_ * cos(angle);
      point1.y = -1 * (yc1 + small_radius_ * sin(angle));
      point1.z = 0;

      pointavg.x = (xc1 + xc2) / 2.0 + (small_radius_ + big_radius_) / 2.0 * cos(angle);
      pointavg.y = -1 * ((yc1 + yc2) / 2.0 + (small_radius_ + big_radius_) / 2.0 * sin(angle));
      pointavg.z = 0;

      point2.x = xc2 + big_radius_ * cos(angle);
      point2.y = -1 * (yc2 + big_radius_ * sin(angle));
      point2.z = 0;

      circle1_stamped.polygon.points.push_back(point1);
      circle2_stamped.polygon.points.push_back(point2);
      circleavg.polygon.points.push_back(pointavg);
    }

    skidpad_path_publisher_->publish(circle1_stamped);
    skidpad_path_publisher_2_->publish(circle2_stamped);
    skidpad_path_publisher_avg_->publish(circleavg);

    double b = (xc1 + xc2) / 2.0;
    double k = (yc1 + yc2) / 2.0;
    double r = (small_radius_ + big_radius_) / 2.0;

    utfr_msgs::msg::ParametricSpline avg_circle_msg;
    avg_circle_msg.header.frame_id = "base_footprint";
    avg_circle_msg.header.stamp = this->get_clock()->now();

    avg_circle_msg.x_params = {0,0,0,0,1,0};
    avg_circle_msg.y_params = {0,0,0,-1/(2*r),b/r, k+r-b*b/(2*r)};

    center_path_publisher_->publish(avg_circle_msg);
  }

  else if (curr_sector_ == 16) {
    bool find = false;
    int ind1, ind2;
    for (int i = 0; i < cone_detections_->small_orange_cones.size() - 1; i++) {
      for (int j = i + 1; j < cone_detections_->small_orange_cones.size(); j++) {
        if (i != j) {
          utfr_msgs::msg::ConeMap cur_test;
          cur_test.small_orange_cones.push_back(
              cone_detections_->small_orange_cones[i]);
          cur_test.small_orange_cones.push_back(
              cone_detections_->small_orange_cones[j]);
          std::tuple<double, double> test_line =
              util::accelLLSOccupancy(cur_test.small_orange_cones);
          m_left = std::get<0>(test_line);
          if (abs(m_left) < 0.8) { // unsure about bounds
            find = true;
            m_left = std::get<0>(test_line);
            c_left = std::get<1>(test_line);
            ind1 = i;
            ind2 = j;
          }
        }
        if (find == true) {
          break;
        }
      }
      if (find == true) {
        break;
      }
    }
    find = false;

    for (int k = 0; k < cone_detections_->small_orange_cones.size() - 1; k++) {
      for (int l = k + 1; l < cone_detections_->small_orange_cones.size(); l++) {
        if (k != l && k != ind1 && k != ind2 && l != ind1 && l != ind2) {
          utfr_msgs::msg::ConeMap cur_test_other;
          cur_test_other.small_orange_cones.push_back(
              cone_detections_->small_orange_cones[k]);
          cur_test_other.small_orange_cones.push_back(
              cone_detections_->small_orange_cones[l]);
          std::tuple<double, double> test_line =
              util::accelLLSOccupancy(cur_test_other.small_orange_cones);
          m_right = std::get<0>(test_line);
          c_right = std::get<1>(test_line);
          if (abs(c_right - c_left) > 2 &&
              abs(m_right - m_left) < 0.1) { // unsure about bounds
            find = true;
          }
        }
        if (find == true) {
          break;
        }
      }
      if (find == true) {
        break;
      }
    }

    publishLine(m_left, m_right, c_left, c_right, 0, 5, 0.02);
  }
}

void CenterPathNode::publishLine(
  double m_left, double m_right, double c_left, double c_right, double x_min, 
  double x_max, double thickness) {

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

  center_path_publisher_->publish(avg_line_msg);
}

std::tuple<double, double, double, double, double, double> CenterPathNode::skidpadMain(){
  int left_size = cone_detections_->left_cones.size();
  int right_size = cone_detections_->right_cones.size();
  bool leftFind = false;
  bool rightFind = false;
  double xc1, yc1, xc2, yc2, r1, r2;
  int turning;
  if (curr_sector_ == 12 || curr_sector_ == 13){
    turning = 1;
  }
  else{
    turning = 0;
  }
  if (left_size > 2 && right_size > 2)
  {
    for (int i = 0; i < left_size - 2; i++)
    {
      for (int j = i + 1; j < left_size - 1; j++)
      {
        for (int k = j + 1; k < left_size; k++)
        {
          if (i != j && j != k && i != k)
          {
            utfr_msgs::msg::ConeMap cur_test_left;
            utfr_msgs::msg::Cone cur_test_cone;
            cur_test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;

            int insideThresholdLeft = 0;
            std::tuple<double, double, double, double> circle;

            cur_test_cone.pos.x = 
                cone_detections_->left_cones[i].pos.x;
            cur_test_cone.pos.y = 
                cone_detections_->left_cones[i].pos.y;
            (cur_test_left.left_cones).push_back(cur_test_cone);
            cur_test_cone.pos.x = 
                cone_detections_->left_cones[j].pos.x;
            cur_test_cone.pos.y = 
                cone_detections_->left_cones[j].pos.y;
            (cur_test_left.left_cones).push_back(cur_test_cone);
            cur_test_cone.pos.x = 
                cone_detections_->left_cones[k].pos.x;
            cur_test_cone.pos.y = 
                cone_detections_->left_cones[k].pos.y;
            (cur_test_left.left_cones).push_back(cur_test_cone);
            if (turning == 0){
              circle = util::ransacCircleLSF(cur_test_left.left_cones,small_radius_);
            }
            else if (turning == 1){
              circle = util::ransacCircleLSF(cur_test_left.left_cones,big_radius_);
            }
            r1 = std::get<2>(circle);
            yc1 = std::get<1>(circle);
            xc1 = std::get<0>(circle);
            double outer_threshold_left = r1 + threshold_radius_;
            double inner_threshold_left = r1 - threshold_radius_;
            for (int a = 0; a < left_size; a++)
            {
              if (inner_threshold_left < sqrt(pow((xc1 -
                  cone_detections_->left_cones[a].pos.x), 2) +
                  pow((yc1 - cone_detections_->left_cones[a].pos.y), 2)) &&
                  outer_threshold_left > sqrt(pow((xc1 -
                  cone_detections_->left_cones[a].pos.x),2) +
                  pow((yc1 - cone_detections_->left_cones[a].pos.y),2)))
              {
                insideThresholdLeft += 1;
              }
            }
            if (((turning == 0 && insideThresholdLeft >= threshold_cones_ && yc1<0)
                || (insideThresholdLeft >= threshold_cones_ && yc1>0 && turning == 1)) 
                && xc1<3.0){
              leftFind = true;
            }
          }
          if (leftFind == true)
          {
            break;
          }
        }
        if (leftFind == true)
        {
          break;
        }
      }
      if (leftFind == true)
      {
        break;
      }
    } 
      for (int i = 0; i < right_size - 2; i++)
      {
        for (int j = i + 1; j < right_size - 1; j++)
        {
          for (int k = j + 1; k < right_size; k++)
          {
            if (i != j && j != k && i != k)
            {
              utfr_msgs::msg::ConeMap cur_test_right;
              utfr_msgs::msg::Cone cur_test_cone;
              cur_test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;

              int insideThresholdRight = 0;
              std::tuple<double, double, double, double> circle;
              
              cur_test_cone.pos.x = cone_detections_->right_cones[i].pos.x;
              cur_test_cone.pos.y = cone_detections_->right_cones[i].pos.y;
              (cur_test_right.right_cones).push_back(cur_test_cone);
              cur_test_cone.pos.x = cone_detections_->right_cones[j].pos.x;
              cur_test_cone.pos.y = cone_detections_->right_cones[j].pos.y;
              (cur_test_right.right_cones).push_back(cur_test_cone);
              cur_test_cone.pos.x = cone_detections_->right_cones[k].pos.x;
              cur_test_cone.pos.y = cone_detections_->right_cones[k].pos.y;
              (cur_test_right.right_cones).push_back(cur_test_cone);
              if (turning == 1){
                circle = util::ransacCircleLSF(cur_test_right.right_cones, small_radius_);
              }
              else if (turning == 0){
                circle = util::ransacCircleLSF(cur_test_right.right_cones, big_radius_);
              }
              r2 = std::get<2>(circle);
              xc2 = std::get<0>(circle);
              yc2 = std::get<1>(circle);
              double outer_threshold_right = r2 + threshold_radius_;
              double inner_threshold_right = r2 - threshold_radius_;
              for (int a = 0; a < right_size; a++)
              {
                if (inner_threshold_right < sqrt(pow((xc2 -
                    cone_detections_->right_cones[a].pos.x),2) +
                    pow((yc2 - cone_detections_->right_cones[a].pos.y),2)) &&
                    outer_threshold_right > sqrt(pow((xc2 -
                    cone_detections_->right_cones[a].pos.x),2) +
                    pow((yc2 - cone_detections_->right_cones[a].pos.y),2)))
                {
                  insideThresholdRight += 1;
                }
              }
              if (((turning == 0 && insideThresholdRight >= threshold_cones_ && yc2<0)
                  || (insideThresholdRight >= threshold_cones_ && yc2>0 && turning == 1)) 
                  && xc2<3.0){
                rightFind = true;
              }
            }
            if (rightFind == true)
            {
              break;
            }
          }
          if (rightFind == true)
          {
            break;
          }
        }
      if (rightFind == true)
      {
        break;
      }
    }
  }
  else if (right_size > 2 && left_size < 3)
  {
    for (int i = 0; i < right_size - 2; i++)
    {
      for (int j = i + 1; j < right_size - 1; j++)
      {
        for (int k = j + 1; k < right_size; k++)
        {
          if (i != j && j != k && i != k)
          {
            utfr_msgs::msg::ConeMap cur_test_right;
            utfr_msgs::msg::Cone cur_test_cone;
            cur_test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;

            int insideThresholdRight = 0;
            std::tuple<double, double, double, double> circle;
            cur_test_cone.pos.x = cone_detections_->right_cones[i].pos.x;
            cur_test_cone.pos.y = cone_detections_->right_cones[i].pos.y;
            (cur_test_right.right_cones).push_back(cur_test_cone);
            cur_test_cone.pos.x = cone_detections_->right_cones[j].pos.x;
            cur_test_cone.pos.y = cone_detections_->right_cones[j].pos.y;
            (cur_test_right.right_cones).push_back(cur_test_cone);
            cur_test_cone.pos.x = cone_detections_->right_cones[k].pos.x;
            cur_test_cone.pos.y = cone_detections_->right_cones[k].pos.y;
            (cur_test_right.right_cones).push_back(cur_test_cone);
            
            if (turning == 1){
              circle = util::ransacCircleLSF(cur_test_right.right_cones, small_radius_);
              r1 = std::get<2>(circle);
              r2 = r1+3.0;
            }
            else if (turning == 0){
              circle = util::ransacCircleLSF(cur_test_right.right_cones, big_radius_);
              r1 = std::get<2>(circle);
              r2 = r1-3.0;
            }
            yc1 = std::get<1>(circle);
            yc2 = std::get<1>(circle);
            xc1 = std::get<0>(circle);
            xc2 = std::get<0>(circle);
              
            double outer_threshold_right = r1 + threshold_radius_;
            double inner_threshold_right = r1 - threshold_radius_;
            for (int a = 0; a < right_size; a++)
            {
              if (inner_threshold_right < sqrt(pow((yc1 -
                  cone_detections_->right_cones[a].pos.y), 2) +
                  pow((xc1 - cone_detections_->right_cones[a].pos.x), 2)) &&
                  outer_threshold_right > sqrt(pow((xc1 -
                  cone_detections_->right_cones[a].pos.x),2) +
                  pow((yc1 - cone_detections_->right_cones[a].pos.y),2)))
              {
                insideThresholdRight += 1;
              }
            }
            if (insideThresholdRight >= threshold_cones_ && xc1<=3.0)
            {
              rightFind = true;
            }
          }
          if (rightFind == true)
          {
            break;
          }
        }
        if (rightFind == true)
        {
          break;
        }
      }
      if (rightFind == true)
      {
        break;
      }
    }
  }
  else if (right_size < 3 && left_size > 2)
  {
    for (int i = 0; i < left_size - 2; i++)
    {
      for (int j = i + 1; j < left_size - 1; j++)
      {
        for (int k = j + 1; k < left_size; k++)
        {
          if (i != j && j != k && i != k)
          {
            utfr_msgs::msg::ConeMap cur_test_left;
            utfr_msgs::msg::Cone cur_test_cone;
            cur_test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;

            int insideThresholdLeft = 0;
            std::tuple<double, double, double, double> circle;

            cur_test_cone.pos.x = cone_detections_->left_cones[i].pos.x;
            cur_test_cone.pos.y = cone_detections_->left_cones[i].pos.y;
            (cur_test_left.left_cones).push_back(cur_test_cone);
            cur_test_cone.pos.x = cone_detections_->left_cones[j].pos.x;
            cur_test_cone.pos.y = cone_detections_->left_cones[j].pos.y;
            (cur_test_left.left_cones).push_back(cur_test_cone);
            cur_test_cone.pos.x = cone_detections_->left_cones[k].pos.x;
            cur_test_cone.pos.y = cone_detections_->left_cones[k].pos.y;
            (cur_test_left.left_cones).push_back(cur_test_cone);
            circle = util::ransacCircleLSF(cur_test_left.left_cones, small_radius_);
            r1 = std::get<2>(circle);
            r2 = r1 + 3.0;
            yc1 = std::get<1>(circle);
            yc2 = std::get<1>(circle);
            xc1 = std::get<0>(circle);
            xc2 = std::get<0>(circle);

            double outer_threshold_left = r1 + threshold_radius_;
            double inner_threshold_left = r1 - threshold_radius_;
            for (int a = 0; a < left_size; a++)
            {
              if (inner_threshold_left < sqrt(pow((xc1 - 
                  cone_detections_->left_cones[a].pos.x),2) + 
                  pow((yc1 - cone_detections_->left_cones[a].pos.y),2)) &&
                  outer_threshold_left > sqrt(pow((xc1 -
                  cone_detections_->left_cones[a].pos.x),2) + 
                  pow((yc1 - cone_detections_->left_cones[a].pos.y),2)))
              {
                insideThresholdLeft += 1;
              }
            }
            if (insideThresholdLeft >= threshold_cones_ && xc1<=3.0)
            {
              leftFind = true;
            }
          }
          if (leftFind == true)
          {
            break;
          }
        }
        if (leftFind == true)
        {
            break;
        }
      }
      if (leftFind == true)
      {
          break;
      }
    }
  }

  // make tuple with r and xc and yc

  return std::make_tuple(xc1, yc1, r1, xc2, yc2, r2);
}

std::tuple<double, double, double, double, double, double> CenterPathNode::skidpadRight(){
  int right_size = cone_detections_->right_cones.size();
  bool rightFind = false;
  double xc1, yc1, xc2, yc2, r1, r2;
  double best_xc1, best_yc1, best_r1, best_xc2, best_yc2, best_r2;
  int best_cones = 0;
  
  for (int i = 0; i < right_size - 2; i++)
  {
    for (int j = i + 1; j < right_size - 1; j++)
    {
      for (int k = j + 1; k < right_size; k++)
      {
        if (i != j && j != k && i != k)
        {
          utfr_msgs::msg::ConeMap cur_test_right;
          utfr_msgs::msg::Cone cur_test_cone;
          cur_test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;

          cur_test_cone.pos.x = 
              cone_detections_->right_cones[i].pos.x;
          cur_test_cone.pos.y = 
              cone_detections_->right_cones[i].pos.y;
          (cur_test_right.right_cones).push_back(cur_test_cone);
          cur_test_cone.pos.x = 
              cone_detections_->right_cones[j].pos.x;
          cur_test_cone.pos.y = 
              cone_detections_->right_cones[j].pos.y;
          (cur_test_right.right_cones).push_back(cur_test_cone);
          cur_test_cone.pos.x = 
              cone_detections_->right_cones[k].pos.x;
          cur_test_cone.pos.y = 
              cone_detections_->right_cones[k].pos.y;
          (cur_test_right.right_cones).push_back(cur_test_cone);

          int insideThresholdRight = 0;
          std::tuple<double, double, double, double>circle = 
              util::ransacCircleLSF(cur_test_right.right_cones, small_radius_);

          xc1 = std::get<0>(circle);
          xc2 = std::get<0>(circle);
          yc1 = std::get<1>(circle);
          yc2 = std::get<1>(circle);
          r1 = std::get<2>(circle);
          r2 = r1 + 3.0;
          double outer_threshold_right = r1 + threshold_radius_;
          double inner_threshold_right = r1 - threshold_radius_;
          for (int a = 0; a < right_size; a++)
          {
            if (inner_threshold_right < sqrt(pow((yc1 -
                cone_detections_->right_cones[a].pos.y), 2) +
                pow((xc1 - cone_detections_->right_cones[a].pos.x), 2)) &&
                outer_threshold_right > sqrt(pow((xc1 -
                cone_detections_->right_cones[a].pos.x),2) +
                pow((yc1 - cone_detections_->right_cones[a].pos.y),2)))
            {
              insideThresholdRight += 1;
            }
          }
          if (insideThresholdRight >= threshold_cones_ && yc1>=0 && xc1<=3.0)
          {
            rightFind = true;
          }
          else{
            if (insideThresholdRight>best_cones){
              best_cones = insideThresholdRight;
              best_xc1 = xc1;
              best_yc1 = yc1;
              best_r1 = r1;
              best_xc2 = xc2;
              best_yc2 = yc2;
              best_r2 = r2;
            }
          }
        }
        if (rightFind == true)
        {
          break;
        }
      }
      if (rightFind == true)
      {
        break;
      }
    }
    if (rightFind == true)
    {
      break;
    }
  }

  if (rightFind == false){
    xc1 = best_xc1;
    xc2 = best_xc2;
    yc1 = best_yc1;
    yc2 = best_yc2;
    r1 = best_r1;
    r2 = best_r2;
  }

  return std::make_tuple(xc1, yc1, r1, xc2, yc2, r2);
}

std::tuple<double, double, double, double, double, double> CenterPathNode::skidpadLeft(){
  int left_size = cone_detections_->left_cones.size();
  bool leftFind = false;
  double xc1, yc1, xc2, yc2, r1, r2;
  double best_xc1, best_yc1, best_r1, best_xc2, best_yc2, best_r2;
  int best_cones = 0;
  
  for (int i = 0; i < left_size - 2; i++)
  {
    for (int j = i + 1; j < left_size - 1; j++)
    {
      for (int k = j + 1; k < left_size; k++)
      {
        if (i != j && j != k && i != k)
        {
          utfr_msgs::msg::ConeMap cur_test_left;
          utfr_msgs::msg::Cone cur_test_cone;
          cur_test_cone.type = utfr_msgs::msg::Cone::UNKNOWN;

          cur_test_cone.pos.x = 
              cone_detections_->left_cones[i].pos.x;
          cur_test_cone.pos.y = 
              cone_detections_->left_cones[i].pos.y;
          (cur_test_left.left_cones).push_back(cur_test_cone);
          cur_test_cone.pos.x = 
              cone_detections_->left_cones[j].pos.x;
          cur_test_cone.pos.y = 
              cone_detections_->left_cones[j].pos.y;
          (cur_test_left.left_cones).push_back(cur_test_cone);
          cur_test_cone.pos.x = 
              cone_detections_->left_cones[k].pos.x;
          cur_test_cone.pos.y = 
              cone_detections_->left_cones[k].pos.y;
          (cur_test_left.left_cones).push_back(cur_test_cone);

          int insideThresholdLeft = 0;
          std::tuple<double, double, double, double>circle = 
              util::ransacCircleLSF(cur_test_left.left_cones, small_radius_);

          xc1 = std::get<0>(circle);
          xc2 = std::get<0>(circle);
          yc1 = std::get<1>(circle);
          yc2 = std::get<1>(circle);
          r1 = std::get<2>(circle);
          r2 = r1 + 3.0;
          double outer_threshold_left = r1 + threshold_radius_;
          double inner_threshold_left = r1 - threshold_radius_;
          for (int a = 0; a < left_size; a++)
          {
            if (inner_threshold_left < sqrt(pow((yc1 -
                cone_detections_->left_cones[a].pos.y), 2) +
                pow((xc1 - cone_detections_->left_cones[a].pos.x), 2)) &&
                outer_threshold_left > sqrt(pow((xc1 -
                cone_detections_->left_cones[a].pos.x),2) +
                pow((yc1 - cone_detections_->left_cones[a].pos.y),2)))
            {
              insideThresholdLeft += 1;
            }
          }
           if (insideThresholdLeft >= threshold_cones_ && yc1<=0 && xc1 <= 3.0)
          {
            leftFind = true;
          }
          else{
            if (insideThresholdLeft>best_cones){
              best_cones = insideThresholdLeft;
              best_xc1 = xc1;
              best_yc1 = yc1;
              best_r1 = r1;
              best_xc2 = xc2;
              best_yc2 = yc2;
              best_r2 = r2;
            }
          }
        }
        if (leftFind == true)
        {
          break;
        }
      }
      if (leftFind == true)
      {
          break;
      }
    }
    if (leftFind == true)
    {
        break;
    }
  }

  if (leftFind == false){
    xc1 = best_xc1;
    xc2 = best_xc2;
    yc1 = best_yc1;
    yc2 = best_yc2;
    r1 = best_r1;
    r2 = best_r2;
  }

  return std::make_tuple(xc1, yc1, r1, xc2, yc2, r2);
}

void CenterPathNode::nextWaypoint(){
  unsigned int i = 0;
  unsigned int j = 0;
  
  double carX = ego_state_->pose.pose.position.x;
  double carY = ego_state_->pose.pose.position.y;
  double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);

  visited.push_back({carX,carY,yaw});
  while(i < waypoints.size() && j < visited.size()){
    auto[pointX, pointY] = this->transformWaypoint(waypoints[i]);
    auto [carX,carY,yaw] = visited[j];
    double localY = (cos(yaw) * (pointY-carY)) - (sin(yaw) * (pointX-carX));
    double localX = (sin(yaw) * (pointY-carY)) + (cos(yaw) * (pointX-carX));
    double angle = util::wrapDeg(util::radToDeg(atan2(localY, localX)));
    if(angle >= 90 && angle <= 270){
      i++;
    }
    else{
      j++;
    }
  }
  visited.pop_back();

  if(i == waypoints.size()){
    RCLCPP_INFO(this->get_logger(), "NO GLOBAL PATH FOUND, USING LOCAL PATH");
    skidPadFit(*cone_detections_, *ego_state_);
    return;
  }

  using geometry_msgs::msg::PolygonStamped;
  static rclcpp::Publisher<PolygonStamped>::SharedPtr path_pub =
    this->create_publisher<PolygonStamped>("Waypoints", 1);

  PolygonStamped points_stamped;
  points_stamped.header.frame_id = "base_footprint";
  points_stamped.header.stamp = this->get_clock()->now();
  
  std::vector<Point> nextPoints;
  for(unsigned int k = i; k < i+6 && k < waypoints.size(); k++){
    auto [x,y] = this->transformWaypoint(waypoints[k]);
    double localY = (cos(yaw) * (y-carY)) - (sin(yaw) * (x-carX));
    double localX = (sin(yaw) * (y-carY)) + (cos(yaw) * (x-carX));
    geometry_msgs::msg::Point32 point;
    point.x = localX;
    point.y = -localY;
    point.z = 0;
    points_stamped.polygon.points.push_back(point);
    nextPoints.push_back(Point(localX, localY));
  }
  path_pub->publish(points_stamped);
  auto [useless, xParams, yParams] = BezierPoints(nextPoints);
  utfr_msgs::msg::ParametricSpline path;
  path.x_params = xParams;
  path.y_params = yParams;
  center_path_publisher_->publish(path);
}

void CenterPathNode::GlobalWaypoints(){
  if(skidpadTransform_ == nullptr) return;

  using geometry_msgs::msg::PolygonStamped;
  static rclcpp::Publisher<PolygonStamped>::SharedPtr waypoints_pub =
    this->create_publisher<PolygonStamped>("Waypoints", 1);

  PolygonStamped points_stamped;
  points_stamped.header.frame_id = "map";
  points_stamped.header.stamp = this->get_clock()->now();

  for(auto &p : waypoints) {
    auto [x,y] = this->transformWaypoint(p);
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = -y;
    point.z = 0;
    points_stamped.polygon.points.push_back(point);
  }
  waypoints_pub->publish(points_stamped);
}

std::vector<std::pair<double,double>> CenterPathNode::getWaypoints(std::string path){
  std::ifstream in(path);
  std::vector<std::pair<double,double>> v;
  while(!in.eof()){
    double x,y;
    in >> x >> y;
    v.push_back({x,y});
  }
  return v;
}

void CenterPathNode::createTransform(){
  double xLeft1,yLeft1;
  std::ifstream("src/planning/global_waypoints/LeftCentre.csv") >> xLeft1 >> yLeft1;
  double xRight1, yRight1;
  std::ifstream("src/planning/global_waypoints/RightCentre.csv") >> xRight1 >> yRight1;

  auto [xLeft2,yLeft2,xRight2,yRight2] = this->getCentres();
  if(isnan(xLeft2) || isnan(xLeft2) || isnan(xRight2) || isnan(yRight2)){
    skidpadTransform_ = nullptr;
    return;
  } 

  auto getIntersect = [&](double x0, double y0, double x1, double y1, int sign){
    double d = util::euclidianDistance2D(x0, x1, y0, y1);
    double a = d/2;
    double h = sqrt((15.25/2+3)*(15.25/2+3)-a*a);
    double xt = x0+a*(x1-x0)/d;
    double yt = y0+a*(y1-y0)/d;
    double x = xt+h*(y1-y0)/d;
    double y = yt-h*(x1-x0)/d;
    // return the coordinates that give the same signed cross product
    double cross = (x1-x) * (y0-y) - (x0-x) * (y1-y);
    if(cross * sign < 0){
      x = xt-h*(y1-y0)/d;
      y = yt+h*(x1-x0)/d;
    }
    return std::make_pair(x, y);
  };

  auto [xInter1, yInter1] = getIntersect(xLeft1, yLeft1, xRight1, yRight1, 1);
  auto [xInter2, yInter2] = getIntersect(xLeft2, yLeft2, xRight2, yRight2, -1);
  MatrixXd X(3,3); // a b tx
  X << xLeft1, yLeft1, 1,
       xRight1, yRight1, 1,
       xInter1, yInter1, 1;
  VectorXd Xb(3);
  Xb << xLeft2, xRight2, xInter2;
  VectorXd Xres = X.fullPivLu().solve(Xb);
  MatrixXd Y(3,3); // c d ty
  Y << xLeft1, yLeft1, 1,
       xRight1, yRight1, 1,
       xInter1, yInter1, 1;
  VectorXd Yb(3);
  Yb << yLeft2, yRight2, yInter2;
  VectorXd Yres = Y.fullPivLu().solve(Yb);
  MatrixXd T(3,3);
  T << Xres(0), Xres(1), Xres(2), Yres(0), Yres(1), Yres(2), 0, 0, 1;

  skidpadTransform_ = std::make_unique<MatrixXd>(T);
}

std::pair<double,double> CenterPathNode::transformWaypoint(const std::pair<double,double> &point){
  auto [x,y] = point;
  VectorXd v(3);
  v << x, y, 1;

  VectorXd out = *skidpadTransform_ * v;
  return {out(0), out(1)};
}

std::tuple<double,double,double,double> CenterPathNode::getCentres(){
  std::vector<utfr_msgs::msg::Cone> &orange = cone_map_->large_orange_cones;
  if(orange.size() != 4){
    return {NAN,NAN,NAN,NAN};
  }
  auto [xLeft,yLeft,xRight,yRight] = this->skidpadCircleCentres();
  if(isnan(xLeft) || isnan(xRight) || isnan(yLeft) || isnan(yRight)){
    return {NAN,NAN,NAN,NAN};
  }
  double xMid = (orange[0].pos.x+orange[1].pos.x+orange[2].pos.x+orange[3].pos.x)/4;
  double yMid = (orange[0].pos.y+orange[1].pos.y+orange[2].pos.y+orange[3].pos.y)/4;
  double rightDist = util::euclidianDistance2D(xRight, xMid, yRight, yMid);
  double leftDist = util::euclidianDistance2D(xMid, xLeft, yMid, yLeft);
  double totalDist = util::euclidianDistance2D(xRight, xLeft, yRight, yLeft);
  if(abs(rightDist-centre_distance_) > 0.25){ // improper right centre
    return {NAN,NAN,NAN,NAN};
  }
  // improper left centre, extrapolate it
  if(abs(totalDist-2*centre_distance_) > 0.5 || abs(leftDist-centre_distance_) > 0.25){
    xLeft = xMid*2-xRight;
    yLeft = yMid*2-yRight;
  }
  return {xLeft, yLeft, xRight, yRight};
}

std::tuple<double,double,double,double> CenterPathNode::skidpadCircleCentres(){
  using geometry_msgs::msg::PolygonStamped;
  static rclcpp::Publisher<PolygonStamped>::SharedPtr small_blue_pub =
    this->create_publisher<PolygonStamped>("SmallBlue", 1);
  static rclcpp::Publisher<PolygonStamped>::SharedPtr small_yellow_pub =
    this->create_publisher<PolygonStamped>("SmallYellow", 1);
  static rclcpp::Publisher<PolygonStamped>::SharedPtr large_blue_pub = 
    this->create_publisher<PolygonStamped>("LargeBlue", 1);
  static rclcpp::Publisher<PolygonStamped>::SharedPtr large_yellow_pub = 
    this->create_publisher<PolygonStamped>("LargeYellow", 1);
  
  std::vector<utfr_msgs::msg::Cone> &blue = cone_map_->left_cones;
  std::vector<utfr_msgs::msg::Cone> &yellow = cone_map_->right_cones;

  auto smallBlue = this->circleCentre(blue, small_radius_, small_circle_cones_-3);
  auto smallYellow = this->circleCentre(yellow, small_radius_, small_circle_cones_-3);
  auto largeBlue = this->circleCentre(blue, big_radius_, big_circle_cones_-3);
  auto largeYellow = this->circleCentre(yellow, big_radius_, big_circle_cones_-3);
  
  auto drawCircle = [this](auto publisher, auto &cord, double radius){
    auto [xc, yc] = cord;
    PolygonStamped circle_stamped;
    circle_stamped.header.frame_id = "map";
    circle_stamped.header.stamp = this->get_clock()->now();

    for(int i = 0; i < 360; i++) {
      geometry_msgs::msg::Point32 point;  
      double angle = 2.0 * M_PI * i / 360;
      point.x = xc + radius * cos(angle);
      point.y = -yc + radius * sin(angle);
      point.z = 0;
      circle_stamped.polygon.points.push_back(point);
    }
    publisher->publish(circle_stamped);
  };

  drawCircle(small_blue_pub, smallBlue, 0.1);
  drawCircle(small_yellow_pub, smallYellow, 0.1);
  drawCircle(large_blue_pub, largeBlue, 0.1);
  drawCircle(large_yellow_pub, largeYellow, 0.1);
  
  // average the centres of the 2 overlapping circles one each side
  double xLeft = (smallBlue.first+largeYellow.first)/2;
  double yLeft = (smallBlue.second+largeYellow.second)/2;
  double xRight = (smallYellow.first+largeBlue.first)/2;
  double yRight = (smallYellow.second+largeBlue.second)/2;
  return {xLeft,yLeft,xRight,yRight};
}

std::pair<double,double> CenterPathNode::circleCentre(std::vector<utfr_msgs::msg::Cone> &cones, double radius, int inlier_count){
  int n = cones.size();

  auto get_threshold = [&](double xc, double yc, double radius){
    // get the distances from the circle's centre
    std::vector<double> distances(n);
    for(int i = 0; i < n; i++){
      auto &pos = cones[i].pos;
      distances[i] = util::euclidianDistance2D(pos.x, xc, pos.y, yc); 
    }
    // sort the distances by closeness to the radius
    auto cmp = [&](double a, double b){
      return abs(a-radius) < abs(b-radius);
    };
    sort(distances.begin(), distances.end(), cmp);
    // find the threshold that contains inlier_count points
    double threshold;
    if(n < inlier_count) threshold = abs(radius-distances[n-1]);
    else threshold = abs(radius-distances[inlier_count-1]);
    return threshold;
  };

  auto circle = [](std::vector<utfr_msgs::msg::Cone> &cones){
    std::tuple<double,double,double> ret;
    if(cones.size() != 3) return ret = {NAN,NAN,NAN};
    MatrixXd A(2,2); // A -> [X Y]
    VectorXd B(2);
    auto x = [&](int i){return cones[i].pos.x;};
    auto y = [&](int i){return cones[i].pos.y;};
    // Solve 3 circle equations (x-a)^2 + (y-b)^2 = r^2
    // equation 1 - equation 2
    A(0,0) = 2*(x(0)-x(1));
    A(0,1) = 2*(y(0)-y(1));
    B(0) = (x(0)*x(0) + y(0)*y(0)) - (x(1)*x(1) + y(1)*y(1));
    // equation 1 - equation 3
    A(1,0) = 2*(x(0)-x(2));
    A(1,1) = 2*(y(0)-y(2));
    B(1) = (x(0)*x(0) + y(0)*y(0)) - (x(2)*x(2) + y(2)*y(2));
    // Solve for X & Y
    VectorXd ans = A.fullPivLu().solve(B);
    double xc = ans(0), yc = ans(1);
    double r = sqrt(pow(x(0)-xc, 2) + pow(y(0)-yc, 2));
    return ret = {xc, yc, r};
  };

  // find the circle with the lowest threshold
  double lowest_threshold = DBL_MAX;
  std::pair<double,double> centre = {NAN,NAN};
  std::vector<utfr_msgs::msg::Cone> ransacCones(3);
  double closest_radius = DBL_MAX;
  for(int i = 0; i < n; i++){
    ransacCones[0] = cones[i];
    for(int j = i+1; j < n; j++){
      ransacCones[1] = cones[j];
      for(int k = j+1; k < n; k++){
        ransacCones[2] = cones[k];
        auto [xc,yc,radiusf] = circle(ransacCones);
        double threshold = get_threshold(xc, yc, radiusf);
        if(abs(radiusf-radius) < closest_radius){
          lowest_threshold = threshold;
          centre = {xc,yc};
          closest_radius = abs(radiusf-radius);
        }
        else if(abs(radiusf-radius) == closest_radius && threshold < lowest_threshold){
          lowest_threshold = threshold;
          centre = {xc,yc};
        }
      }
    }
  }
  return centre;
}

} // namespace center_path
} // namespace utfr_dv
