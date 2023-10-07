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

  update_rate_ = this->get_parameter("update_rate").as_double();
  event_ = this->get_parameter("event").as_string();
  small_radius_ = this->get_parameter("small_radius").as_double();
  big_radius_ = this->get_parameter("big_radius").as_double();
  threshold_radius_ = this->get_parameter("threshold_radius").as_double();
  threshold_cones_ = this->get_parameter("threshold_cones").as_int();
}

void CenterPathNode::initSubscribers() {
  ego_state_subscriber_ = this->create_subscription<utfr_msgs::msg::EgoState>(
      topics::kEgoState, 10, std::bind(&CenterPathNode::egoStateCB, this, _1));

  // cone_map_subscriber_ = this->create_subscription<utfr_msgs::msg::ConeMap>(
  //     topics::kConeMap, 10, std::bind(&CenterPathNode::coneMapCB, this, _1));

  cone_detection_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ConeDetections>(
          topics::kConeDetections, 10,
          std::bind(&CenterPathNode::coneDetectionsCB, this, _1));

  gt_cone_detection_subscriber_ =
      this->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
          topics::kEUFSConeMap, 10,
          std::bind(&CenterPathNode::gtConeDetectionsCB, this, _1));
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

void CenterPathNode::gtConeDetectionsCB(
    const eufs_msgs::msg::ConeArrayWithCovariance &msg) {
  if (gt_cone_detections_ == nullptr) {
    // first initialization:
    utfr_msgs::msg::ConeMap template_cone_map;
    gt_cone_detections_ =
        std::make_shared<utfr_msgs::msg::ConeMap>(template_cone_map);
  }
  gt_cone_detections_->header = msg.header;
  gt_cone_detections_->left_cones = convertToUTFRMsg(msg.blue_cones);
  gt_cone_detections_->right_cones = convertToUTFRMsg(msg.yellow_cones);
  gt_cone_detections_->large_orange_cones =
      convertToUTFRMsg(msg.big_orange_cones);
  gt_cone_detections_->small_orange_cones = convertToUTFRMsg(msg.orange_cones);
}

std::vector<utfr_msgs::msg::Cone> CenterPathNode::convertToUTFRMsg(
    const std::vector<eufs_msgs::msg::ConeWithCovariance> &input_cones) {
  std::vector<utfr_msgs::msg::Cone> output;
  for (const auto &cone : input_cones) {
    utfr_msgs::msg::Cone utfr_cone;
    utfr_cone.pos = cone.point;
    utfr_cone.covariance[0] = cone.covariance[0];
    utfr_cone.covariance[1] = cone.covariance[1];
    output.push_back(utfr_cone);
  }
  return output;
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
      skidPadFit(*cone_detections_, *ego_state_);

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

} // namespace center_path
} // namespace utfr_dv
