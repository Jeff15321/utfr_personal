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

  std::vector<double> x = {0, 0, 0, 0, 1, 0};
  std::vector<double> y = {0, 0, 0, 0, m, c};

  center_path_msg.x_params = x;
  center_path_msg.y_params = y;

  center_path_publisher_->publish(center_path_msg);
}

void CenterPathNode::timerCBSkidpad() {
  const std::string function_name{"center_path_timerCB:"};

  // CODE GOES HERE
}

void CenterPathNode::timerCBAutocross() {
  const std::string function_name{"center_path_timerCB:"};

  std::vector<Point> midpoints = Midpoints(cone_detections_);
  std::tuple<std::vector<Point>, std::vector<double>, std::vector<double>> result = BezierPoints(midpoints);
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

  for (int i = 0 ; i < static_cast<int>(bezier_curve.size()); i++) {
    midpoint_pt32.x = bezier_curve[i].x();
    midpoint_pt32.y = bezier_curve[i].y() * -1;
    midpoint_pt32.z = 0.0;

    delaunay_midpoints_stamped.polygon.points.push_back(midpoint_pt32);
  }

  for (int i = static_cast<int>(bezier_curve.size()) - 1 ; i > 0; i--) {
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
}

void CenterPathNode::timerCBTrackdrive() {
  const std::string function_name{"center_path_timerCB:"};

  std::vector<Point> midpoints = Midpoints(cone_detections_);
  std::tuple<std::vector<Point>, std::vector<double>, std::vector<double>> result = BezierPoints(midpoints);
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

  for (int i = 0 ; i < static_cast<int>(bezier_curve.size()); i++) {
    midpoint_pt32.x = bezier_curve[i].x();
    midpoint_pt32.y = bezier_curve[i].y() * -1;
    midpoint_pt32.z = 0.0;

    delaunay_midpoints_stamped.polygon.points.push_back(midpoint_pt32);
  }

  for (int i = static_cast<int>(bezier_curve.size()) - 1 ; i > 0; i--) {
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

std::vector<CGAL::Point_2<CGAL::Epick> > CenterPathNode::Midpoints(utfr_msgs::msg::ConeDetections_<std::allocator<void> >::SharedPtr cone_detections_) {
  if (!cone_detections_) {
    return std::vector<Point>();
  }
  
  std::vector< std::pair<Point, unsigned int> > points;
  
  for (int i = 0; i < cone_detections_->left_cones.size(); i++) {
    points.push_back( std::make_pair(Point(cone_detections_->left_cones[i].pos.x, cone_detections_->left_cones[i].pos.y), i ));
  }
  for (int i = 0; i < cone_detections_->right_cones.size(); i++) {
    points.push_back( std::make_pair(Point(cone_detections_->right_cones[i].pos.x, cone_detections_->right_cones[i].pos.y), cone_detections_->left_cones.size() + i ));
  }

  Delaunay T;

  T.insert(points.begin(), points.end());
  // vertices located in an array with starting pointer T.finite_vertices.begin()

  bool flag = false;

  std::vector<Point> midpoints;

  midpoints.push_back(Point(0, 0));
  
  for (Delaunay::Finite_edges_iterator it = T.finite_edges_begin(); it != T.finite_edges_end(); ++it) {
    Delaunay::Edge edge=*it;

    Delaunay::Vertex_handle vh1 = edge.first->vertex((edge.second + 1) % 3);  // First vertex of the edge
    Delaunay::Vertex_handle vh2 = edge.first->vertex((edge.second + 2) % 3);  // Second vertex of the edge
    

    int index1 = vh1->info();
    int index2 = vh2->info();
    if ((index1 < cone_detections_->left_cones.size()) == (index2 < cone_detections_->left_cones.size())) {
      continue;
    }

    Point p1 = vh1->point();
    Point p2 = vh2->point();

    Point midpoint = Point((p1.x() + p2.x()) / 2.0, (p1.y() + p2.y()) / 2.0);
    midpoints.push_back(midpoint);

    if (!flag) {
      //  RCLCPP_WARN(this->get_logger(), "Edge pair: (%d, %d)", index1, index2);
       flag = false;
    }
  }
  
  if (midpoints.empty()) {
    RCLCPP_WARN(this->get_logger(), "No midpoints found");
  }
  
  // first sort cones based on distance from car
  std::sort(midpoints.begin(), midpoints.end(), [] (Point a, Point b) { return MidpointCostFunction(a.x(), a.y(), b.x(), b.y()); });
  // second sort cones based on angle bewteen them
  for (int i = 0; i < midpoints.size()-1; i++) {
    float max_angle = M_PI/4;
    float dx = abs(midpoints[i].x() - midpoints[i+1].x());
    float dy = abs(midpoints[i].y() - midpoints[i+1].y());
    if (atan(dy/dx) > max_angle) {
      midpoints.erase(midpoints.begin() + (i+1));
    }
  }
  return midpoints;
}

std::tuple<std::vector<CGAL::Point_2<CGAL::Epick> >, std::vector<double>, std::vector<double>> CenterPathNode::BezierPoints(std::vector<CGAL::Point_2<CGAL::Epick> > midpoints) {  // Creating functions x(t) and y(t) given midpoints
  std::vector<Point> bezier_points;

  unsigned long maxDegree = 5;
  int degree = std::min(maxDegree, midpoints.size());

  midpoints.push_back(Point(0,0));

  for (int i = 0; i < maxDegree + 1; i++) {
    bezier_points.push_back(Point(0,0));
  }

  for (int i = 1; i <= degree; i++) { //std::min(bezier_points.size(), midpoints.size())
    bezier_points[i + maxDegree - degree] = midpoints[i-1];
  }

  // std::vector<Point> a, b, c, d, e, f = bezier_points[0],bezier_points[1],bezier_points[2],bezier_points[3],bezier_points[4],bezier_points[5];
  Point a = bezier_points[0];
  Point b = bezier_points[1];
  Point c = bezier_points[2];
  Point d = bezier_points[3];
  Point e = bezier_points[4];
  Point f = bezier_points[5];

  std::vector<double> xoft, yoft;

  xoft.push_back(-a.x()     + 5*b.x()   -10*c.x()   +10*d.x()   -5*e.x()  +f.x());
  xoft.push_back(5*a.x()    - 20*b.x()  + 30*c.x()  -20*d.x()   +5*e.x());
  xoft.push_back(-10*a.x()  + 30*b.x()  - 30*c.x()  + 10*d.x());
  xoft.push_back(10*a.x()   - 20*b.x()  + 10*c.x());
  xoft.push_back(-5*a.x()   + 5*b.x());
  xoft.push_back(a.x());

  yoft.push_back(-a.y()     + 5*b.y()   -10*c.y()   +10*d.y()   -5*e.y()  +f.y());
  yoft.push_back(5*a.y()    - 20*b.y()  + 30*c.y()  -20*d.y()   +5*e.y());
  yoft.push_back(-10*a.y()  + 30*b.y()  - 30*c.y()  + 10*d.y());
  yoft.push_back(10*a.y()   - 20*b.y()  + 10*c.y());
  yoft.push_back(-5*a.y()   + 5*b.y());
  yoft.push_back(a.y());
  

  std::vector<Point> bezier_curve;

  for (double t = 0; t < 1; t = t + 0.1) {
    double xval = xoft[0]*pow(t, 5) +xoft[1]*pow(t, 4) +xoft[2]*pow(t, 3) +xoft[3]*pow(t, 2) +xoft[4]*t + xoft[5];
    double yval = yoft[0]*pow(t, 5) +yoft[1]*pow(t, 4) +yoft[2]*pow(t, 3) +yoft[3]*pow(t, 2) +yoft[4]*t + yoft[5];
    Point temp = Point(xval, yval);
    // RCLCPP_WARN(this->get_logger(), "bezierpt: %f, %f", xval, yval);
    bezier_curve.push_back(temp);
  }
  return std::make_tuple(bezier_curve, xoft, yoft);
}

} // namespace center_path
} // namespace utfr_dv
