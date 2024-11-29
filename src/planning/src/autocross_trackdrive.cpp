#include <center_path_node.hpp>

namespace utfr_dv {
namespace center_path {

void CenterPathNode::timerCBAutocross() {
  if (as_state != utfr_msgs::msg::SystemStatus::AS_STATE_DRIVING){
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }
  const std::string function_name{"center_path_timerCB:"};
  RCLCPP_WARN(this->get_logger(), "Autocross Timer CB");
  try {
    if (cone_detections_ == nullptr || cone_map_global_ == nullptr ||
        ego_state_ == nullptr) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                  "Data not published or initialized yet. Using defaults.");
      return;
    }
    if (cone_map_global_ == nullptr) {
      RCLCPP_WARN(get_logger(), "%s Cone Map is empty", function_name.c_str());
      return;
    }

    utfr_msgs::msg::ParametricSpline center_path = getBestPath();
    center_path_publisher_->publish(center_path);

    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);

    trackdriveLapCounter();
    publishLapTime();
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
}

void CenterPathNode::timerCBTrackdrive() {
  if (as_state != utfr_msgs::msg::SystemStatus::AS_STATE_DRIVING){
    publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
    return;
  }
  const std::string function_name{"center_path_timerCB:"};

  try {
    if (cone_detections_ == nullptr || cone_map_global_ == nullptr ||
        ego_state_ == nullptr) {
      RCLCPP_WARN(rclcpp::get_logger("TrajectoryRollout"),
                  "Data not published or initialized yet. Using defaults.");
      return;
    }
    if (cone_map_global_ == nullptr) {
      RCLCPP_WARN(get_logger(), "%s Cone Map is empty", function_name.c_str());
      return;
    }

    utfr_msgs::msg::ParametricSpline center_path = getBestPath();
    center_path_publisher_->publish(center_path);

    publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);

    trackdriveLapCounter();
    publishLapTime();
  } catch (int e) {
    publishHeartbeat(utfr_msgs::msg::Heartbeat::ERROR);
  }
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
                   const std::vector<utfr_msgs::msg::Cone> &cones,
                   const std::vector<int> &nodes) {
  std::vector<std::vector<double>> distances(nodes.size());
  for (int i = 0; i < static_cast<int>(nodes.size()); i++) {
    int node = nodes[i];
    for (const auto &cone : cones) {
      distances[i].push_back(util::euclidianDistance2D(
          midpoints[node].x(), cone.pos.x, midpoints[node].y(), cone.pos.y));
    }
  }
  return distances;
}

double findMaxMinDistance(const std::vector<std::vector<double>> &distances) {
  double maxMinDistance = 0;
  for (const auto &distArr : distances) {
      double minDistance = *std::min_element(distArr.begin(), distArr.end());
      maxMinDistance = std::max(maxMinDistance, minDistance);
  }
  return maxMinDistance;
}

double CenterPathNode::midpointCostFunction(
    const std::vector<int> &nodes,
    const std::vector<CGAL::Point_2<CGAL::Epick>> &midpoints,
    const std::vector<std::pair<CGAL::Point_2<CGAL::Epick>, unsigned int>> &all_cones,
    const std::vector<utfr_msgs::msg::Cone> &yellow_cones,
    const std::vector<utfr_msgs::msg::Cone> &blue_cones,
    const std::vector<std::pair<int, int>> &midpoint_index_to_cone_indices) {
  constexpr double A = 0.9;
  constexpr double B = 2.0;
  constexpr double C = 2.0;
  constexpr double D = 3.0;
  constexpr double E = 1.0;
  constexpr double F = 0.3;
  constexpr double Z = 0.1;

  constexpr double MAX_MAX_ANGLE = 3.3141592653589793;
  constexpr double MAX_MAX_MIDPOINT_TO_CONE_DIST = 25.0;
  constexpr double MAX_SQUARED_RANGE_PATH_LENGTH_DIFF = 
      MAX_MAX_MIDPOINT_TO_CONE_DIST * MAX_MAX_MIDPOINT_TO_CONE_DIST;
  constexpr int MAX_POINT_COUNT_COST = 10;
  constexpr double MAX_INTERPOLATED_MIDPOINT_TO_CONE_DISTANCE_COST = 1.2 / 2.0 * 4.0;
  constexpr double MAX_STD_DEV = 1.2;
  constexpr double LENGTH_COST = 0.2;

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

  double max_angle = -100000.0;
  for (int i = 1; i < static_cast<int>(nodes.size()) - 1; i++) {
    double angle =
        getTurnAngle(midpoints[nodes[i]].x() - midpoints[nodes[i - 1]].x(),
                     midpoints[nodes[i]].y() - midpoints[nodes[i - 1]].y(),
                     midpoints[nodes[i + 1]].x() - midpoints[nodes[i]].x(),
                     midpoints[nodes[i + 1]].y() - midpoints[nodes[i]].y());
    if (angle > max_angle) max_angle = angle;
  }
  max_angle = std::max(std::max(initial_angle, next_angle), max_angle);

  double path_length = 0;
  for (int i = 0; i < static_cast<int>(nodes.size()) - 1; i++) {
    path_length += util::euclidianDistance2D(
        midpoints[nodes[i]].x(), midpoints[nodes[i + 1]].x(),
        midpoints[nodes[i]].y(), midpoints[nodes[i + 1]].y());
  }

  double squared_range_path_length_diff =
      pow(MAX_MAX_MIDPOINT_TO_CONE_DIST - path_length, 2);

  std::vector<std::vector<double>> midpoint_to_yellow_cone_distances;
  std::vector<std::vector<double>> midpoint_to_blue_cone_distances;

  midpoint_to_yellow_cone_distances =
      calculateDistances(midpoints, cone_map_global_->right_cones, nodes);

  midpoint_to_blue_cone_distances =
      calculateDistances(midpoints, cone_map_global_->left_cones, nodes);

  double max_midpoint_to_yellow_distance =
      findMaxMinDistance(midpoint_to_yellow_cone_distances);
  double max_midpoint_to_blue_distance =
      findMaxMinDistance(midpoint_to_blue_cone_distances);

  double min_interpolated_midpoint_to_cone_distance = -10000.0;
  for (size_t i = 1; i < nodes.size(); ++i) {
    Point interpolatedMidpoint =
        Point((midpoints[nodes[i - 1]].x() + midpoints[nodes[i]].x()) / 2.0,
              (midpoints[nodes[i - 1]].y() + midpoints[nodes[i]].y()) / 2.0);

    double minDistance = std::numeric_limits<double>::max();
    for (const auto &cone : yellow_cones) {
      double distance =
          util::euclidianDistance2D(cone.pos.x, interpolatedMidpoint.x(),
                                    cone.pos.y, interpolatedMidpoint.y());
      minDistance = std::min(minDistance, distance);
    }
    for (const auto &cone : blue_cones) {
      double distance =
          util::euclidianDistance2D(cone.pos.x, interpolatedMidpoint.x(),
                                    cone.pos.y, interpolatedMidpoint.y());
      minDistance = std::min(minDistance, distance);
    }
    if (minDistance > min_interpolated_midpoint_to_cone_distance)
      min_interpolated_midpoint_to_cone_distance = minDistance;
  }

  double length = 0.0;
  std::vector<double> track_widths;
  for (int node : nodes) {
    Point cone1 = all_cones[midpoint_index_to_cone_indices[node].first].first;
    Point cone2 = all_cones[midpoint_index_to_cone_indices[node].second].first;

    double width =
        util::euclidianDistance2D(cone1.x(), cone2.x(), cone1.y(), cone2.y());
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

utfr_msgs::msg::ParametricSpline CenterPathNode::getBestPath() {
  if (!cone_map_global_) {
    utfr_msgs::msg::ParametricSpline path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "ground";
    path.x_params = {0, 0, 0, 0, 0, 0};
    path.y_params = {0, 0, 0, 0, 0, 0};;
    path.lap_count = curr_sector_;
    return path;
  }

  std::vector<std::pair<Point, unsigned int>> points;

  for (int i = 0; i < static_cast<int>(cone_map_global_->left_cones.size()); i++) {
    points.push_back(std::make_pair(
        Point(cone_map_global_->left_cones[i].pos.x, cone_map_global_->left_cones[i].pos.y),
        i));
  }
  for (int i = 0; i < static_cast<int>(cone_map_global_->right_cones.size()); i++) {
    points.push_back(std::make_pair(
        Point(cone_map_global_->right_cones[i].pos.x, cone_map_global_->right_cones[i].pos.y),
        cone_map_global_->left_cones.size() + i));
  }

  Delaunay T;

  T.insert(points.begin(), points.end());
  // vertices located in an array with starting pointer
  // T.finite_vertices.begin()

  std::vector<std::pair<int, int>> midpoint_index_to_cone_indices;
  std::vector<std::vector<int>> cone_index_to_midpoint_indices(points.size());

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
    if ((index1 < static_cast<int>(cone_map_global_->left_cones.size())) ==
        (index2 < static_cast<int>(cone_map_global_->left_cones.size()))) {
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
    double local_y = translated_x * sin(-yaw) + translated_y * cos(-yaw);

    double distance = sqrt(pow(local_x, 2) + pow(local_y, 2));

    if (local_x < 0 || distance > 15) {
      continue;
    }

    midpoints.push_back(midpoint);
    int midpointIndex = midpoints.size() - 1;
    midpoint_index_to_cone_indices.push_back(std::make_pair(index1, index2));
    cone_index_to_midpoint_indices[index1].push_back(midpointIndex);
    cone_index_to_midpoint_indices[index2].push_back(midpointIndex);
  }

  visualization_msgs::msg::Marker midpoint_viz;
  midpoint_viz.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  midpoint_viz.header.stamp = this->get_clock()->now();
  midpoint_viz.header.frame_id = "ground";
  midpoint_viz.scale.x = 0.1;
  midpoint_viz.scale.y = 0.1;
  midpoint_viz.scale.z = 0.1;
  midpoint_viz.color.a = 1.0;
  midpoint_viz.color.r = 1.0;
  midpoint_viz.color.g = 0.0;
  midpoint_viz.color.b = 0.0;

  geometry_msgs::msg::Point point;
  double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);
  for (int i = 0; i < static_cast<int>(midpoints.size()); i++) {
    double global_x = midpoints[i].x();
    double global_y = midpoints[i].y();
    double translated_x = global_x - ego_state_->pose.pose.position.x;
    double translated_y = global_y - ego_state_->pose.pose.position.y;
    double local_x = translated_x * cos(-yaw) - translated_y * sin(-yaw);
    double local_y = translated_x * sin(-yaw) + translated_y * cos(-yaw);

    point.x = local_x;
    point.y = local_y;
    point.z = 0.0;
    midpoint_viz.points.push_back(point);
  }
  delauny_midpoint_path_publisher_->publish(midpoint_viz);

  std::vector<int> midpoint_indices_by_dist(midpoints.size());
  std::iota(midpoint_indices_by_dist.begin(), midpoint_indices_by_dist.end(),
            0);

  double car_tip_x =
      ego_state_->pose.pose.position.x +
      1.58 / 2.0 * cos(util::quaternionToYaw(ego_state_->pose.pose.orientation));
  double car_tip_y =
      ego_state_->pose.pose.position.y +
      1.58 / 2.0 * sin(util::quaternionToYaw(ego_state_->pose.pose.orientation));

  std::sort(midpoint_indices_by_dist.begin(), midpoint_indices_by_dist.end(),
            [&midpoints, &car_tip_x, &car_tip_y](int a, int b) {
              double distA = util::euclidianDistance2D(
                  midpoints[a].x(), car_tip_x, midpoints[a].y(), car_tip_y);
              double distB = util::euclidianDistance2D(
                  midpoints[b].x(), car_tip_x, midpoints[b].y(), car_tip_y);
              return distA < distB;
            });

  std::vector<std::vector<int>> all_paths;
  std::deque<std::vector<int>> q;

  for (int i = 0; i < std::min(4, static_cast<int>(midpoints.size())); ++i) {
    q.push_back({midpoint_indices_by_dist[i]});
  }

  for (int i = 0; i < 8; ++i) {
    std::vector<std::vector<int>> new_paths;
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
          new_paths.push_back(newPath);
        }
      }
    }

    std::sort(new_paths.begin(), new_paths.end(),
              [&](const std::vector<int> &a, const std::vector<int> &b) {
                return midpointCostFunction(a, midpoints, points,
                                            cone_map_global_->right_cones,
                                            cone_map_global_->left_cones,
                                            midpoint_index_to_cone_indices) <
                       midpointCostFunction(b, midpoints, points,
                                            cone_map_global_->right_cones,
                                            cone_map_global_->left_cones,
                                            midpoint_index_to_cone_indices);
              });

    for (int j = 0; j < std::min(4, static_cast<int>(new_paths.size())); ++j) {
      q.push_back(new_paths[j]);
    }
    all_paths.insert(all_paths.end(), new_paths.begin(), new_paths.end());
  }

  std::sort(all_paths.begin(), all_paths.end(),
            [&](const std::vector<int> &a, const std::vector<int> &b) {
              return midpointCostFunction(a, midpoints, points,
                                          cone_map_global_->right_cones,
                                          cone_map_global_->left_cones,
                                          midpoint_index_to_cone_indices) <
                     midpointCostFunction(
                         b, midpoints, points, cone_map_global_->right_cones,
                         cone_map_global_->left_cones, midpoint_index_to_cone_indices);
            });

  std::vector<Point> best_path_points;
  if (!all_paths.empty()) {
    const std::vector<int> &best_path_indices = all_paths.front();
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
    }
  }

  std::tuple<std::vector<Point>, std::vector<double>, std::vector<double>>
      result = BezierPoints(best_path_points);
  std::vector<Point> bezier_curve = std::get<0>(result);
  std::vector<double> xoft = std::get<1>(result);
  std::vector<double> yoft = std::get<2>(result);

  // Bezier Curve Drawing
  geometry_msgs::msg::PolygonStamped delaunay_midpoints_stamped;
  geometry_msgs::msg::PolygonStamped first_midpoint_stamped;

  delaunay_midpoints_stamped.header.frame_id = "ground";
  delaunay_midpoints_stamped.header.stamp = this->get_clock()->now();
  first_midpoint_stamped.header.frame_id = "ground";
  first_midpoint_stamped.header.stamp = this->get_clock()->now();
  geometry_msgs::msg::Point32 midpoint_pt32;

  for (int i = 0; i < static_cast<int>(bezier_curve.size()); i++) {
    midpoint_pt32.x = bezier_curve[i].x();
    midpoint_pt32.y = bezier_curve[i].y();
    midpoint_pt32.z = 0.0;

    delaunay_midpoints_stamped.polygon.points.push_back(midpoint_pt32);
  }

  for (int i = static_cast<int>(bezier_curve.size()) - 1; i > 0; i--) {
    midpoint_pt32.x = bezier_curve[i].x();
    midpoint_pt32.y = bezier_curve[i].y();
    midpoint_pt32.z = 0.0;

    delaunay_midpoints_stamped.polygon.points.push_back(midpoint_pt32);
  }
  
  delaunay_path_publisher_->publish(delaunay_midpoints_stamped);
  
  utfr_msgs::msg::ParametricSpline path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "ground";
  path.x_params = xoft;
  path.y_params = yoft;
  path.lap_count = curr_sector_;
  return path;
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


void CenterPathNode::trackdriveLapCounter() {
  rclcpp::Time curr_time = this->get_clock()->now();
  double time_diff = (curr_time - last_time).seconds();

  int large_orange_cones_size = cone_detections_->large_orange_cones.size();

  if (time_diff > 20.0) {
    if (large_orange_cones_size > 0) {
      average_distance_to_cones_ = 0.0;
      for (int i = 0; i < static_cast<int>(large_orange_cones_size); i++) {
        average_distance_to_cones_ += util::euclidianDistance2D(
            cone_detections_->large_orange_cones[i].pos.x, 0.0,
            cone_detections_->large_orange_cones[i].pos.y, 0.0);
      }

      average_distance_to_cones_ =
          average_distance_to_cones_ / large_orange_cones_size;

    }
    if (large_orange_cones_size >= 4) {
      found_4_large_orange = true;
    }

    if (loop_closed_) {
      if (!lock_sector_ && checkPassedDatum(getTrackDriveDatum(*cone_map_global_), *ego_state_)) {
        last_time = curr_time;
        curr_sector_ += 1;
        lock_sector_ = true;
        found_4_large_orange = false;

        RCLCPP_INFO(this->get_logger(), "Lap incremented: Global trigger");
        RCLCPP_INFO(this->get_logger(), "Sector: %d", curr_sector_);
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Lock sector: %d", lock_sector_);
      RCLCPP_INFO(this->get_logger(), "Found 4 large orange: %d", found_4_large_orange);
      RCLCPP_INFO(this->get_logger(), "Large orange cones size: %d", large_orange_cones_size);
      RCLCPP_INFO(this->get_logger(), "Average distance to cones: %f", average_distance_to_cones_);

      if (!lock_sector_ && found_4_large_orange &&
          large_orange_cones_size == 0
          && average_distance_to_cones_ < 4.0) {
        last_time = curr_time;
        curr_sector_ += 1;
        lock_sector_ = true;
        found_4_large_orange = false;
        RCLCPP_INFO(this->get_logger(), "Lap incremented: Local trigger");
      }
    }
  }
  

  if (found_4_large_orange && lock_sector_ && large_orange_cones_size == 0 &&
      time_diff > 10.0) {
    lock_sector_ = false;
    found_4_large_orange = false;
  }
  // RCLCPP_INFO(this->get_logger(), "Sector: %d", curr_sector_);
}

utfr_msgs::msg::EgoState CenterPathNode::getTrackDriveDatum(const utfr_msgs::msg::ConeMap &cone_map) {
  utfr_msgs::msg::EgoState datum;

  if (cone_map.large_orange_cones.size() == 1) {
    datum.pose.pose.position.x = cone_map.large_orange_cones[0].pos.x;
    datum.pose.pose.position.y = cone_map.large_orange_cones[0].pos.y;
    datum.pose.pose.orientation = util::yawToQuaternion(0.0);
  } else if (cone_map.large_orange_cones.size() >= 2) {
    double x = (cone_map.large_orange_cones[0].pos.x + cone_map.large_orange_cones[1].pos.x) / 2.0;
    double y = (cone_map.large_orange_cones[0].pos.y + cone_map.large_orange_cones[1].pos.y) / 2.0;
    datum.pose.pose.position.x = x;
    datum.pose.pose.position.y = y;
    datum.pose.pose.orientation = util::yawToQuaternion(0.0);
  } else {
    datum.pose.pose.position.x = -100.0;
    datum.pose.pose.position.y = -100.0;
    datum.pose.pose.position.z = -100.0;
    datum.pose.pose.orientation = util::yawToQuaternion(0.0);
  }

  // visualize to lap_datum_publisher_ as rviz marker
  /*
  visualization_msgs::msg::Marker datum_marker;
  datum_marker.header.frame_id = "ground";
  datum_marker.header.stamp = this->get_clock()->now();
  datum_marker.ns = "datum";  datum_marker.id = 0;  datum_marker.type = visualization_msgs::msg::Marker::SPHERE;
  datum_marker.action = visualization_msgs::msg::Marker::ADD;

  double translated_x = datum.pose.pose.position.x - ego_state_->pose.pose.position.x;
  double translated_y = datum.pose.pose.position.y - ego_state_->pose.pose.position.y;
  double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);
  datum_marker.pose.position.x = translated_x * cos(-yaw) - translated_y * sin(-yaw);  datum_marker.pose.position.y = -(translated_x * sin(-yaw) + translated_y * cos(-yaw));  datum_marker.pose.position.z = 0.0;
  
  datum_marker.scale.x = 0.5;  datum_marker.scale.y = 0.5;  datum_marker.scale.z = 0.1;
  datum_marker.color.a = 1.0;  datum_marker.color.r = 0.0;  datum_marker.color.g = 0.0;  datum_marker.color.b = 1.0;

  lap_datum_publisher_->publish(datum_marker);
  */

  return datum;
}

} // namespace center_path
} // namespace utfr_dv