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


  std::ofstream out("cones.txt");
  for(auto &c : msg.left_cones){
    out << "(" << c.pos.x << "," << c.pos.y << ")" << std::endl;
  }
  out << std::endl;
  for(auto &c : msg.right_cones){
    out << "(" << c.pos.x << "," << c.pos.y << ")" << std::endl;
  }
  this->GlobalWaypoints();
  this->nextWaypoint(*path_);
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

void CenterPathNode::nextWaypoint(std::deque<std::pair<double,double>> &waypoints){
  while(waypoints.size()){
    auto[x, y] = waypoints.front();
    double carX = ego_state_->pose.pose.position.x;
    double carY = ego_state_->pose.pose.position.y;
    double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);
    double localY = (cos(yaw) * (y-carY)) - (sin(yaw) * (x-carX));
    double localX = (sin(yaw) * (y-carY)) + (cos(yaw) * (x-carX));
    double angle = util::wrapDeg(util::radToDeg(atan2(localY, localX)));
    // std::cout << "Waypoint angle: " << angle << std::endl;
    if(util::euclidianDistance2D(x, carX, y, carY) <= 0.4 || (angle >= 90 && angle <= 270)){
      waypoints.pop_front();
    }
    else break;
  }

  using geometry_msgs::msg::PolygonStamped;
  static rclcpp::Publisher<PolygonStamped>::SharedPtr path_pub =
    this->create_publisher<PolygonStamped>("Waypoints", 1);

  PolygonStamped points_stamped;
  points_stamped.header.frame_id = "base_footprint";
  points_stamped.header.stamp = this->get_clock()->now();

  for(auto [x,y] : waypoints) {
    double carX = ego_state_->pose.pose.position.x;
    double carY = ego_state_->pose.pose.position.y;
    double yaw = util::quaternionToYaw(ego_state_->pose.pose.orientation);
    double localY = (cos(yaw) * (y-carY)) - (sin(yaw) * (x-carX));
    double localX = (sin(yaw) * (y-carY)) + (cos(yaw) * (x-carX));
    geometry_msgs::msg::Point32 point;
    point.x = localX;
    point.y = -localY;
    point.z = 0;
    points_stamped.polygon.points.push_back(point);
  }
  path_pub->publish(points_stamped);
  // std::cout << "Next Waypoint Function Ran, " << waypoints.size() << " Points published" << std::endl;
}

void CenterPathNode::GlobalWaypoints(){
  std::vector<std::pair<double,double>> v = this->getWaypoints();
  // std::cout << "Old:" << std::endl;
  // for(auto [x,y] : v) std::cout << x << "," << y << std::endl;
  v = this->transform(v);
  // std::cout << "New:" << std::endl;
  // for(auto [x,y] : v) std::cout << x << "," << y << std::endl;

  using geometry_msgs::msg::PolygonStamped;
  static rclcpp::Publisher<PolygonStamped>::SharedPtr waypoints_pub =
    this->create_publisher<PolygonStamped>("Waypoints", 1);

  PolygonStamped points_stamped;
  points_stamped.header.frame_id = "map";
  points_stamped.header.stamp = this->get_clock()->now();

  for(auto [x,y] : v) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = -y;
    point.z = 0;
    points_stamped.polygon.points.push_back(point);
  }
  waypoints_pub->publish(points_stamped);

  if(!path_)
    path_ = new std::deque<std::pair<double,double>>(v.begin(), v.end());
}

std::vector<std::pair<double,double>> CenterPathNode::getWaypoints(){
  std::ifstream in("src/planning/points/Waypoints.csv");
  std::vector<std::pair<double,double>> v;
  while(!in.eof()){
    double x,y;
    in >> x >> y;
    v.push_back({x,y});
  }
  return v;
}

std::vector<std::pair<double,double>> CenterPathNode::transform(std::vector<std::pair<double,double>> &points){
  std::ifstream referencePoints("src/planning/points/ReferencePoints.csv");
  double xLeft1,yLeft1, xRight1, yRight1;
  referencePoints >> xLeft1 >> yLeft1 >> xRight1 >> yRight1;

  auto [xLeft2,yLeft2,xRight2,yRight2] = this->getCentres();
  if(isnan(xLeft2) || isnan(xLeft2) || isnan(xRight2) || isnan(yRight2)){
    return {};
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

  using namespace std;
  // cout << "Reference: X_intersect = " << xInter1 << ", Y_intersect = " << yInter1 << endl;
  // cout << "New: X_intersect = " << xInter2 << ", Y_intersect = " << yInter2 << endl;
  // cout << "T:" << endl << T << endl;
  
  std::vector<std::pair<double,double>> transformed;
  for(auto &p: points){
    double &x = p.first, &y = p.second;
    VectorXd v(3);
    v << x, y, 1;

    VectorXd out = T*v;
    transformed.push_back({out(0), out(1)});
  }
  return transformed;
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
  if(abs(rightDist-centreDistance_) > 0.25){ // improper right centre
    return {NAN,NAN,NAN,NAN};
  }
  // improper left centre
  if(abs(totalDist-2*centreDistance_) > 0.5 || abs(leftDist-centreDistance_) > 0.25){
    double theta = atan2(yMid-yRight, xMid-xRight);
    double dx = cos(theta) * rightDist, dy = sin(theta) * rightDist;
    xLeft = xMid + dx;
    yLeft = yMid + dy;
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

  auto smallBlue = this->circleCentre(blue, smallRadius_, smallCircleCones_-3);
  auto smallYellow = this->circleCentre(yellow, smallRadius_, smallCircleCones_-3);
  auto largeBlue = this->circleCentre(blue, largeRadius_, largeCircleCones_-3);
  auto largeYellow = this->circleCentre(yellow, largeRadius_, largeCircleCones_-3);
  // printf("Small Blue Circle: Xc = %lf, Yc = %lf\n", smallBlue.first, smallBlue.second);
  // printf("Small Yellow Circle: Xc = %lf, Yc = %lf\n", smallYellow.first, smallYellow.second);
  // printf("Large Blue Circle: Xc = %lf, Yc = %lf\n", largeBlue.first, largeBlue.second);
  // printf("Large Yellow Circle: Xc = %lf, Yc = %lf\n", largeYellow.first, largeYellow.second);
  
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
