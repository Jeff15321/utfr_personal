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
  this->skidpadCircleCentres();
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

void CenterPathNode::getTransform(){
  auto[leftX,leftY,rightX,rightY] = this->skidpadCircleCentres();
  MatrixXd A(2, 3);
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
  
  std::vector<utfr_msgs::msg::Cone> blue = cone_map_->left_cones;
  std::vector<utfr_msgs::msg::Cone> yellow = cone_map_->right_cones;
  /*ONLY FOR TESTING*/ blue.pop_back(); yellow.pop_back(); // pop the extra cone

  std::cout << "Size: " << blue.size() << "," << yellow.size() << std::endl;

  auto smallBlue = this->circleCentre(blue, 15.25/2, 16-3);
  printf("Small Blue Circle: Xc = %lf, Yc = %lf\n", smallBlue.first, smallBlue.second);

  auto smallYellow = this->circleCentre(yellow, 15.25/2, 16-3);
  printf("Small Yellow Circle: Xc = %lf, Yc = %lf\n", smallYellow.first, smallYellow.second);

  auto largeBlue = this->circleCentre(blue, 15.25/2+3, 13-3);
  printf("Large Blue Circle: Xc = %lf, Yc = %lf\n", largeBlue.first, largeBlue.second);
  
  auto largeYellow = this->circleCentre(yellow, 15.25/2+3, 13-3);
  printf("Large Yellow Circle: Xc = %lf, Yc = %lf\n", largeYellow.first, largeYellow.second);
  
  auto drawCircle = [this](auto publisher, auto cord, double radius){
    auto [xc, yc] = cord;
    PolygonStamped circle_stamped;
    circle_stamped.header.frame_id = "base_link";
    circle_stamped.header.stamp = this->get_clock()->now();

    for(int i = 0; i < 360; i++) {
      geometry_msgs::msg::Point32 point;  
      double angle = 2.0 * M_PI * i / 360;
      point.x = xc + radius * cos(angle);
      point.y = yc + radius * sin(angle);
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
  double leftX = (smallBlue.first+largeYellow.first)/2;
  double leftY = (smallBlue.second+largeYellow.second)/2;
  double rightX = (smallYellow.first+largeBlue.first)/2;
  double rightY = (smallYellow.second+largeBlue.second)/2;
  return {leftX,leftY,rightX,rightY};
}

std::pair<double,double> CenterPathNode::circleCentre(std::vector<utfr_msgs::msg::Cone> &cones, double radius, int inlier_count){
  int n = cones.size();

  auto get_threshold = [&](double xc, double yc){
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

  // find the circle with the lowest threshold
  double lowest_threshold = DBL_MAX;
  std::pair<double,double> centre = {NAN,NAN};
  std::vector<utfr_msgs::msg::Cone> ransacCones(3);
  double radf;
  for(int i = 0; i < n; i++){
    ransacCones[0] = cones[i];
    for(int j = i+1; j < n; j++){
      ransacCones[1] = cones[j];
      for(int k = j+1; k < n; k++){
        ransacCones[2] = cones[k];
        auto [xc, yc, _, __] = util::ransacCircleLSF(ransacCones, radius);
        std:: cout << _ << " vs " << __ << std::endl;
        double threshold = get_threshold(xc, yc);
        if(threshold < lowest_threshold){
          lowest_threshold = threshold;
          centre = {xc,yc};
          radf = t;
        }
      }
    }
  }
  return centre;
}

} // namespace center_path
} // namespace utfr_dv
