/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: cone_map_display.cpp
* auth: Kelvin Cui, Justin Lim
* desc: Cone map rviz plugin display class
*/
#include "utfr_rviz_plugins/displays/cone_map/cone_map_display.hpp"

#include <Eigen/Eigen>

namespace utfr_rviz_plugins {
namespace displays {

ConeMapDisplay::ConeMapDisplay()
    : rviz_common::RosTopicDisplay<utfr_msgs::msg::ConeMap>(), id_(0),
      marker_common_(
          std::make_unique<rviz_default_plugins::displays::MarkerCommon>(
              this)) {
  color_option_property_ = new rviz_common::properties::EnumProperty(
      "Color Display", "Cone", "Cone colour to use", this,
      SLOT(updateColorOption()));
  color_option_property_->addOption("Cone", ConeColorOption::CONE);
  color_option_property_->addOption("Flat", ConeColorOption::FLAT);

  color_property_ = new rviz_common::properties::ColorProperty(
      "Color", QColor(200, 200, 200), "Color of cones to display", this);
  color_property_->hide();

  orientation_property_ = new rviz_common::properties::BoolProperty(
      "Flip Orientation", false, "Flip the Vertical Orientation of the Cones",
      this, SLOT(updateOrientation()));
}

void ConeMapDisplay::updateColorOption() {
  ConeColorOption color_option =
      static_cast<ConeColorOption>(color_option_property_->getOptionInt());
  cone_color_option_ = color_option;
  switch (color_option) {
  case CONE:
    color_property_->hide();
    break;
  case FLAT:
    color_property_->show();
    break;
  }
}

void ConeMapDisplay::updateOrientation() {
  if (!orientation_property_->getBool()) {
    z_orientation_ = -1.0;
  } else {
    z_orientation_ = 1.0;
  }

  // Re-init markers:
  initMarkers();
}

void ConeMapDisplay::onInitialize() {
  RTDClass::onInitialize();
  marker_common_->initialize(context_, scene_node_);

  topic_property_->setValue("/mapping/cone_map");
  topic_property_->setDescription(
      "utfr_msgs::msg::ConeMap topic to subscribe to.");

  initMarkers();
}

void ConeMapDisplay::load(const rviz_common::Config &config) {
  Display::load(config);
  marker_common_->load(config);
}

void ConeMapDisplay::processMessage(
    utfr_msgs::msg::ConeMap::ConstSharedPtr msg) {
  delete_all_marker_.header = msg->header;
  delete_all_marker_.id = id_;
  marker_array_.markers.push_back(delete_all_marker_);

  marker_common_->addMessage(
      std::make_shared<visualization_msgs::msg::MarkerArray>(marker_array_));

  marker_array_.markers.clear();

  setMarkerArray(msg);
  marker_common_->addMessage(
      std::make_shared<visualization_msgs::msg::MarkerArray>(marker_array_));

  marker_array_.markers.clear();
}

void ConeMapDisplay::update(float wall_dt, float ros_dt) {
  marker_common_->update(wall_dt, ros_dt);
}

void ConeMapDisplay::reset() {
  RosTopicDisplay::reset();
  marker_common_->clearMarkers();
}

void ConeMapDisplay::initMarkers() {
  delete_all_marker_.action = visualization_msgs::msg::Marker::DELETEALL;

  blue_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
  blue_cone_marker_.type = visualization_msgs::msg::Marker::CUBE;
  blue_cone_marker_.pose.orientation.x = 0.0;
  blue_cone_marker_.pose.orientation.y = 0.0;
  blue_cone_marker_.pose.orientation.z = 0.0;
  blue_cone_marker_.pose.orientation.w = 1.0;
  blue_cone_marker_.scale.x = 0.25;
  blue_cone_marker_.scale.y = 0.25;
  blue_cone_marker_.scale.z = 0.25;
  blue_cone_marker_.color.r = 0.0;
  blue_cone_marker_.color.g = 0.0;
  blue_cone_marker_.color.b = 1.0;
  blue_cone_marker_.color.a = 1.0;
  blue_cone_marker_.ns = "cone";

  yellow_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
  yellow_cone_marker_.type = visualization_msgs::msg::Marker::CUBE;
  yellow_cone_marker_.pose.orientation.x = 0.0;
  yellow_cone_marker_.pose.orientation.y = 0.0;
  yellow_cone_marker_.pose.orientation.z = 0.0;
  yellow_cone_marker_.pose.orientation.w = 1.0;
  yellow_cone_marker_.scale.x = 0.25;
  yellow_cone_marker_.scale.y = 0.25;
  yellow_cone_marker_.scale.z = 0.25;
  yellow_cone_marker_.color.r = 1.0;
  yellow_cone_marker_.color.g = 1.0;
  yellow_cone_marker_.color.b = 0.0;
  yellow_cone_marker_.color.a = 1.0;
  yellow_cone_marker_.ns = "cone";

  orange_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
  orange_cone_marker_.type = visualization_msgs::msg::Marker::CUBE;
  orange_cone_marker_.pose.orientation.x = 0.0;
  orange_cone_marker_.pose.orientation.y = 0.0;
  orange_cone_marker_.pose.orientation.z = 0.0;
  orange_cone_marker_.pose.orientation.w = 1.0;
  orange_cone_marker_.scale.x = 0.25;
  orange_cone_marker_.scale.y = 0.25;
  orange_cone_marker_.scale.z = 0.25;
  orange_cone_marker_.color.r = 1.0;
  orange_cone_marker_.color.g = 0.549;
  orange_cone_marker_.color.b = 0.0;
  orange_cone_marker_.color.a = 1.0;
  orange_cone_marker_.ns = "cone";

  big_orange_cone_marker_.action = visualization_msgs::msg::Marker::ADD;
  big_orange_cone_marker_.type = visualization_msgs::msg::Marker::CUBE;
  big_orange_cone_marker_.pose.orientation.x = 0.0;
  big_orange_cone_marker_.pose.orientation.y = 0.0;
  big_orange_cone_marker_.pose.orientation.z = 0.0;
  big_orange_cone_marker_.pose.orientation.w = 1.0;

  big_orange_cone_marker_.scale.x = 0.25;
  big_orange_cone_marker_.scale.y = 0.25;
  big_orange_cone_marker_.scale.z = 0.25;
  big_orange_cone_marker_.color.r = 1.0;
  big_orange_cone_marker_.color.g = 0.549;
  big_orange_cone_marker_.color.b = 0.0;
  big_orange_cone_marker_.color.a = 1.0;
  big_orange_cone_marker_.ns = "cone";

  covariance_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  covariance_marker_.pose.orientation.x = 0.0;
  covariance_marker_.pose.orientation.y = 0.0;
  covariance_marker_.pose.orientation.z = 0.0;
  covariance_marker_.pose.orientation.w = 1.0;
  covariance_marker_.scale.x = 1.0;
  covariance_marker_.scale.y = 1.0;
  covariance_marker_.scale.z = 1.0;
  covariance_marker_.color.r = 1.0;
  covariance_marker_.color.g = 0.271;
  covariance_marker_.color.b = 0.271;
  covariance_marker_.color.a = 0.7;
  covariance_marker_.ns = "covariance";
}

void ConeMapDisplay::setConeMarker(const utfr_msgs::msg::Cone &cone,
                                   const std_msgs::msg::Header &header,
                                   const int &id,
                                   visualization_msgs::msg::Marker *marker) {
  // Assuming input is in FRD frame
  marker->id = id;
  marker->header = header;
  marker->pose.position.x = cone.pos.x;
  marker->pose.position.y = -cone.pos.y;
  marker->pose.position.z = 0;
}

void ConeMapDisplay::setCovarianceMarker(const utfr_msgs::msg::Cone &cone,
                                         const std_msgs::msg::Header &header,
                                         const int &id) {
  // https://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
  covariance_marker_.id = id;
  covariance_marker_.header = header;
  covariance_marker_.pose.position.x = cone.pos.x;
  covariance_marker_.pose.position.y = -cone.pos.y;
  covariance_marker_.pose.position.z = -cone.pos.z;

  // Convert the covariance message to a matrix
  Eigen::Matrix2d cov_matrix;
  cov_matrix << static_cast<float>(0.0), static_cast<float>(0.0),
      static_cast<float>(0.0), static_cast<float>(0.0);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov_matrix);
  const Eigen::Vector2d &eigValues(eig.eigenvalues());
  const Eigen::Matrix2d &eigVectors(eig.eigenvectors());
  double x_scale = 2 * std::sqrt(5.991 * eigValues[0]);
  double y_scale = 2 * std::sqrt(5.991 * eigValues[1]);
  covariance_marker_.scale.x = x_scale;
  covariance_marker_.scale.y = y_scale;
  covariance_marker_.scale.z = 0.01;
  // Angle between x-axis and first eigenvector
  double angle = std::atan2(eigVectors(1, 0), eigVectors(0, 0));
  // Rotate ellipse such that x-axis is aligned with first eignevector
  // See:
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Intuition
  covariance_marker_.pose.orientation.x = 0;
  covariance_marker_.pose.orientation.y = 0;
  covariance_marker_.pose.orientation.z = std::sin(angle * 0.5);
  covariance_marker_.pose.orientation.w = std::cos(angle * 0.5);
}

visualization_msgs::msg::Marker
ConeMapDisplay::getColoredMarker(visualization_msgs::msg::Marker cone_marker) {
  visualization_msgs::msg::Marker marker = cone_marker;
  switch (cone_color_option_) {
  case FLAT: {
    QColor color = color_property_->getColor();
    marker.color.r = static_cast<float>(color.red()) / 255.0f;
    marker.color.g = static_cast<float>(color.green()) / 255.0f;
    marker.color.b = static_cast<float>(color.blue()) / 255.0f;
  }
  default:
    break;
  }
  return marker;
}

void ConeMapDisplay::setMarkerArray(
    const utfr_msgs::msg::ConeMap::ConstSharedPtr &msg) {
  for (const auto &cone : msg->left_cones) {
    setConeMarker(cone, msg->header, id_, &blue_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    auto marker = getColoredMarker(blue_cone_marker_);
    marker.pose.position.y = -marker.pose.position.y;
    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
  for (const auto &cone : msg->right_cones) {
    setConeMarker(cone, msg->header, id_, &yellow_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    auto marker = getColoredMarker(yellow_cone_marker_);
    marker.pose.position.y = -marker.pose.position.y;
    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
  for (const auto &cone : msg->small_orange_cones) {
    setConeMarker(cone, msg->header, id_, &orange_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    auto marker = getColoredMarker(orange_cone_marker_);
    marker.pose.position.y = -marker.pose.position.y;
    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
  for (const auto &cone : msg->large_orange_cones) {
    setConeMarker(cone, msg->header, id_, &big_orange_cone_marker_);
    setCovarianceMarker(cone, msg->header, id_);
    auto marker = getColoredMarker(big_orange_cone_marker_);
    marker.pose.position.y = -marker.pose.position.y;
    marker_array_.markers.push_back(marker);
    marker_array_.markers.push_back(covariance_marker_);
    id_++;
  }
}

} // namespace displays
} // namespace utfr_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(utfr_rviz_plugins::displays::ConeMapDisplay,
                       rviz_common::Display)
