/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: cone_detections_display.hpp
* auth: Kelvin Cui
* desc: Cone detection rviz plugin display header
*/
#include <memory>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <rviz_default_plugins/displays/marker/marker_common.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rviz_default_plugins/visibility_control.hpp"
#include "utfr_msgs/msg/cone.hpp"
#include "utfr_msgs/msg/cone_detections.hpp"

#pragma once

namespace utfr_rviz_plugins {
namespace displays {

enum ConeColorOption { CONE = 0, FLAT = 1 };

class ConeDetectionsDisplay
    : public rviz_common::RosTopicDisplay<utfr_msgs::msg::ConeDetections> {
  Q_OBJECT

public:
  ConeDetectionsDisplay();

  void onInitialize() override;

  void load(const rviz_common::Config &config) override;

  void update(float wall_dt, float ros_dt) override;

  void reset() override;

private Q_SLOTS:
  void updateColorOption();

  void updateOrientation();

private:
  void initMarkers();

  void setConeMarker(const utfr_msgs::msg::Cone &cone,
                     const std_msgs::msg::Header &header, const int &id,
                     visualization_msgs::msg::Marker *marker);

  void setCovarianceMarker(const utfr_msgs::msg::Cone &cone,
                           const std_msgs::msg::Header &header, const int &id);

  visualization_msgs::msg::Marker
  getColoredMarker(visualization_msgs::msg::Marker cone_marker);

  void
  setMarkerArray(const utfr_msgs::msg::ConeDetections::ConstSharedPtr &msg);

  void
  processMessage(utfr_msgs::msg::ConeDetections::ConstSharedPtr msg) override;

  int id_;

  double z_orientation_ = 1.0;

  ConeColorOption cone_color_option_;

  std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> marker_common_;

  rviz_common::properties::EnumProperty *color_option_property_;
  rviz_common::properties::ColorProperty *color_property_;
  rviz_common::properties::BoolProperty *orientation_property_;

  visualization_msgs::msg::Marker blue_cone_marker_;

  visualization_msgs::msg::Marker yellow_cone_marker_;

  visualization_msgs::msg::Marker orange_cone_marker_;

  visualization_msgs::msg::Marker big_orange_cone_marker_;

  visualization_msgs::msg::Marker unknown_cone_marker_;

  visualization_msgs::msg::Marker covariance_marker_;

  visualization_msgs::msg::Marker delete_all_marker_;

  visualization_msgs::msg::MarkerArray marker_array_;
};

} // namespace displays
} // namespace utfr_rviz_plugins
