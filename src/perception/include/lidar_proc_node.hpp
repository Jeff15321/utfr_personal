/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: lidar_proc_node.hpp
* auth: Kareem Elsawah
* desc: lidar processing node header
*/

#pragma once

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// System Requirements
#include <chrono>
#include <fstream>
#include <functional>
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include <clusterer.hpp>
#include <cone_filter.hpp>
#include <filter.hpp>

// Message Requirements
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <utfr_msgs/msg/cone_detections.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/parametric_spline.hpp>
#include <utfr_msgs/msg/system_status.hpp>
#include <utfr_msgs/msg/target_state.hpp>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace lidar_proc {

class LidarProcNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, initPublishers and initTimers.
   */
  LidarProcNode();

private:
  /*! Initialize and load params from config.yaml:
   */
  void initParams();

  /*! Initialize Subscribers:
   */
  void initSubscribers();

  /*! Initialize Publishers:
   */
  void initPublishers();

  /*! Initialize Timers:
   */
  void initTimers();

  /*! Setup Heartbeat message with appropriate module name and update rate.
   */
  void initHeartbeat();

  /*! Send Heartbeat on every timer loop.
   *
   *  @param[in] status current module status, using Heartbeat status enum.
   */
  void publishHeartbeat(const int status);

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input);

  void leftImageCB(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  void rightImageCB(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  void egoStateCB(const utfr_msgs::msg::EgoState::SharedPtr msg);

  sensor_msgs::msg::PointCloud2::SharedPtr
  convertToPointCloud2(const std::vector<std::array<float, 3>> &points,
                       const std::string &frame_id);

  void publishPointCloud(
      const PointCloud &cloud,
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);

  PointCloud filterAndConvertToCustomPointCloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr &input);

  void timerCB();

  /*! Initialize global variables:
   */
  double update_rate_;
  bool debug_;
  Filter filter;
  Clusterer clusterer;
  ConeLRFilter cone_filter;
  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Time ros_time_;
  utfr_msgs::msg::TargetState target_;
  utfr_msgs::msg::SystemStatus::SharedPtr status_{nullptr};
  utfr_msgs::msg::Heartbeat heartbeat_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_point_cloud;

  // TF2 buffer and listener for transform handling
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_subscriber_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_no_ground;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clustered;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_lidar_detected;
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      left_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      right_image_publisher;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      left_image_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      right_image_subscriber;
  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      ego_state_subscriber;
  rclcpp::Publisher<utfr_msgs::msg::EgoState>::SharedPtr ego_state_publisher;

  bool hold_image = true;
  sensor_msgs::msg::CompressedImage left_img;
  sensor_msgs::msg::CompressedImage right_img;
  utfr_msgs::msg::EgoState ego_state;
};
} // namespace lidar_proc
} // namespace utfr_dv
