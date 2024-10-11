/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: lidar_proc_node.cpp
* auth: Kareem Elsawah
* desc: lidar processing node class
*/

#include <filter.hpp>
#include <lidar_proc_node.hpp>

namespace utfr_dv {
namespace lidar_proc {

LidarProcNode::LidarProcNode()
    : Node("lidar_proc_node"), tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
  this->initParams();
  this->initHeartbeat();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::NOT_READY);
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  publishHeartbeat(utfr_msgs::msg::Heartbeat::READY);
}

void LidarProcNode::initParams() {
  this->declare_parameter("update_rate", 33.33);
  this->declare_parameter("debug", false);
  this->declare_parameter("topics.input", rclcpp::PARAMETER_STRING);
  this->declare_parameter("view_filter.outer_box.xmin",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("view_filter.outer_box.xmax",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("view_filter.outer_box.ymin",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("view_filter.outer_box.ymax",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("view_filter.inner_box.xmin",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("view_filter.inner_box.xmax",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("view_filter.inner_box.ymin",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("view_filter.inner_box.ymax",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("view_filter.grid_size_x", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("view_filter.grid_size_y", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("ground_ransac.max_iterations",
                          rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("ground_ransac.distance_threshold",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("clustering.cluster_tolerance",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("clustering.min_cluster_size",
                          rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("clustering.max_cluster_size",
                          rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("clustering.reconstruct_radius",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("cone_filter.cone_radius", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("cone_filter.cone_height", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("cone_filter.big_cone_radius",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("cone_filter.big_cone_height",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("cone_filter.mse_threshold",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("cone_filter.lin_threshold",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("cone_filter.IOUThreshold", rclcpp::PARAMETER_DOUBLE);

  update_rate_ = this->get_parameter("update_rate").as_double();
  debug_ = this->get_parameter("debug").as_bool();
  double outer_xmin =
      this->get_parameter("view_filter.outer_box.xmin").as_double();
  double outer_xmax =
      this->get_parameter("view_filter.outer_box.xmax").as_double();
  double outer_ymin =
      this->get_parameter("view_filter.outer_box.ymin").as_double();
  double outer_ymax =
      this->get_parameter("view_filter.outer_box.ymax").as_double();
  double inner_xmin =
      this->get_parameter("view_filter.inner_box.xmin").as_double();
  double inner_xmax =
      this->get_parameter("view_filter.inner_box.xmax").as_double();
  double inner_ymin =
      this->get_parameter("view_filter.inner_box.ymin").as_double();
  double inner_ymax =
      this->get_parameter("view_filter.inner_box.ymax").as_double();
  ViewBounds bounds = {{outer_xmin, outer_xmax, outer_ymin, outer_ymax},
                       {inner_xmin, inner_xmax, inner_ymin, inner_ymax}};

  int grid_size_x = this->get_parameter("view_filter.grid_size_x").as_int();
  int grid_size_y = this->get_parameter("view_filter.grid_size_y").as_int();
  filter = Filter(bounds, grid_size_x, grid_size_y);

  int groundRansacMaxIterations =
      this->get_parameter("ground_ransac.max_iterations").as_int();
  double groundRansacDistanceThreshold =
      this->get_parameter("ground_ransac.distance_threshold").as_double();
  RansacParams ransac_params = {groundRansacMaxIterations,
                                groundRansacDistanceThreshold};

  double cluster_tolerace =
      this->get_parameter("clustering.cluster_tolerance").as_double();
  int min_cluster_size =
      this->get_parameter("clustering.min_cluster_size").as_int();
  int max_cluster_size =
      this->get_parameter("clustering.max_cluster_size").as_int();
  double reconstruct_radius =
      this->get_parameter("clustering.reconstruct_radius").as_double();

  ClusterParams cluster_params = {cluster_tolerace, min_cluster_size,
                                  max_cluster_size};
  clusterer = Clusterer(ransac_params, cluster_params, reconstruct_radius);

  double cone_height =
      this->get_parameter("cone_filter.cone_height").as_double();
  double cone_radius =
      this->get_parameter("cone_filter.cone_radius").as_double();
  double big_cone_height =
      this->get_parameter("cone_filter.big_cone_height").as_double();
  double big_cone_radius =
      this->get_parameter("cone_filter.big_cone_radius").as_double();
  double mse_threshold =
      this->get_parameter("cone_filter.mse_threshold").as_double();
  double lin_threshold =
      this->get_parameter("cone_filter.lin_threshold").as_double();
  double IOUThreshold =
      this->get_parameter("cone_filter.IOUThreshold").as_double();
  ConeLRFilterParams cone_filter_params = {
      cone_height,   cone_radius,   big_cone_height, big_cone_radius,
      mse_threshold, lin_threshold, IOUThreshold};

  cone_filter = ConeLRFilter(cone_filter_params);
}

void LidarProcNode::initSubscribers() {
  point_cloud_subscriber_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/ouster/points",
          rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
          std::bind(&LidarProcNode::pointCloudCallback, this, _1));

  left_image_subscriber =
      this->create_subscription<sensor_msgs::msg::CompressedImage>(
          "/left_camera_node/images/compressed", 10,
          std::bind(&LidarProcNode::leftImageCB, this, _1));
  right_image_subscriber =
      this->create_subscription<sensor_msgs::msg::CompressedImage>(
          "/right_camera_node/images/compressed", 10,
          std::bind(&LidarProcNode::rightImageCB, this, _1));
  ego_state_subscriber = this->create_subscription<utfr_msgs::msg::EgoState>(
      topics::kEgoState, 10, std::bind(&LidarProcNode::egoStateCB, this, _1));
}

void LidarProcNode::initPublishers() {
  pub_filtered = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      topics::kFiltered, 10);
  pub_no_ground = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      topics::kNoGround, 10);
  pub_clustered = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      topics::kClustered, 10);
  pub_lidar_detected = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      topics::kDetected, 10);
  left_image_publisher =
      this->create_publisher<sensor_msgs::msg::CompressedImage>(
          "synced_left_image", 10);
  right_image_publisher =
      this->create_publisher<sensor_msgs::msg::CompressedImage>(
          "synced_right_image", 10);
  ego_state_publisher =
      this->create_publisher<utfr_msgs::msg::EgoState>("synced_ego_state", 10);
}

void LidarProcNode::initTimers() {
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(this->update_rate_),
      std::bind(&LidarProcNode::timerCB, this));

  RCLCPP_INFO(this->get_logger(), "Finished Initializing Timer");
}

void LidarProcNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kLidarProcHeartbeat, 10);
  heartbeat_.module.data = "lidar_proc_node";
  heartbeat_.update_rate = update_rate_;
}

void LidarProcNode::publishHeartbeat(const int status) {
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_publisher_->publish(heartbeat_);
}

void LidarProcNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {
  hold_image = true;

  std::cout << "\nBegin point cloud processing" << std::endl;
  int startTime;
  int initialStartTime = this->get_clock()->now().nanoseconds() / 1000000; // ms

  PointCloud filtered_cloud = filterAndConvertToCustomPointCloud(point_cloud);

  if (debug_) {
    std::cout << "filter and conversion to custom point cloud: ms"
              << int(this->get_clock()->now().nanoseconds() / 1000000 -
                     initialStartTime)
              << std::endl;
    startTime = this->get_clock()->now().nanoseconds() / 1000000; // ms
  }

  // Filtering
  Grid min_points_grid = filter.ground_grid(filtered_cloud);

  if (debug_) {
    std::cout << "make ground grid: ms"
              << int(this->get_clock()->now().nanoseconds() / 1000000 -
                     startTime)
              << std::endl;
    startTime = this->get_clock()->now().nanoseconds() / 1000000; // ms
    publishPointCloud(filtered_cloud, pub_filtered);
    std::cout << "DEBUG ENABLED. PUBLISHING POINT CLOUD TAKES 20-30 MS!"
              << std::endl;
  }

  // Clustering and reconstruction
  std::tuple<PointCloud, std::vector<PointCloud>> cluster_results =
      clusterer.clean_and_cluster(filtered_cloud, min_points_grid);

  if (debug_) {
    std::cout << "clean and cluster: ms"
              << int(this->get_clock()->now().nanoseconds() / 1000000 -
                     startTime)
              << std::endl;
    startTime = this->get_clock()->now().nanoseconds() / 1000000; // ms
  }

  PointCloud no_ground_cloud = std::get<0>(cluster_results);

  if (debug_) {
    std::cout << "no ground cloud: ms"
              << int(this->get_clock()->now().nanoseconds() / 1000000 -
                     startTime)
              << std::endl;
    startTime = this->get_clock()->now().nanoseconds() / 1000000; // ms
  }

  std::vector<PointCloud> reconstructed_clusters = std::get<1>(cluster_results);

  if (debug_) {
    std::cout << "reconstructed clusters: ms"
              << int(this->get_clock()->now().nanoseconds() / 1000000 -
                     startTime)
              << std::endl;
    startTime = this->get_clock()->now().nanoseconds() / 1000000; // ms
    publishPointCloud(no_ground_cloud, pub_no_ground);
  }

  // // Publishing each reconstructed cluster in a loop

  PointCloud combined_clusters;
  for (int i = 0; i < reconstructed_clusters.size(); i++) {
    for (int j = 0; j < reconstructed_clusters[i].size(); j++) {
      combined_clusters.push_back(reconstructed_clusters[i][j]);
    }
  }

  if (debug_) {
    std::cout << "combined_clusters clusters: ms"
              << int(this->get_clock()->now().nanoseconds() / 1000000 -
                     startTime)
              << std::endl;
    startTime = this->get_clock()->now().nanoseconds() / 1000000; // ms
    publishPointCloud(combined_clusters, pub_clustered);
  }

  // Gather the cluster centers into a single point cloud
  PointCloud cluster_centers =
      cone_filter.filter_clusters(reconstructed_clusters);

  if (debug_) {
    std::cout << "filter clusters: ms"
              << int(this->get_clock()->now().nanoseconds() / 1000000 -
                     startTime)
              << std::endl;
    startTime = this->get_clock()->now().nanoseconds() / 1000000; // ms
  }

  if (cluster_centers.size() > 0) {
    publishPointCloud(cluster_centers, pub_lidar_detected);
  }

  if (debug_) {
    std::cout << "cluster centers: ms"
              << int(this->get_clock()->now().nanoseconds() / 1000000 -
                     startTime)
              << std::endl;
  }

  std::cout << "lidar proc time: ms"
            << int(this->get_clock()->now().nanoseconds() / 1000000 -
                   initialStartTime)
            << std::endl;

  left_image_publisher->publish(left_img);
  right_image_publisher->publish(right_img);
  ego_state_publisher->publish(ego_state);

  hold_image = false;

  RCLCPP_INFO(this->get_logger(), "Published Processed Point Clouds");
}

void LidarProcNode::leftImageCB(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  if (!hold_image) {
    left_img.header = msg->header;

    left_img.format = msg->format;

    left_img.data = msg->data;
  }
}

void LidarProcNode::rightImageCB(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  if (!hold_image) {
    right_img.header = msg->header;

    right_img.format = msg->format;

    right_img.data = msg->data;
  }
}

void LidarProcNode::egoStateCB(const utfr_msgs::msg::EgoState::SharedPtr msg) {
  if (!hold_image) {
    ego_state.header = msg->header;

    ego_state.pose = msg->pose;
    ego_state.accel = msg->accel;
    ego_state.vel = msg->vel;
    ego_state.steering_angle = msg->steering_angle;
  }
}

void LidarProcNode::publishPointCloud(
    const PointCloud &cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub) {
  sensor_msgs::msg::PointCloud2::SharedPtr output =
      convertToPointCloud2(cloud, "os_sensor");

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    // Get the transform from the LiDAR frame to the ground frame
    transform_stamped = tf_buffer_.lookupTransform(
        "ground", output->header.frame_id, tf2::TimePointZero);
    // RCLCPP_INFO(this->get_logger(), "Transformed %s to ground frame",
    //             output->header.frame_id.c_str());
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
                "Could not transform %s to ground frame: %s",
                output->header.frame_id.c_str(), ex.what());
    return;
  }

  // Transform the point cloud to the ground frame
  sensor_msgs::msg::PointCloud2 output_tfed;
  tf2::doTransform(*output, output_tfed, transform_stamped);

  pub->publish(output_tfed);
}

PointCloud LidarProcNode::filterAndConvertToCustomPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &input) {
  /* Filter the point cloud to only include points within the specified
  view bounds and convert to custom PointCloud type, which is a vector of (x, y,
  z) coordinates*/

  PointCloud custom_cloud;
  custom_cloud.reserve(10000);

  // Offsets for x, y, z fields in PointCloud2 data
  int x_offset = input->fields[0].offset;
  int y_offset = input->fields[1].offset;
  int z_offset = input->fields[2].offset;
  ViewBounds bound = filter.getBounds();
  Point pt;
  for (uint32_t i = 0; i < input->height * input->width; ++i) {

    pt[0] = *reinterpret_cast<const float *>(
        &input->data[i * input->point_step + x_offset]);
    pt[1] = *reinterpret_cast<const float *>(
        &input->data[i * input->point_step + y_offset]);
    pt[2] = *reinterpret_cast<const float *>(
        &input->data[i * input->point_step + z_offset]);

    // Check if point is within outer box
    if (!(pt[0] >= bound.inner_box.x_min && pt[0] <= bound.inner_box.x_max &&
          pt[1] >= bound.inner_box.y_min && pt[1] <= bound.inner_box.y_max)) {
      // Check if point is outside inner box
      if (pt[0] >= bound.outer_box.x_min && pt[0] <= bound.outer_box.x_max &&
          pt[1] >= bound.outer_box.y_min && pt[1] <= bound.outer_box.y_max) {
        // add to cloud if it's within the view
        custom_cloud.push_back(pt);
      }
    }
  }

  return custom_cloud;
}

sensor_msgs::msg::PointCloud2::SharedPtr LidarProcNode::convertToPointCloud2(
    const std::vector<std::array<float, 3>> &points,
    const std::string &frame_id) {
  sensor_msgs::msg::PointCloud2::SharedPtr cloud =
      std::make_shared<sensor_msgs::msg::PointCloud2>();
  cloud->header.frame_id = frame_id;
  cloud->header.stamp = rclcpp::Clock().now();

  cloud->height = 1; // Unordered point cloud
  cloud->width = points.size();

  cloud->is_bigendian = false;
  cloud->is_dense = true;

  cloud->point_step = 12; // x, y, z each take 4 bytes (float)
  cloud->row_step = cloud->point_step * cloud->width;

  cloud->fields.resize(3);
  cloud->fields[0].name = "x";
  cloud->fields[0].offset = 0;
  cloud->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud->fields[0].count = 1;

  cloud->fields[1].name = "y";
  cloud->fields[1].offset = 4;
  cloud->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud->fields[1].count = 1;

  cloud->fields[2].name = "z";
  cloud->fields[2].offset = 8;
  cloud->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud->fields[2].count = 1;

  cloud->data.resize(cloud->row_step * cloud->height);

  auto float_ptr = reinterpret_cast<float *>(&cloud->data[0]);

  for (const auto &point : points) {
    *float_ptr++ = point[0];
    *float_ptr++ = point[1];
    *float_ptr++ = point[2];
  }

  return cloud;
}

void LidarProcNode::timerCB() {
  const std::string function_name{"timerCB"};
  publishHeartbeat(utfr_msgs::msg::Heartbeat::ACTIVE);
}

} // namespace lidar_proc
} // namespace utfr_dv