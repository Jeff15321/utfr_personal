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

#include <lidar_proc_node.hpp>

namespace utfr_dv {
namespace center_path {

LidarProcNode::LidarProcNode() : Node("lidar_proc_node") {
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

void LidarProcNode::initParams() {
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
  this->declare_parameter("cone_filter.inlier_mse_threshold",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("cone_filter.ransac_max_iters",
                          rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("cone_filter.min_inlier_ratio",
                          rclcpp::PARAMETER_DOUBLE);

  outer_xmin_ = this->get_parameter("view_filter.outer_box.xmin").as_double();
  outer_xmax_ = this->get_parameter("view_filter.outer_box.xmax").as_double();
  outer_ymin_ = this->get_parameter("view_filter.outer_box.ymin").as_double();
  outer_ymax_ = this->get_parameter("view_filter.outer_box.ymax").as_double();
  inner_xmin_ = this->get_parameter("view_filter.inner_box.xmin").as_double();
  inner_xmax_ = this->get_parameter("view_filter.inner_box.xmax").as_double();
  inner_ymin_ = this->get_parameter("view_filter.inner_box.ymin").as_double();
  inner_ymax_ = this->get_parameter("view_filter.inner_box.ymax").as_double();
  ViewBounds bounds = {{outer_xmin_, outer_xmax_, outer_ymin_, outer_ymax_},
                       {inner_xmin_, inner_xmax_, inner_ymin_, inner_ymax_}};

  grid_size_x_ = this->get_parameter("view_filter.grid_size_x").as_int();
  grid_size_y_ = this->get_parameter("view_filter.grid_size_y").as_int();
  filter = Filter(bounds, grid_size_x, grid_size_y);

  groundRansacMaxIterations_ =
      this->get_parameter("ground_ransac.max_iterations").as_int();
  groundRansacDistanceThreshold_ =
      this->get_parameter("ground_ransac.distance_threshold").as_double();
  ransac_params_ = {groundRansacMaxIterations_, groundRansacDistanceThreshold_};

  cluster_tolerace_ =
      this->get_parameter("clustering.cluster_tolerance").as_double();
  min_cluster_size_ =
      this->get_parameter("clustering.min_cluster_size").as_int();
  max_cluster_size_ =
      this->get_parameter("clustering.max_cluster_size").as_int();
  reconstruct_radius_ =
      this->get_parameter("clustering.reconstruct_radius").as_double();

  ClusterParams cluster_params = {cluster_tolerace_, min_cluster_size_,
                                  max_cluster_size_};
  clusterer = Clusterer(ransac_params_, cluster_params_, reconstruct_radius_);

  cone_height_ = this->get_parameter("cone_filter.cone_height").as_double();
  cone_radius_ = this->get_parameter("cone_filter.cone_radius").as_double();
  inlier_mse_threshold =
      this->get_parameter("cone_filter.inlier_mse_threshold").as_double();
  ransac_max_iters_ =
      this->get_parameter("cone_filter.ransac_max_iters").as_int();
  min_inlier_ratio_ =
      this->get_parameter("cone_filter.min_inlier_ratio").as_double();
  ConeFilterParams cone_filter_params = {cone_height_, cone_radius_,
                                         inlier_mse_threshold_,
                                         ransac_max_iters_, min_inlier_ratio_};

  cone_filter = ConeFilter(cone_filter_params);
}

void LidarProcNode::initSubscribers() {
  ego_state_subscriber_ = this->create_subscription<utfr_msgs::msg::EgoState>(
      topics::kEgoState, 10, std::bind(&LidarProcNode::egoStateCB, this, _1));
}

void LidarProcNode::initPublishers() {
  center_path_publisher_ =
      this->create_publisher<utfr_msgs::msg::ParametricSpline>(
          topics::kCenterPath, 10);

  sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, 10,
      std::bind(&LidarProcessingNode::pointCloudCallback, this,
                std::placeholders::_1));
  pub_filtered = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      topics::kFiltered, 10);
  pub_no_ground = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      topics::kNoGround, 10);
  pub_clustered = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      topics::kClustered, 10);
  pub_clustered_center = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      topics::kClustered, 10);
  pub_heartbeat = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kLidarHeartbeat, 10);
}

void LidarProcNode::initTimers() {
  main_timer_ = this->create_wall_timer(
      std::chrono::duration<double, std::milli>(this->update_rate_),
      std::bind(&LidarProcNode::timerCB, this));

  RCLCPP_INFO(this->get_logger(), "Finished Initializing Timer");
}

void LidarProcNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kMappingComputeHeartbeat, 10);
}

void LidarProcNode::publishHeartbeat(const int status) {
  heartbeat_.status = status;
  heartbeat_.header.stamp = this->get_clock()->now();
  heartbeat_publisher_->publish(heartbeat_);
}

void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input) {
  PointCloud custom_cloud = convertToCustomPointCloud(input);

  // Filtering
  std::tuple<PointCloud, Grid> filtered_cloud_and_grid =
      filter.view_filter(custom_cloud);
  PointCloud filtered_cloud = std::get<0>(filtered_cloud_and_grid);
  Grid min_points_grid = std::get<1>(filtered_cloud_and_grid);
  publishPointCloud(filtered_cloud, pub_filtered);

  // Clustering and reconstruction
  std::tuple<PointCloud, std::vector<PointCloud>> cluster_results =
      clusterer.clean_and_cluster(filtered_cloud, min_points_grid);
  PointCloud no_ground_cloud = std::get<0>(cluster_results);
  std::vector<PointCloud> reconstructed_clusters = std::get<1>(cluster_results);

  publishPointCloud(no_ground_cloud, pub_no_ground);

  // Publishing each reconstructed cluster in a loop

  PointCloud combined_clusters;
  for (int i = 0; i < reconstructed_clusters.size(); i++) {
    for (int j = 0; j < reconstructed_clusters[i].size(); j++) {
      combined_clusters.push_back(reconstructed_clusters[i][j]);
    }
  }
  publishPointCloud(combined_clusters, pub_clustered);

  // Gather the cluster centers into a single point cloud
  PointCloud cluster_centers =
      cone_filter.filter_clusters(reconstructed_clusters);
  if (cluster_centers.size() > 0) {
    publishPointCloud(cluster_centers, pub_clustered_center);
  }
  heartbeat.status = utfr_msgs::msg::Heartbeat::ACTIVE;
  publishHeartbeat();
}

sensor_msgs::msg::PointCloud2
convertToPointCloud2(const std::vector<std::array<float, 3>> &points,
                     const std::string &frame_id) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = rclcpp::Clock().now();

  cloud.height = 1; // Unordered point cloud
  cloud.width = points.size();

  cloud.is_bigendian = false;
  cloud.is_dense = true;

  cloud.point_step = 12; // x, y, z each take 4 bytes (float)
  cloud.row_step = cloud.point_step * cloud.width;

  cloud.fields.resize(3);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;

  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;

  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;

  cloud.data.resize(cloud.row_step * cloud.height);

  auto float_ptr = reinterpret_cast<float *>(&cloud.data[0]);
  for (const auto &point : points) {
    *float_ptr++ = point[0];
    *float_ptr++ = point[1];
    *float_ptr++ = point[2];
  }

  return cloud;
}

void publishPointCloud(
    const PointCloud &cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub) {
  sensor_msgs::msg::PointCloud2 output =
      convertToPointCloud2(cloud, "velodyne");
  pub->publish(output);
}

PointCloud convertToCustomPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &input) {
  PointCloud custom_cloud;

  for (uint32_t i = 0; i < input->height * input->width; ++i) {
    Point pt;

    // Offsets for x, y, z fields in PointCloud2 data
    int x_offset = input->fields[0].offset;
    int y_offset = input->fields[1].offset;
    int z_offset = input->fields[2].offset;

    // Extract x, y, z from the data
    pt[0] = *reinterpret_cast<const float *>(
        &input->data[i * input->point_step + x_offset]);
    pt[1] = *reinterpret_cast<const float *>(
        &input->data[i * input->point_step + y_offset]);
    pt[2] = *reinterpret_cast<const float *>(
        &input->data[i * input->point_step + z_offset]);

    custom_cloud.push_back(pt);
  }

  return custom_cloud;
}

void LidarProcNode::timerCB() { const std::string function_name{"timerCB"}; }

} // namespace center_path
} // namespace utfr_dv