#include "../include/clusterer.hpp"
#include "../include/filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

class LidarProcessingNode : public rclcpp::Node {
public:
  LidarProcessingNode() : Node("lidar_processing_node") {
    // Initialize ROS node, publisher, and subscriber
    sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", 10,
        std::bind(&LidarProcessingNode::pointCloudCallback, this,
                  std::placeholders::_1));

    pub_filtered = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lidar_pipeline/filtered_point_cloud", 10);
    pub_no_ground = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lidar_pipeline/no_ground_point_cloud", 10);
    pub_clustered = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lidar_pipeline/clustered_point_cloud", 10);
    pub_clustered_center =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar_pipeline/clustered_center_point_cloud", 10);

    // Initialize Filter and Clusterer instances
    ViewBounds bounds = {{0, 50, -5, 5}, {-1, 1, -2, 2}};
    int grid_size_x = 200;
    int grid_size_y = 200;
    filter = Filter(bounds, grid_size_x, grid_size_y);

    RansacParams ransac_params = {100, 0.2};
    ClusterParams cluster_params = {0.5, 5, 1000};
    float reconstruct_radius = 0.5;
    clusterer = Clusterer(ransac_params, cluster_params, reconstruct_radius);
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
    std::vector<PointCloud> reconstructed_clusters =
        std::get<1>(cluster_results);

    publishPointCloud(no_ground_cloud, pub_no_ground);

    // Publishing each reconstructed cluster in a loop
    for (const auto &cluster : reconstructed_clusters) {
      publishPointCloud(cluster, pub_clustered);
    }

    // Gather the cluster centers into a single point cloud
    PointCloud cluster_centers;
    for (const auto &cluster : reconstructed_clusters) {
      // Computer mean of the cluster
      Point mean = {0, 0, 0};
      for (const auto &point : cluster) {
        mean[0] += point[0];
        mean[1] += point[1];
        mean[2] += point[2];
      }
      mean[0] /= cluster.size();
      mean[1] /= cluster.size();
      mean[2] /= cluster.size();
      cluster_centers.push_back(mean);
    }
    publishPointCloud(cluster_centers, pub_clustered_center);
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

  void publishPointCloud(const PointCloud &cloud,
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

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_no_ground;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clustered;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_clustered_center;
  Filter filter;
  Clusterer clusterer;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarProcessingNode>());
  rclcpp::shutdown();
  return 0;
}