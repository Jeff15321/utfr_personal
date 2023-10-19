#include "../include/clusterer.h"

using namespace std;

PointCloud Clusterer::remove_ground(PointCloud points, Grid min_points_grid) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Resize the PCL point cloud
  cloud->width = points.size();
  cloud->height = 1; // This makes the point cloud unorganized
  cloud->points.resize(cloud->width * cloud->height);

  // Fill the PCL point cloud with data from the input vector of min_points_grid
  // PointCloud min_points;
  // for (int i = 0; i < min_points_grid.size(); i++)
  // {
  //   for (int j = 0; j < min_points_grid[i].size(); j++){
  //     if (min_points_grid[i][j][2] < 99){
  //       min_points.push_back(min_points_grid[i][j]);
  //     }
  //   }
  // }
  for (size_t i = 0; i < points.size(); ++i) {
    cloud->points[i].x = points[i][0];
    cloud->points[i].y = points[i][1];
    cloud->points[i].z = points[i][2];
  }

  // Create a shared pointer for the RANSAC plane model
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(
      new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
  ransac.setDistanceThreshold(this->ransac_params.distanceThreshold);
  ransac.setMaxIterations(this->ransac_params.maxIterations);

  ransac.computeModel();

  Eigen::VectorXf coefficients;
  ransac.getModelCoefficients(coefficients);

  // For debugging: printing the coefficients of the plane equation ax + by + cz
  // + d = 0 std::cout << "The coefficients of the fitted plane are: "
  //           << coefficients[0] << "x + "
  //           << coefficients[1] << "y + "
  //           << coefficients[2] << "z + "
  //           << coefficients[3] << " = 0" << std::endl;

  float a = coefficients[0], b = coefficients[1], c = coefficients[2], d = coefficients[3];

  // Remove ground points (inliers of the plane)
  PointCloud filtered_points;
  for (int i = 0; i < cloud->points.size(); ++i) {
    float x = cloud->points[i].x, y = cloud->points[i].y, z = cloud->points[i].z;

    // Calculate the distance of each point to the plane
    float distance = std::abs(a*x + b*y + c*z + d) / std::sqrt(a*a + b*b + c*c);
    
    // Only include points that are farther than the threshold
    if (distance > this->ransac_params.distanceThreshold) {
      filtered_points.push_back({x, y, z});
    }
  }

  return filtered_points;
}

std::vector<Point> Clusterer::cluster(PointCloud points) {
  // Convert std::vector<Point> to pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.resize(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    cloud->points[i].x = points[i][0];
    cloud->points[i].y = points[i][1];
    cloud->points[i].z = points[i][2];
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(this->cluster_params.clusterTolerance);
  ec.setMinClusterSize(this->cluster_params.minClusterSize);
  ec.setMaxClusterSize(this->cluster_params.maxClusterSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusterIndices);

  // Find and return the centers of the clusters
  std::vector<Point> clusterCenters;
  for (const auto &indices : clusterIndices) {
    float avgX = 0, avgY = 0, avgZ = 0;
    for (int index : indices.indices) {
      avgX += cloud->points[index].x;
      avgY += cloud->points[index].y;
      avgZ += cloud->points[index].z;
    }
    avgX /= indices.indices.size();
    avgY /= indices.indices.size();
    avgZ /= indices.indices.size();
    clusterCenters.push_back({avgX, avgY, avgZ});
  }

  return clusterCenters;
}

std::vector<PointCloud>
Clusterer::reconstruct(PointCloud points, std::vector<Point> cluster_centers) {
  // For each cluster, get points that are within radius of the cluster center,
  // use kd_trees

  // Convert std::vector<Point> to pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->points.resize(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    cloud->points[i].x = points[i][0];
    cloud->points[i].y = points[i][1];
    cloud->points[i].z = points[i][2];
  }

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  // For each cluster center, find the points within radius
  std::vector<PointCloud> clusters;
  for (int i = 0; i < cluster_centers.size(); i++) {
    PointCloud cluster;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    pcl::PointXYZ search_point;
    search_point.x = cluster_centers[i][0];
    search_point.y = cluster_centers[i][1];
    search_point.z = cluster_centers[i][2];
    tree->radiusSearch(search_point, this->reconstruct_radius,
                       pointIdxRadiusSearch, pointRadiusSquaredDistance);

    // Add the points to the cluster
    for (int j = 0; j < pointIdxRadiusSearch.size(); j++) {
      cluster.push_back(points[pointIdxRadiusSearch[j]]);
    }

    clusters.push_back(cluster);
  }

  return clusters;
}

std::tuple<PointCloud, std::vector<PointCloud>>
Clusterer::clean_and_cluster(PointCloud points, Grid min_points_grid) {
  // Remove ground points
  PointCloud filtered_points = remove_ground(points, min_points_grid);

  // For debugging
  // cout << "Num filtered points: " << filtered_points.size() << endl;

  // Cluster the remaining points
  std::vector<Point> cluster_centers = cluster(filtered_points);

  // For debugging
  // for(int i = 0; i < cluster_centers.size(); i++){
  //     cout << cluster_centers[i][0] << " " << cluster_centers[i][1] << " " <<
  //     cluster_centers[i][2] << endl;
  // }

  // Reconstruct the clusters
  std::vector<PointCloud> clusters = reconstruct(points, cluster_centers);

  return std::make_tuple(filtered_points, clusters);
}