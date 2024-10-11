/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: clusterer.hpp
* auth: Kareem Elsawah
* desc: clusterer class
*/

#include <chrono>
#include <clusterer.hpp>

using std::chrono::duration;
using std::chrono::high_resolution_clock;

PointCloud Clusterer::remove_ground(PointCloud points, Grid min_points_grid) {

  // Fill the PCL point cloud with data from the input vector of min_points_grid
  PointCloud min_points;
  for (int i = 0; i < min_points_grid.size(); i++) {
    for (int j = 0; j < min_points_grid[i].size(); j++) {
      if (min_points_grid[i][j][2] < 99) {
        min_points.push_back(min_points_grid[i][j]);
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Resize the PCL point cloud
  cloud->width = min_points.size();
  cloud->height = 1; // This makes the point cloud unorganized
  cloud->points.resize(cloud->width * cloud->height);
  for (size_t i = 0; i < min_points.size(); ++i) {
    cloud->points[i].x = min_points[i][0];
    cloud->points[i].y = min_points[i][1];
    cloud->points[i].z = min_points[i][2];
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

  float a = coefficients[0], b = coefficients[1], c = coefficients[2],
        d = coefficients[3];

  // Remove ground points (inliers of the plane)
  PointCloud filtered_points;
  for (int i = 0; i < points.size(); ++i) {
    float x = points[i][0], y = points[i][1], z = points[i][2];

    // Calculate the distance of each point to the plane
    float distance =
        std::abs(a * x + b * y + c * z + d) / std::sqrt(a * a + b * b + c * c);

    // Only include points that are farther than the threshold
    if (distance > this->ransac_params.distanceThreshold) {
      filtered_points.push_back({x, y, z});
    }
  }

  return filtered_points;
}

PointCloud Clusterer::remove_ground_himmelsbach(PointCloud points,
                                                Grid min_points_grid) {
  PointCloud min_points;
  for (int i = 0; i < min_points_grid.size(); i++) {
    for (int j = 0; j < min_points_grid[i].size(); j++) {
      if (min_points_grid[i][j][2] < 99) {
        min_points.push_back(min_points_grid[i][j]);
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width = min_points.size();
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  for (size_t i = 0; i < min_points.size(); ++i) {
    cloud->points[i].x = min_points[i][0];
    cloud->points[i].y = min_points[i][1];
    cloud->points[i].z = min_points[i][2];
  }

  PointCloud filtered_points;
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    float x = cloud->points[i].x;
    float y = cloud->points[i].y;
    float z = cloud->points[i].z;

    float expected_ground_z = 0.0000001 * sqrt(x * x + y * y) - 100;

    if (z > expected_ground_z + 0.00) {
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
  // profiling code
  //  auto startTime = high_resolution_clock::now();
  //  auto t1 = high_resolution_clock::now();

  // Remove ground points
  PointCloud filtered_points = remove_ground(points, min_points_grid);

  // auto t2 = high_resolution_clock::now();
  // duration<double, std::milli> ms_double = t2 - t1;
  // std::cout << "ground remove " << ms_double.count() << "ms\n";
  // t1 = high_resolution_clock::now();

  // Cluster the remaining points
  std::vector<Point> cluster_centers = cluster(filtered_points);

  // t2 = high_resolution_clock::now();
  // ms_double = t2 - t1;
  // std::cout << "cluster " << ms_double.count() << "ms\n";
  // t1 = high_resolution_clock::now();

  // Reconstruct the clusters
  std::vector<PointCloud> clusters = reconstruct(points, cluster_centers);

  // t2 = high_resolution_clock::now();
  // ms_double = t2 - t1;
  // std::cout << "reconstruct " << ms_double.count() << "ms\n";
  // t1 = high_resolution_clock::now();

  // ms_double = high_resolution_clock::now() - startTime;
  // std::cout << "final time: " << ms_double.count() << "ms\n";

  // t1 = high_resolution_clock::now();

  auto tuple = std::make_tuple(filtered_points, clusters);

  // t2 = high_resolution_clock::now();
  // ms_double = t2 - t1;
  // std::cout << "reconstruct " << ms_double.count() << "ms\n";
  // t1 = high_resolution_clock::now();

  return tuple;
}