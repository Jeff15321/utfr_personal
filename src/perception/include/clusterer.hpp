/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: clusterer.hpp
* auth: Kareem Elsawah
* desc: clusterer header
*/

#pragma once

#include <algorithm>
#include <cmath>
#include <filter.hpp>
#include <limits>
#include <random>
#include <tuple>

#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>

#include <utfr_common/topics.hpp>

struct RansacParams {
  int maxIterations;
  float distanceThreshold;
};

struct ClusterParams {
  float clusterTolerance;
  int minClusterSize;
  int maxClusterSize;
};

class Clusterer {
  /*
  This class is used to remove ground points from the point cloud,
  cluster the remaining points,
  do reconstruction,
  then return the reconstructed clusters.
  */
public:
  RansacParams ransac_params;
  ClusterParams cluster_params;
  float reconstruct_radius;

  Clusterer(){};

  Clusterer(RansacParams ransac_params, ClusterParams cluster_params,
            float reconstruct_radius)
      : ransac_params(ransac_params), cluster_params(cluster_params),
        reconstruct_radius(reconstruct_radius){};

  /**
   * @brief Remove ground points from the point cloud, cluster the remaining
   * points, do reconstruction, then return the reconstructed clusters
   *
   * @param points The point cloud to cluster
   * @param min_points_grid The grid of lowest points
   *
   * @return A tuple, first element is the filtered points, second element is
   * the reconstructed clusters
   */
  std::tuple<PointCloud, std::vector<PointCloud>>
  clean_and_cluster(PointCloud points, Grid min_points_grid);

  /**
   * @brief Remove ground points from the point cloud
   *
   * @param points The point cloud to remove ground points from
   * @param min_points The grid of lowest points
   *
   * @return The point cloud with ground points removed
   */
  PointCloud remove_ground(PointCloud points, Grid min_points);

  /**
   * @brief Cluster the remaining points
   *
   * @param points The point cloud to cluster
   *
   * @return The cluster centers
   */
  std::vector<Point> cluster(PointCloud points);

  /**
   * @brief Do reconstruction
   *
   * @param points The point cloud to reconstruct
   * @param cluster_centers The cluster centers
   *
   * @return The reconstructed clusters
   */
  std::vector<PointCloud> reconstruct(PointCloud points,
                                      std::vector<Point> cluster_centers);
};
