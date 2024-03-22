#include "../include/clusterer.h"
#include "../src/clusterer.cpp"
#include <gtest/gtest.h>
#include <iostream>

using namespace std;

TEST(ClustererTest, RemoveGroundTest) {
  Clusterer clusterer;
  clusterer.ransac_params = {100, 0.05};

  PointCloud points = {{0, 0, 0}, {0, 1, 0}, {1, 0, 0}, {3, 3, 3}};
  Grid min_points_grid; // Assuming Grid is correctly defined and initialized

  PointCloud filtered_points = clusterer.remove_ground(points, min_points_grid);

  // For debugging, print filtered_points
  // for(int i = 0; i < filtered_points.size(); i++){
  //     cout << filtered_points[i][0] << " " << filtered_points[i][1] << " " <<
  //     filtered_points[i][2] << endl;
  // }

  // Assuming ground is z=0 plane, we expect point {3, 3, 3} to remain
  EXPECT_EQ(filtered_points.size(), 1);
  EXPECT_EQ(filtered_points[0], (Point{3, 3, 3}));
}

TEST(ClustererTest, ClusterTest) {
  Clusterer clusterer;
  clusterer.cluster_params = {0.5, 1, 10};

  PointCloud points = {{0, 0, 0}, {0.1, 0.1, 0.1}, {3, 3, 3}};

  PointCloud cluster_centers = clusterer.cluster(points);

  // For debugging
  // for(int i = 0; i < cluster_centers.size(); i++){
  //     cout << cluster_centers[i][0] << " " << cluster_centers[i][1] << " " <<
  //     cluster_centers[i][2] << endl;
  // }

  // We expect two clusters: one near the origin and one at {3, 3, 3}
  EXPECT_EQ(cluster_centers.size(), 2);
  EXPECT_NEAR(cluster_centers[0][0], 0.05, 0.001);
  EXPECT_NEAR(cluster_centers[0][1], 0.05, 0.001);
  EXPECT_NEAR(cluster_centers[0][2], 0.05, 0.001);
  EXPECT_NEAR(cluster_centers[1][0], 3, 0.001);
  EXPECT_NEAR(cluster_centers[1][1], 3, 0.001);
  EXPECT_NEAR(cluster_centers[1][2], 3, 0.001);
}

TEST(ClustererTest, ReconstructTest) {
  Clusterer clusterer;
  clusterer.reconstruct_radius = 0.5;

  PointCloud points = {{0, 0, 0}, {0.1, 0.1, 0.1}, {3, 3, 3}};
  std::vector<Point> cluster_centers = {{0, 0, 0}, {3, 3, 3}};

  std::vector<PointCloud> clusters =
      clusterer.reconstruct(points, cluster_centers);

  // For debugging
  // for(int i = 0; i < clusters.size(); i++){
  //     for(int j = 0; j < clusters[i].size(); j++){
  //         cout << clusters[i][j][0] << " " << clusters[i][j][1] << " " <<
  //         clusters[i][j][2] << endl;
  //     }
  // }

  // We expect each cluster to contain the points that are close to its center
  EXPECT_EQ(clusters.size(), 2);
  EXPECT_EQ(clusters[0].size(), 2);
  EXPECT_EQ(clusters[1].size(), 1);
}

TEST(ClustererTest, CleanAndClusterTest) {
  Clusterer clusterer;
  clusterer.ransac_params = {100, 0.05};
  clusterer.cluster_params = {0.5, 1, 10};
  clusterer.reconstruct_radius = 0.5;

  PointCloud points = {{0, 0, 0}, {0, 1, 0}, {1, 0, 0}, {3, 3, 2}, {-3, 3, 3}};
  Grid min_points_grid; // Assuming Grid is correctly defined and initialized
  std::tuple<PointCloud, std::vector<PointCloud>> output =
      clusterer.clean_and_cluster(points, min_points_grid);
  PointCloud filtered_points = std::get<0>(output);
  std::vector<PointCloud> clusters = std::get<1>(output);

  // Check the filtered_points and clusters
  EXPECT_EQ(filtered_points.size(), 2);
  EXPECT_EQ(clusters.size(), 2);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
