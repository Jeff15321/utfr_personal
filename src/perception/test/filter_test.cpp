#include "../include/filter.h"
#include <gtest/gtest.h>
#include <iostream>

using namespace std;

TEST(FilterTest, TestViewFilter) {
  ViewBounds bounds = {{0, 10, 0, 10}, {2, 8, 2, 8}};
  Filter filter(bounds, 3, 3);
  PointCloud points = {{1, 1, 1}, {3, 3, 3}, {5, 5, 5}, {7, 7, 7}, {9, 9, 9}};
  tuple<PointCloud, Grid> filtered_points = filter.view_filter(points);
  PointCloud expected_points = {{1, 1, 1}, {9, 9, 9}};

  EXPECT_EQ(get<0>(filtered_points), expected_points);

  // For debugging
  /*
  // print grid
  Grid grid = get<1>(filtered_points);
  for(int i = 0; i < grid.size(); i++){
      for(int j = 0; j < grid[i].size(); j++){
          cout << grid[i][j][2] << " ";
      }
      cout << endl;
  }

  // print points
  for(int i = 0; i < get<0>(filtered_points).size(); i++){
      cout << get<0>(filtered_points)[i][0] << " " <<
  get<0>(filtered_points)[i][1] << " " << get<0>(filtered_points)[i][2] << endl;
  }
  */
}

TEST(FilterTest, TestConstructorAndAttributes) {
  ViewBounds bounds = {{0, 10, 0, 10}, {2, 8, 2, 8}};
  Filter filter(bounds, 3, 3);

  EXPECT_EQ(filter.bounds.outer_box.x_min, 0);
  EXPECT_EQ(filter.grid_size_x, 3);
  EXPECT_EQ(filter.grid_size_y, 3);
}

TEST(FilterTest, TestGetGridIndex) {
  ViewBounds bounds = {{0, 10, 0, 10}, {2, 8, 2, 8}};
  Filter filter(bounds, 10, 10);
  Point point = {5, 5, 5};

  std::array<int, 2> index = filter.get_grid_index(point);

  EXPECT_EQ(index[0], 5);
  EXPECT_EQ(index[1], 5);
}

TEST(FilterTest, TestOuterBoxFiltering) {
  ViewBounds bounds = {{0, 10, 0, 10}, {2, 8, 2, 8}};
  Filter filter(bounds, 3, 3);
  PointCloud points = {{-1, 1, 1}, {11, 11, 11}};

  std::tuple<PointCloud, Grid> output = filter.view_filter(points);
  PointCloud filtered_points = get<0>(output);

  EXPECT_TRUE(filtered_points.empty());
}

TEST(FilterTest, TestInnerBoxFiltering) {
  ViewBounds bounds = {{0, 10, 0, 10}, {2, 8, 2, 8}};
  Filter filter(bounds, 3, 3);
  PointCloud points = {{3, 3, 3}, {7, 7, 7}};

  std::tuple<PointCloud, Grid> output = filter.view_filter(points);
  PointCloud filtered_points = get<0>(output);

  EXPECT_TRUE(filtered_points.empty());
}

TEST(FilterTest, TestGridLowestPointCalculation) {
  ViewBounds bounds = {{0, 10, 0, 10}, {2, 8, 2, 8}};
  Filter filter(bounds, 10, 10);
  PointCloud points = {{1, 1, 5}, {1, 1, 3}, {1, 1, 7}};

  Grid grid = get<1>(filter.view_filter(points));

  EXPECT_EQ(grid[1][1][2], 3);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
