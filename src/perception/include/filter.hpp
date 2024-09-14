/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: filter.hpp
* auth: Kareem Elsawah
* desc: filter header
*/

#pragma once

#include <array>
#include <tuple>
#include <vector>

typedef std::array<float, 3> Point;
typedef std::vector<Point> PointCloud;
typedef std::vector<std::vector<Point>> Grid;

struct Box {
  double x_min;
  double x_max;
  double y_min;
  double y_max;
};

struct ViewBounds {
  Box outer_box;
  Box inner_box;
};

class Filter {
  /*
  This class is used to filter the point cloud to only include points within the
  specified view bounds. It also creates a grid of points that are the lowest
  points in each grid cell.
  */
public:
  ViewBounds bounds;
  int grid_size_x;
  int grid_size_y;

  ViewBounds getBounds() { return bounds; }

  Filter(){};

  /**
   * @brief Constructor for the filter class
   *
   * @param bounds The outer and inner bounds of the view
   * @param grid_size_x The number of grid cells in the x direction
   * @param grid_size_y The number of grid cells in the y direction
   */
  Filter(ViewBounds bounds, int grid_size_x, int grid_size_y)
      : bounds(bounds), grid_size_x(grid_size_x), grid_size_y(grid_size_y){};

  /**
   * @brief Get the grid index of a point
   *
   * @param point The point to get the grid index of
   *
   * @return The grid index of the point (x, y)
   */
  std::array<int, 2> get_grid_index(Point point);

  /**
   * @brief Create a grid of the lowest points in each grid cell
   *
   * @param points The point cloud to filter
   *
   * @return A tuple containing the filtered point cloud and the grid of lowest
   * points
   */
  Grid ground_grid(PointCloud points);
};
