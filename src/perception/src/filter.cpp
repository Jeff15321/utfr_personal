/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: filter.hpp
* auth: Kareem Elsawah
* desc: filter class
*/

#include <filter.hpp>

Grid Filter::ground_grid(PointCloud points) {
  PointCloud filtered_points;
  Grid grid(this->grid_size_x,
            std::vector<Point>(this->grid_size_y, {0, 0, 100}));

  for (Point point : points) {

    // Update ground grid
    std::array<int, 2> grid_index = this->get_grid_index(point);
    if (point[2] < grid[grid_index[0]][grid_index[1]][2]) {
      grid[grid_index[0]][grid_index[1]] = point;
    }
  }

  return grid;
}

std::array<int, 2> Filter::get_grid_index(Point point) {
  std::array<int, 2> grid_index;
  grid_index[0] =
      (point[0] - this->bounds.outer_box.x_min) /
      (this->bounds.outer_box.x_max - this->bounds.outer_box.x_min) *
      this->grid_size_x;
  grid_index[1] =
      (point[1] - this->bounds.outer_box.y_min) /
      (this->bounds.outer_box.y_max - this->bounds.outer_box.y_min) *
      this->grid_size_y;
  return grid_index;
}