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

std::tuple<PointCloud, Grid> Filter::view_filter(PointCloud points) {
  PointCloud filtered_points;
  Grid grid(this->grid_size_x,
            std::vector<Point>(this->grid_size_y, {0, 0, 100}));

  for (Point point : points) {
    // Check if point is within outer box
    if (point[0] >= this->bounds.outer_box.x_min &&
        point[0] <= this->bounds.outer_box.x_max &&
        point[1] >= this->bounds.outer_box.y_min &&
        point[1] <= this->bounds.outer_box.y_max) {
      // Check if point is outside inner box
      if (!(point[0] >= this->bounds.inner_box.x_min &&
            point[0] <= this->bounds.inner_box.x_max &&
            point[1] >= this->bounds.inner_box.y_min &&
            point[1] <= this->bounds.inner_box.y_max)) {
        // Points are in the specified view
        filtered_points.push_back(point);

        // Update ground grid
        std::array<int, 2> grid_index = this->get_grid_index(point);
        if (point[2] < grid[grid_index[0]][grid_index[1]][2]) {
          grid[grid_index[0]][grid_index[1]] = point;
        }
      }
    }
  }

  return std::tuple<PointCloud, Grid>(filtered_points, grid);
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