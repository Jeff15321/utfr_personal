/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: cone_filter.hpp
* auth: Kareem Elsawah
* desc: cone filter header
*/

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filter.hpp>
#include <tuple>
#include <vector>

struct ConeFilterParams {
  float cone_height;
  float cone_radius;
  float inlier_mse_threshold;
  int max_iterations;
  float min_inlier_ratio;
};

class ConeFilter {
  /*
   * Class used to determine if a given cluster of points are a cone or not
   */
public:
  float height;
  float a;
  float threshold;
  int max_iterations;
  float min_inlier_ratio;

  ConeFilter(){};

  ConeFilter(ConeFilterParams params);

  /**
   * @brief filters clusters and keeps only cone centers
   *
   * @param clusters clusters from filtering pipeline
   *
   * @return PointCloud of cone centers
   */
  PointCloud filter_clusters(std::vector<PointCloud> clusters);

  /**
   * @brief runs ransac using the cone model to fit it to a cluster of points
   *
   * @param points set of points to fit the cone model to
   *
   * @return Point center of the cone fitted, int number of inliers
   */
  std::tuple<Point, int> ransac(const PointCloud &points);

  /**
   * @brief Fit the cone equation to four points to determine the cone center
   *
   * @param p1, p2, p3, p4 points to fit cone to
   *
   * @return x and y center of the cone
   */
  std::array<float, 2> fit_model(Point p1, Point p2, Point p3, Point p4);

  /**
   * @brief Compute the square error between one point and the fitted model
   *
   * @param model fitted model containing the cone center, point to compute
   * squared error to
   *
   * @return squared distance between fitted cone and point
   */
  float calculateMSE(std::array<float, 2> model, Point point);
};
