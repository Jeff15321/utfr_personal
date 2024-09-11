#include "../include/cone_filter.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

ConeLRFilter::ConeLRFilter(ConeLRFilterParams params) {
  this->cone_height = params.cone_height;
  this->cone_radius = params.cone_radius;
  this->big_cone_height = params.big_cone_height;
  this->big_cone_radius = params.big_cone_radius;
  this->mse_threshold = params.mse_threshold;
  this->lin_threshold = params.lin_threshold;
  this->IOUThreshold = params.IOUThreshold;
}

float calculateVolume(float h, float r) { return M_PI * r * r * h / 3; }

float calculateIOU(float fitted_r, float fitted_h, float true_r, float true_h) {
  float fitted_volume = calculateVolume(fitted_h, fitted_r);
  float true_volume = calculateVolume(true_h, true_r);

  float overlap_height = std::min(fitted_h, true_h);
  float overlap_radius = std::min(fitted_r, true_r);
  float intersection_volume = calculateVolume(overlap_height, overlap_radius);

  return intersection_volume /
         (fitted_volume + true_volume - intersection_volume);
}

std::tuple<Point, float, float, float, float, float>
fit_cone(PointCloud points, float h, float r) {
  float a = r / h;

  int n_points = points.size();
  Eigen::MatrixXd phi(n_points, 3);
  Eigen::VectorXd target(n_points);

  float avg_z = 0;

  for (int i = 0; i < n_points; i++) {
    float x = points[i][0];
    float y = points[i][1];
    float z = points[i][2];
    phi(i, 0) = -2 * x;
    phi(i, 1) = -2 * y;
    phi(i, 2) = 1;

    target(i) = a * a * (z - h) * (z - h) - x * x - y * y;

    avg_z += z;
  }
  avg_z /= n_points;

  Eigen::VectorXd solution = phi.colPivHouseholderQr().solve(target);
  float xc = solution(0);
  float yc = solution(1);
  float b = solution(2);

  Eigen::VectorXd loss = phi * solution - target;

  float mse_loss = loss.dot(loss) / n_points;
  float lin_loss = abs(xc * xc + yc * yc - b);

  Point center = {xc, yc, avg_z};

  Eigen::MatrixXd C(n_points, 3);
  Eigen::VectorXd d(n_points);

  float z_min = std::numeric_limits<float>::max();
  float z_max = std::numeric_limits<float>::min();
  float base_radius = 0.0;

  for (int i = 0; i < n_points; i++) {
    float x = points[i][0];
    float y = points[i][1];
    float z = points[i][2];

    float radius = std::sqrt(std::pow(x - xc, 2) + std::pow(y - yc, 2));

    C(i, 0) = std::pow(z, 2);
    C(i, 1) = -2 * z;
    C(i, 2) = 1;
    d(i) = std::pow(radius, 2);

    if (z < z_min) {
      z_min = z;
      base_radius = (float)radius;
    }
    if (z > z_max) {
      z_max = z;
    }
  }

  Eigen::VectorXd solution2 = C.colPivHouseholderQr().solve(d);

  float c_squared = solution2(0);
  float c_squared_apex = solution2(1);
  float c_squared_apex_squared = solution2(2);

  float z_apex = c_squared_apex / c_squared;

  float height = z_apex - z_min;

  // center[2] = z_apex;

  return std::tuple<Point, float, float, float, float, float>(
      center, mse_loss, lin_loss, height, base_radius, z_apex);
}

PointCloud ConeLRFilter::filter_clusters(std::vector<PointCloud> clusters) {
  PointCloud cone_centers;
  for (const auto &cluster : clusters) {
    std::tuple<Point, float, float, float, float, float> result =
        fit_cone(cluster, this->cone_height, this->cone_radius);
    Point center = std::get<0>(result);
    float mse_loss = std::get<1>(result);
    float lin_loss = std::get<2>(result);
    float fitted_height = std::get<3>(result);
    float base_radius = std::get<4>(result);
    float n_points = (float)cluster.size();
    float z = center[3];
    float z_apex = std::get<5>(result);
    // std::cout << "fitted_height: " << fitted_height
    //           << ", base_radius: " << base_radius << std::endl;
    float IOU = calculateIOU(base_radius, fitted_height, this->cone_radius,
                             this->cone_height);
    float IOU_big = calculateIOU(base_radius, fitted_height,
                                 this->big_cone_radius, this->big_cone_height);
    // std::cout << "IOU: " << IOU << ", IOU_big: " << IOU_big << std::endl;
    if (z < 5.0 && base_radius < 0.9 && !std::isnan(mse_loss) &&
        (IOU > this->IOUThreshold || IOU_big > this->IOUThreshold)) {
      cone_centers.push_back(center);
    }
  }
  return cone_centers;
}