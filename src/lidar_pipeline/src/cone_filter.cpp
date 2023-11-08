#include "cone_filter.h"

ConeFilter::ConeFilter(ConeFilterParams params) {
  this->height = params.cone_height;
  this->a = (params.cone_height * params.cone_height) /
            (params.cone_radius * params.cone_radius);
  this->threshold = params.inlier_mse_threshold;
  this->max_iterations = params.max_iterations;
  this->min_inlier_ratio = params.min_inlier_ratio;
}

std::array<float, 2> ConeFilter::fit_model(Point p1, Point p2, Point p3,
                                           Point p4) {
  float t1 = (p1[2] - this->height) * (p1[2] - this->height);
  float t2 = (p2[2] - this->height) * (p2[2] - this->height);
  float t3 = (p3[2] - this->height) * (p3[2] - this->height);
  float t4 = (p4[2] - this->height) * (p4[2] - this->height);
  float a = this->a;
  float b = this->a;

  float x1 = p1[0];
  float x2 = p2[0];
  float y1 = p1[1];
  float y2 = p2[1];
  float x3 = p3[0];
  float x4 = p4[0];
  float y3 = p3[1];
  float y4 = p4[1];

  float k0 = (a * x1 * x1 - a * x2 * x2 + b * y1 * y1 - b * y2 * y2 - t1 + t2) /
             (2 * a * (x1 - x2));
  float k1 = (b) / (a * (x1 - x2));

  float yc_num = a * k0 * x3 - a * k0 * x4 +
                 0.5 * (-1 * a * x3 * x3 + a * x4 * x4 - b * y3 * y3 +
                        b * y4 * y4 + t3 - t4);
  float yc_denom = a * k1 * x3 * y1 - a * k1 * x3 * y2 - a * k1 * x4 * y1 +
                   a * k1 * x4 * y2 - b * y3 + b * y4;

  float yc = yc_num / yc_denom;

  float xc = (a * x1 * x1 - a * x2 * x2 + b * y1 * y1 - 2 * b * y1 * yc -
              b * y2 * y2 + 2 * b * y2 * yc - t1 + t2) /
             (2 * a * (x1 - x2));

  std::array<float, 2> center = {xc, yc};

  return center;
}

float ConeFilter::calculateMSE(std::array<float, 2> model, Point point) {
  float x = point[0];
  float y = point[1];
  float z = point[2];
  float a = this->a;
  float b = this->a;
  float h = this->height;

  float pred =
      a * (x - model[0]) * (x - model[0]) + b * (y - model[1]) * (y - model[1]);
  float pred_z = h - sqrt(pred);
  float mse = (z - pred_z) * (z - pred_z);

  return mse;
}

std::tuple<Point, int> ConeFilter::ransac(const PointCloud &points) {
  int best_inliers = 0;
  std::array<float, 2> best_model;

  for (int i = 0; i < this->max_iterations; ++i) {
    int index1 = rand() % points.size();
    int index2;
    do {
      index2 = rand() % points.size();
    } while (index2 == index1);

    int index3;
    do {
      index3 = rand() % points.size();
    } while (index3 == index1 || index3 == index2);

    int index4;
    do {
      index4 = rand() % points.size();
    } while (index4 == index1 || index4 == index2 || index4 == index3);

    std::array<float, 2> model = fit_model(points[index1], points[index2],
                                           points[index3], points[index4]);

    int inliers_count = 0;
    for (const auto &point : points) {
      float mse = calculateMSE(model, {point});
      if (mse < this->threshold) {
        ++inliers_count;
      }
    }

    if (inliers_count > best_inliers) {
      best_inliers = inliers_count;
      best_model = model;
    }
  }

  Point model_to_return = {best_model[0], best_model[1], 0};

  std::tuple<Point, float> result = {model_to_return, best_inliers};
  return result;
}

PointCloud ConeFilter::filter_clusters(std::vector<PointCloud> clusters) {
  PointCloud cone_centers;
  for (const auto &cluster : clusters) {
    if (cluster.size() < 4) {
      continue;
    }
    std::tuple<Point, float> result = ransac(cluster);
    Point center = std::get<0>(result);
    int inliers = std::get<1>(result);
    if (inliers / cluster.size() > this->min_inlier_ratio) {
      cone_centers.push_back(center);
    }
  }
  return cone_centers;
}