#pragma once

#include <array>
#include <tuple>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <Eigen/Dense>
#include "filter.hpp"

struct ConeLRFilterParams {
  float cone_height;
  float cone_radius;
  float mse_threshold;
  float lin_threshold;
  float IOUThreshold;
};

class ConeLRFilter {
  /*
  */
public:
  float cone_height;
  float cone_radius;
  float mse_threshold;
  float lin_threshold;
  float IOUThreshold;

  ConeLRFilter(){};

  ConeLRFilter(ConeLRFilterParams params);

  /**
   * @brief filters clusters and keeps only cone centers
   * 
   * @param clusters clusters from filtering pipeline
   * 
   * @return PointCloud of cone centers
   */
  PointCloud filter_clusters(std::vector<PointCloud> clusters);
};
