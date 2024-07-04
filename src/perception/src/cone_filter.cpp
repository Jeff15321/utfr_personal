#include "../include/cone_filter.hpp"
#include <iostream>

ConeLRFilter::ConeLRFilter(ConeLRFilterParams params){
    this->cone_height = params.cone_height;
    this->cone_radius = params.cone_radius;
    this->mse_threshold = params.mse_threshold;
    this->lin_threshold = params.lin_threshold;
}

std::tuple<Point, float, float> fit_cone(PointCloud points, float h, float r){
    float a = r/h;

    int n_points = points.size();
    Eigen::MatrixXd phi(n_points, 3);
    Eigen::VectorXd  target(n_points);

    float avg_z = 0;

    for (int i = 0; i < n_points; i++){
        float x = points[i][0];
        float y = points[i][1];
        float z = points[i][2];
        phi(i, 0) = -2*x;
        phi(i, 1) = -2*y;
        phi(i, 2) = 1;

        target(i) = a * a * (z - h) * (z - h) - x * x - y * y;

        avg_z += z;
    }
    avg_z /= n_points;

    Eigen::VectorXd solution = phi.colPivHouseholderQr().solve(target);
    float xc = solution(0);
    float yc = solution(1);
    float b = solution(2);

    Eigen::VectorXd loss = phi*solution - target;

    float mse_loss = loss.dot(loss)/n_points;
    float lin_loss = abs(xc * xc + yc * yc - b);

    Point center = {xc, yc, avg_z};
    return std::tuple<Point, float, float>(center, mse_loss, lin_loss);
}

PointCloud ConeLRFilter::filter_clusters(std::vector<PointCloud> clusters){
    PointCloud cone_centers;
    for(const auto& cluster : clusters){
        std::tuple<Point, float, float> result = fit_cone(cluster, this->cone_height, this->cone_radius);
        Point center = std::get<0>(result);
        float mse_loss = std::get<1>(result);
        float lin_loss = std::get<2>(result);
        float n_points = (float) cluster.size();
        std::cout << "MSE: " << mse_loss << ", Lin: " << lin_loss << std::endl;
        if (!std::isnan(mse_loss) && mse_loss < this->mse_threshold && lin_loss < this->lin_threshold && center[2] < this->cone_height + 0.1) //check if mse_loss is NaN
        {
            cone_centers.push_back(center);
        }
    }
    return cone_centers;
}