/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: path_optimization_node.cpp
* auth: Justin Lim
* desc: path optimization node class
*/

#include <path_optimization_node.hpp>

namespace utfr_dv {
namespace path_optimization {

PathOptimizationNode::PathOptimizationNode() : Node("path_optimization_node") {
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

void PathOptimizationNode::initParams() {}

void PathOptimizationNode::initSubscribers() {}

void PathOptimizationNode::initPublishers() {}

void PathOptimizationNode::initTimers() {}

void PathOptimizationNode::initHeartbeat() {}

std::vector<double> PathOptimizationNode::calculateCurvatures(
                    utfr_msgs::msg::ParametricSpline &spline, double L, int n){
  if(n <= 1) return {};
  std::vector<double> &x = spline.x_params;
  std::vector<double> &y = spline.y_params;
  auto x_first_derivative = [&x](double t){
    return 5*x[0]*pow(t,4) + 4*x[1]*pow(t,3) + 3*x[2]*pow(t,2) + 2*x[3]*t + x[4];
  };
  auto x_second_derivative = [&x](double t){
    return 20*x[0]*pow(t,3) + 12*x[1]*pow(t,2) + 6*x[2]*t + 2*x[3];
  };
  auto y_first_derivative = [&y](double t){
    return 5*y[0]*pow(t,4) + 4*y[1]*pow(t,3) + 3*y[2]*pow(t,2) + 2*y[3]*t + y[4];
  };
  auto y_second_derivative = [&y](double t){
    return 20*y[0]*pow(t,3) + 12*y[1]*pow(t,2) + 6*y[2]*t + 2*y[3];
  };
  auto k = [&](double t){
    double numerator = x_first_derivative(t) * y_second_derivative(t) -
                       x_second_derivative(t) * y_first_derivative(t);
    double denominator = pow(x_first_derivative(t) * x_first_derivative(t) +
                             y_first_derivative(t) * y_first_derivative(t) ,
                             1.5);
    return numerator/denominator;
  };
  std::vector<double> curvatures;
  for(double t = 0; t <= L; t+=L/(n-1)){
    curvatures.push_back(k(t));
  }
  return curvatures;
}

} // namespace path_optimization
} // namespace utfr_dv

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<utfr_dv::path_optimization::PathOptimizationNode>());
  rclcpp::shutdown();
  return 0;
}
