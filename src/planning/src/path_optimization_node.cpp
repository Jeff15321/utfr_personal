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
  RCLCPP_INFO(this->get_logger(), "Path Optimization Node Launched");
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

std::vector<double> PathOptimizationNode::calculateVelocities(
                    utfr_msgs::msg::ParametricSpline &spline,
                    double L, int n, double a_lateral){
  if(n <= 1) return {};
  auto first_derivative = [](std::vector<double> &c, double t, double t2, 
                              double t3, double t4){
    return 5*c[0]*t4 + 4*c[1]*t3 + 3*c[2]*t2 + 2*c[3]*t + c[4];
  };
  auto second_derivative = [](std::vector<double> &c, double t, double t2, 
                              double t3){
    return 20*c[0]*t3 + 12*c[1]*t2 + 6*c[2]*t + 2*c[3];
  };
  std::vector<double> &x = spline.x_params;
  std::vector<double> &y = spline.y_params;
  auto k = [&x, &y, &first_derivative, &second_derivative](double t){
    double t2 = t*t, t3 = t2*t, t4 = t3*t;
    double x_first_derivative = first_derivative(x,t,t2,t3,t4);
    double x_second_derivative = second_derivative(x,t,t2,t3);
    double y_first_derivative = first_derivative(y,t,t2,t3,t4);
    double y_second_derivative = second_derivative(y,t,t2,t3);
    double numerator = x_first_derivative * y_second_derivative -
                       x_second_derivative * y_first_derivative;
    double val = x_first_derivative * x_first_derivative +
                 y_first_derivative * y_first_derivative;
    double denominator = val * sqrt(val);
    return abs(numerator/denominator);
  };
  std::vector<double> velocities;
  for(double t = 0; t <= L; t+=L/(n-1)){
    velocities.push_back(sqrt(a_lateral/k(t)));
  }
  return velocities;
}

} // namespace path_optimization
} // namespace utfr_dv
