/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: pid_controller.cpp
* auth: Youssef Elhadad
* desc: pid controller class
*
*/

#include "controller/pid_controller.hpp"

namespace utfr_dv {
namespace controls {

void PIDController::initController(const std::vector<double> &params,
                                   const std::string &name) {
  name_ = name;
  this->setParams(params);
  this->resetVariables();
}

void PIDController::setParams(const std::vector<double> &params) {
  const std::string function_name{"PIDController:setParams" + name_ +
                                  "Controller"};
  std::string key = "";
  try {
    p_gain_ = params[0];
    if (p_gain_ < 0) {
      throw std::invalid_argument("p_gain < 0");
    }
    i_gain_ = params[1];
    if (i_gain_ < 0) {
      throw std::invalid_argument("i_gain < 0");
    }
    d_gain_ = params[2];
    if (d_gain_ < 0) {
      throw std::invalid_argument("d_gain < 0");
    }
    output_lim_lower = params[3];
    output_lim_upper = params[4];

  } catch (const std::out_of_range &e) {
    std::string error = function_name;
    error += e.what();
    error += " '" + key + "', please verify controller config.yaml";
    throw std::out_of_range(error);
  } catch (const std::invalid_argument &e) {
    std::string error = function_name;
    error += e.what();
    error += ", please verify controller config.yaml";
    throw std::invalid_argument(error);
  }
}

double PIDController::getCommand(double pv, double sp, double dt) {

  // The pure pursuit controller sends out NaN. Ignore
  if (std::isnan(pv) || std::isnan(sp)) {
    return 0;
  }
  double error = pv - sp;
  auto node = std::make_shared<rclcpp::Node>("a_test_node");

  error_sum_ += error * dt; // compute integral

  double error_rate = (error - error_prev_) / dt; // compute derivative
  double cmd = p_gain_ * error + i_gain_ * error_sum_ +
               d_gain_ * error_rate; // PID output

  error_prev_ = error; // remember current error

  if (cmd > output_lim_upper)
    cmd = output_lim_upper;
  else if (cmd < output_lim_lower)
    cmd = output_lim_lower;

  return cmd;
}

void PIDController::resetVariables() {
  error_sum_ = 0;
  error_prev_ = 0;
}

} // namespace controls
} // namespace utfr_dv
