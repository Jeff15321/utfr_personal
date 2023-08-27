/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: pid_controller.hpp
* auth: Youssef Elhadad
* desc: pid controller base class
*/
#pragma once

// System Requirements
#include <map>
#include <string>

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/jetson.hpp>
#include <utfr_msgs/msg/target_state.hpp>

namespace utfr_dv {
namespace controls {

class PIDController {
public:
  /*! Initialize controller with car parameters.
   *
   *  @param[in] params std::vector of all controller gains & params
   *  @param[in] name std::string of controller name for error messages
   */
  void initController(const std::vector<double> &params,
                      const std::string &name);

  /*! Calculate Steering angle using pure pursuit algorithim.
   *
   *  @param[in] sp double of controller set point
   *  @param[in] pv double of controller process variable
   *  @param[in] dt double of timestep since last getCommand call
   *  @returns double of calculated control command
   */
  double getCommand(double sp, double pv, double dt);

private:
  /*! Reset error_term_ to prevent integrator windup.
   */
  void resetVariables();

  /*!  Set controller params.
   *  @param[in] params std::map std::map of all controller gains & params
   *  @throws std::out_of_range if required field is missing from map.
   *  @throws std::invalid_argument if required field is negative.
   */
  void setParams(const std::vector<double> &params);

  // stored params
  double p_gain_;
  double i_gain_;
  double d_gain_;

  double output_lim_lower;
  double output_lim_upper;

  std::string name_;

  // controller variables
  double error_sum_;
  double error_prev_;
};

using PIDControllerUPTr = std::unique_ptr<PIDController>;
using PIDControllerSPtr = std::shared_ptr<PIDController>;

} // namespace controls
} // namespace utfr_dv