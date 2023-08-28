/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: controls_unit_tests.cpp
* auth: Kelvin Cui
* desc: sim_interface common functions unit tests
*/

#include <controls_node.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <utfr_common/math.hpp>

using namespace utfr_dv::controls;

/*TEST(Controllers, PurePursuit){

  double wheelbase = 4;
  double lookahead_distance_scaling_factor = 1;

  utfr_dv::controls::PurePursuitController pure_pursuit =
      PurePursuitController();

  pure_pursuit.initController(
      wheelbase, lookahead_distance_scaling_factor);

  utfr_msgs::msg::TargetState::SharedPtr current_target_state;

  current_target_state->trajectory.pos.x = 0;
  current_target_state->trajectory.pos.y = 0;

  utfr_msgs::msg::EgoState::SharedPtr current_ego_state;

  current_ego_state->pos.x = 0;
  current_ego_state->pos.y = 0;
  current_ego_state->pos.z = 0;
  current_ego_state->vel.x = 1;

  double angle = pure_pursuit.getSteeringAngle(current_target_state,
                                                current_ego_state);


  ASSERT_DOUBLE_EQ(angle, 0.0);

  //ASSERT_DOUBLE_EQ(degToRad(deg), 1.0);

  // backwards
  current_target_state->trajectory.pos.x = -20;
  current_target_state->trajectory.pos.y = 0;

  current_ego_state->pos.x = 0;
  current_ego_state->pos.y = 0;
  current_ego_state->pos.z = 0;
  current_ego_state->vel.x = 1;

  angle = pure_pursuit.getSteeringAngle(current_target_state,
                                                current_ego_state);

  // pi
  ASSERT_DOUBLE_EQ(angle, round((utfr_dv::util::radToDeg(0.41822)/90)*100)/100);

  // Full left
  current_target_state->trajectory.pos.x = -2;
  current_target_state->trajectory.pos.y = 20;

  current_ego_state->pos.x = 0;
  current_ego_state->pos.y = 0;
  current_ego_state->pos.z = 0;
  current_ego_state->vel.x = 1;

  angle = pure_pursuit.getSteeringAngle(current_target_state,
                                                current_ego_state);

  ASSERT_DOUBLE_EQ(angle,round((utfr_dv::util::radToDeg(0.38050)/90)*100)/100);

  //wheel base
  current_target_state->trajectory.pos.x = 0;
  current_target_state->trajectory.pos.y = wheelbase;

  current_ego_state->pos.x = 0;
  current_ego_state->pos.y = 0;
  current_ego_state->pos.z = 0;
  current_ego_state->vel.x = 1;

  angle = pure_pursuit.getSteeringAngle(current_target_state,
                                                current_ego_state);

  ASSERT_DOUBLE_EQ(angle,
std::clamp(round((utfr_dv::util::radToDeg(1.01219)/90)*100)/100,-1.0,1.0));

  //wheel base left
  current_target_state->trajectory.pos.x = wheelbase;
  current_target_state->trajectory.pos.y = 0;

  current_ego_state->pos.x = 0;
  current_ego_state->pos.y = 0;
  current_ego_state->pos.z = 0;
  current_ego_state->vel.x = 1;

  angle = pure_pursuit.getSteeringAngle(current_target_state,
                                                current_ego_state);
  ASSERT_DOUBLE_EQ(angle, 0.0);

  //89.9 degrees left
  current_target_state->trajectory.pos.x = -1;
  current_target_state->trajectory.pos.y = 572;

  current_ego_state->pos.x = 0;
  current_ego_state->pos.y = 0;
  current_ego_state->pos.z = 0;
  current_ego_state->vel.x = 1;

  angle = pure_pursuit.getSteeringAngle(current_target_state,
                                                current_ego_state);
  ASSERT_DOUBLE_EQ(angle, round((utfr_dv::util::radToDeg(0.01396)/90)*100)/100);

  //90.1 degrees left
  current_target_state->trajectory.pos.x = -3;
  current_target_state->trajectory.pos.y = 573;

  current_ego_state->pos.x = 0;
  current_ego_state->pos.y = 0;
  current_ego_state->pos.z = 0;
  current_ego_state->vel.x = 1;

  angle = pure_pursuit.getSteeringAngle(current_target_state,
                                                current_ego_state);
  ASSERT_DOUBLE_EQ(angle, round((utfr_dv::util::radToDeg(0.01396)/90)*100)/100);

  // 0.1 degrees
  current_target_state->trajectory.pos.x = 571;
  current_target_state->trajectory.pos.y = 1;

  current_ego_state->pos.x = 0;
  current_ego_state->pos.y = 0;
  current_ego_state->pos.z = 0;
  current_ego_state->vel.x = 1;

  angle = pure_pursuit.getSteeringAngle(current_target_state,
                                                current_ego_state);
  ASSERT_DOUBLE_EQ(angle, round((utfr_dv::util::radToDeg(0.00139)/90)*100)/100);

  // 364.9 degrees
  current_target_state->trajectory.pos.x = 571;
  current_target_state->trajectory.pos.y = -1;

  current_ego_state->pos.x = 0;
  current_ego_state->pos.y = 0;
  current_ego_state->pos.z = 0;
  current_ego_state->vel.x = 1;

  angle = pure_pursuit.getSteeringAngle(current_target_state,
                                                current_ego_state);
  ASSERT_DOUBLE_EQ(angle,
round((utfr_dv::util::radToDeg(-0.00139)/90)*100)/100);
}*/

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
