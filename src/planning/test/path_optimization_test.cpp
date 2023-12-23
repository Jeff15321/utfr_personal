/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: path_optimization_test.cpp
* auth: Justin Lim, Richard Li
* desc: tests for path optimization node
*/

#include <gtest/gtest.h>
#include <math.h>
#include <path_optimization_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utfr_common/math.hpp>

using namespace utfr_dv::path_optimization;

TEST(PathOptimizationNode, calculateVelocities) {
  PathOptimizationNode node;
  utfr_msgs::msg::ParametricSpline spline;
  std::vector<double> curvatures;
  double L;
  int n;
  double a_lateral;
  std::vector<double> velocities;

  spline.x_params = {5.5, 6, 82, 12, -2, -13};
  spline.y_params = {-323, 23, -32.1, 0, 5, -23};
  curvatures = {
      0.768394502801593937135749001754447817802429199218750000000000000000000000000000000000000,
      0.000204322584520719803449079199353377589432056993246078491210937500000000000000000000000,
      0.000001755728428985666169054140339778058432784746401011943817138671875000000000000000000,
      0.000000105888674565807749814371971353033075047278543934226036071777343750000000000000000,
      0.000000014581750458310574596171452839855203764685143141832668334245681762695312500000000,
      0.000000003160793591098613176104636620419169323881192212866153568029403686523437500000000,
      0.000000000912506333903875565319945154581660756720751237480726558715105056762695312500000,
      0.000000000320888752617905436020770856313179437979687946835838374681770801544189453125000,
      0.000000000130321346256518592480948724260597995638910617799410829320549964904785156250000,
      0.000000000059062509456425061000142101429204915044590684658487589331343770027160644531250,
      0.000000000029179596973419697748607865694976822544537409243048386997543275356292724609375};

  L = 10;
  n = 11;
  a_lateral = 15;
  velocities = node.calculateVelocities(spline, L, n, a_lateral);
  ASSERT_EQ(curvatures.size(), velocities.size());
  int size = curvatures.size();
  for (int i = 0; i < size; i++) {
    double calculated_velocity = sqrt(a_lateral / curvatures[i]);
    ASSERT_DOUBLE_EQ(calculated_velocity, velocities[i]);
  }
}

TEST(PathOptimizationNode, filterVelocities) {
  PathOptimizationNode node;
  std::vector<double> v;
  std::ofstream out("test_output.txt");

  out << std::setprecision(17);

  auto test = [&v, &out, &node](double cur_vel, double time, double max_vel,
                                double max_accel, double max_decel) {
    out << "Max velocities:" << std::endl;
    for (auto i : v)
      out << i << std::endl;
    out << "New velocities (current_velocity=" << cur_vel
        << ", max_velocity=" << max_vel << ", max_accel=" << max_accel
        << ", max_decel=" << max_decel << "):" << std::endl;
    auto a =
        node.filterVelocities(v, cur_vel, time, max_vel, max_accel, max_decel);
    for (auto i : a)
      out << i << std::endl;
    return a;
  };

  std::vector<double> ret, ans;

  // starts at 0 and is increasing
  v = {1, 2, 5, 9};
  ans = {0, 2, 5, 8.06225774829855};
  ret = test(0, 4, 1000, 15, -15);
  ASSERT_EQ(ret, ans);

  // current velocity exceeds and drops
  v = {10, 12, 30, 51};
  ans = {15, 13.601470508735444, 15.000000000000002, 16.27882059609971};
  ret = test(15, 4, 1000, 15, -15);
  ASSERT_EQ(ret, ans);

  // velocity drops multiple times
  v = {10, 6, 10, 3};
  ans = {0, 6, 10, 3};
  ret = test(0, 4, 1000, 100, -100);
  ASSERT_EQ(ret, ans);

  // doesn't change
  v = {10, 10, 10, 10};
  ans = {10, 10, 10, 10};
  ret = test(10, 4, 1000, 100, -100);
  ASSERT_EQ(ret, ans);

  // invalid
  v = {-1.0 / 0.0, sqrt(-1), 231434, 1.0 / 0.0};
  ans = {1000, 1000, 1000, 1000};
  ret = test(1000, 4, 1000, 100, -100);
  ASSERT_EQ(ret, ans);
  out << "Adjusted Max Velocities" << std::endl;
  for (double d : v)
    out << d << std::endl;
}

TEST(PathOptimizationNode, getMaxA_longit) {
  PathOptimizationNode node;
  double velocity;
  double a_lateral;
  double a_longit;

  velocity = 10.4;
  a_lateral = 15.1;
  a_longit = node.getMaxA_longit(velocity, a_lateral);
  std::cout << "a_longit: " << a_longit << std::endl;
  ASSERT_DOUBLE_EQ(a_longit, 3.1957371091771924);
  
  velocity = 9.1;
  a_lateral = -15.1;
  a_longit = node.getMaxA_longit(velocity, a_lateral);

  ASSERT_DOUBLE_EQ(a_longit, 3.1957371091771924);

  velocity = 0.3;
  a_lateral = 14.9;
  a_longit = node.getMaxA_longit(velocity, a_lateral);
  ASSERT_DOUBLE_EQ(a_longit, 2.7753496968663423);

  velocity = 0.9;
  a_lateral = -14.9;
  a_longit = node.getMaxA_longit(velocity, a_lateral);
  ASSERT_DOUBLE_EQ(a_longit, 2.7753496968663423);

  velocity = 10.8;
  a_lateral = 0;
  a_longit = node.getMaxA_longit(velocity, a_lateral);
  ASSERT_DOUBLE_EQ(a_longit, 8.001412230575237);

  velocity = 0;
  a_lateral = 0;
  a_longit = node.getMaxA_longit(velocity, a_lateral);
  ASSERT_DOUBLE_EQ(a_longit, 7.959990224049994);

  velocity = 30.5;
  a_lateral = 0.5;
  a_longit = node.getMaxA_longit(velocity, a_lateral);
  ASSERT_DOUBLE_EQ(a_longit, 7.937797440348381);

  velocity = 30.5;
  a_lateral = -0.6;
  a_longit = node.getMaxA_longit(velocity, a_lateral);
  ASSERT_DOUBLE_EQ(a_longit, 7.937797440348381);

  // edge cases: out of bounds 
  velocity = 0.5;
  a_lateral = -21.6;
  a_longit = node.getMaxA_longit(velocity, a_lateral);
  ASSERT_DOUBLE_EQ(a_longit, 0.5587490382447899);

  velocity = 0.2;
  a_lateral = 20.3;
  a_longit = node.getMaxA_longit(velocity, a_lateral);
  ASSERT_DOUBLE_EQ(a_longit, 0.5587490382447899);

  velocity = 36.4;
  a_lateral = -0.6;
  a_longit = node.getMaxA_longit(velocity, a_lateral);
  ASSERT_DOUBLE_EQ(a_longit, 7.937797440348381);

  velocity = -10;
  a_lateral = -0.3;
  a_longit = node.getMaxA_longit(velocity, a_lateral);
  ASSERT_DOUBLE_EQ(a_longit, 7.959990224049994);

}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}