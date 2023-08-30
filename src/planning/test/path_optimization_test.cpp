/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: path_optimization_test.cpp
* auth: Justin Lim
* desc: tests for path optimization node
*/

#include <gtest/gtest.h>
#include <math.h>
#include <path_optimization_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utfr_common/math.hpp>
#include <path_optimization_node.hpp>

using namespace utfr_dv::path_optimization;

TEST(PathOptimizationNode, calculateVelocities){
  PathOptimizationNode node;
  utfr_msgs::msg::ParametricSpline spline;
  std::vector<double> curvatures;
  double L;
  int n;
  double a_lateral;
  std::vector<double> velocities;
  
  spline.x_params = {5.5,6,82,12,-2,-13};
  spline.y_params = {-323,23,-32.1,0,5,-23};
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
  0.000000000029179596973419697748607865694976822544537409243048386997543275356292724609375
  };

  L = 10;
  n = 11;
  a_lateral = 15;
  velocities  = node.calculateVelocities(spline,L,n,a_lateral);
  ASSERT_EQ(curvatures.size(), velocities.size());
  int size = curvatures.size();
  for(int i = 0; i < size; i++){
    double calculated_velocity = sqrt(a_lateral/curvatures[i]);
    ASSERT_DOUBLE_EQ(calculated_velocity, velocities[i]);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}