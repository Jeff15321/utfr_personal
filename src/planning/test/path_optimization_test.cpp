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
  utfr_msgs::msg::ParametricSpline spline;

  std::ofstream out("test_output.txt");

  auto test = [&out, &spline, &node](double cur_vel, double time, double max_vel,
                               double max_accel, double max_decel){
    out << "x:{";
    for(int i = 0; i < 6; i++){
      out << spline.x_params[i] << (i==5? "" : ",");
    }
    out << "}, ";
    out << "y:{";
    for(int i = 0; i < 6; i++){
      out << spline.y_params[i] << (i==5? "" : ",");
    }
    out << "}" << std::endl;
    auto v = node.calculateVelocities(spline, 4, 5, 3);
    out << "Max velocities:" << std::endl;
    for(auto i : v) out << i << std::endl;
    out << "New velocities (current_velocity=" << cur_vel << ", max_velocity="
              << max_vel << ", max_accel=" << max_accel << ", max_decel=" << max_decel 
              << "):" << std::endl; 
    auto a = node.filterVelocities(v, cur_vel, time, max_vel, max_accel, max_decel);
    for(auto i : a) out << i << std::endl;
    return node.filterVelocities(v, cur_vel, time, max_vel, max_accel, max_decel);
  };
  
  std::vector<double> ret, ans;

  spline.x_params = {1,2,3,4,5,6};
  spline.y_params = {6,5,4,3,2,1};

  ans = {
    0,
    5.4772255750516611882972028979565948247909545898438,
    7.7459666924148340427791481488384306430816650390625,
    9.486832980505138124271979904733598232269287109375,
    10.954451150103322376594405795913189649581909179688
  };
  ret = test(0, 4, 1000, 15, -15); // normal case
  ASSERT_EQ(ret, ans);

  ans = {
    100,
    99.8498873309329297853764728643000125885009765625,
    100,
    100.1498876684342604903577012009918689727783203125,
    100.299551344958672416396439075469970703125

  };
  ret = test(100, 4, 1000, 15, -15); // current velocity exceeds
  ASSERT_EQ(ret, ans);

  spline.x_params = {1,2,-3,-4,-5,6};
  spline.y_params = {6,5,-44,-43,2,1};
  
  ans = {
    0,
    14.1421356237309510106570087373256683349609375,
    7.361880547095847049376970971934497356414794921875,
    15.943565636008409924784245959017425775527954101562,
    21.3119047761974655941230594180524349212646484375
  };
  ret = test(0, 4, 1000, 100, -100); // velocity drops in the middle
  ASSERT_EQ(ret, ans);

  ans = {
    100,
    98.9949493661166570745990611612796783447265625,
    97.979589711327122358852648176252841949462890625,
    98.9949493661166570745990611612796783447265625,
    100
  };
  ret = test(100, 4, 1000, 100, -100); // current velocity exceeds & velocity drops
  ASSERT_EQ(ret, ans);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}