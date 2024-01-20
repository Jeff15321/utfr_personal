/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: math.cpp
* auth: Kelvin Cui
* desc: math common functions unit tests
*/

#include <gtest/gtest.h>
#include <math.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace utfr_dv::util;

TEST(math, doubleEQ) {
  double a = 1.00;
  double b = 1.0;
  ASSERT_TRUE(doubleEQ(a, b));

  a = 2.0;
  b = 1.0;
  ASSERT_FALSE(doubleEQ(a, b));

  a = 1.000000023;
  b = 1.000000024;
  ASSERT_TRUE(doubleEQ(a, b));

  a = 1.00000023;
  b = 1.00000024;
  ASSERT_FALSE(doubleEQ(a, b));
}

TEST(Math, radToDeg) {
  double rad = 0;
  ASSERT_DOUBLE_EQ(radToDeg(rad), 0.0);
  rad = M_PI;
  ASSERT_DOUBLE_EQ(radToDeg(rad), 180.0);
  rad = 2 * M_PI;
  ASSERT_DOUBLE_EQ(radToDeg(rad), 360.0);
  rad = M_PI / 2;
  ASSERT_DOUBLE_EQ(radToDeg(rad), 90.0);
  rad = M_PI / 4;
  ASSERT_DOUBLE_EQ(radToDeg(rad), 45.0);
  rad = -M_PI;
  ASSERT_DOUBLE_EQ(radToDeg(rad), -180.0);
  rad = -2 * M_PI;
  ASSERT_DOUBLE_EQ(radToDeg(rad), -360.0);
  rad = -M_PI / 2;
  ASSERT_DOUBLE_EQ(radToDeg(rad), -90.0);
  rad = -M_PI / 4;
  ASSERT_DOUBLE_EQ(radToDeg(rad), -45.0);
  rad = 4 * M_PI / 3;
  ASSERT_DOUBLE_EQ(radToDeg(rad), 240.0);
}

TEST(Math, degToRad) {
  double deg = 0;
  ASSERT_DOUBLE_EQ(degToRad(deg), 0.0);
  deg = 120;
  ASSERT_DOUBLE_EQ(degToRad(deg), 2 * M_PI / 3);
  deg = 180;
  ASSERT_DOUBLE_EQ(degToRad(deg), M_PI);
  deg = 360;
  ASSERT_DOUBLE_EQ(degToRad(deg), 2 * M_PI);
  deg = 510;
  ASSERT_DOUBLE_EQ(degToRad(deg), 17 * M_PI / 6);

  deg = -20;
  ASSERT_DOUBLE_EQ(degToRad(deg), -M_PI / 9);
  deg = -45;
  ASSERT_DOUBLE_EQ(degToRad(deg), -M_PI / 4);
  deg = -90;
  ASSERT_DOUBLE_EQ(degToRad(deg), -M_PI / 2);
}

TEST(Math, convertLidarToCoordinates) {

  sensor_msgs::msg::LaserScan lidar_scan;
  lidar_scan.angle_min = 0.0;
  lidar_scan.angle_max = 2.0 * M_PI;

  // Build Lidar message:
  int lidar_points = 4;
  double angle_increment = (2 * M_PI) / lidar_points;

  lidar_scan.angle_increment = angle_increment;
  lidar_scan.ranges = {1.0, 2.0, 3.0, 4.0};

  double threshold = 3.5;
  std::vector<utfr_msgs::msg::Cone> result =
      convertLidarToCoordinates(lidar_scan, threshold);

  const long unsigned int expected_size = 3;
  ASSERT_EQ(result.size(), expected_size);

  ASSERT_NEAR(result[0].pos.x, 1.0, 1e-6);
  ASSERT_NEAR(result[0].pos.y, 0.0, 1e-6);

  ASSERT_NEAR(result[1].pos.x, 0.0, 1e-6);
  ASSERT_NEAR(result[1].pos.y, 2.0, 1e-6);

  ASSERT_NEAR(result[2].pos.x, -3.0, 1e-6);
  ASSERT_NEAR(result[2].pos.y, 0.0, 1e-6);

  // Case 1: Angles 0 to 360
  lidar_scan.angle_min = 0.0;
  lidar_scan.angle_max = 2 * M_PI;

  int lidar_points1 = 10;
  double angle_increment1 = 2 * M_PI / lidar_points1;

  lidar_scan.angle_increment = angle_increment1;
  lidar_scan.ranges = {1.31276231, 3.89812345, 0.29384676,  3.11267283,
                       2.64238054, 2.73485001, 2.135912547, 3.52482117,
                       1.12634345, 0.29186443};

  double threshold_new = 4;
  std::vector<utfr_msgs::msg::Cone> result1 =
      convertLidarToCoordinates(lidar_scan, threshold_new);

  const long unsigned int expected_size1 = 10;
  ASSERT_EQ(result1.size(), expected_size1);

  // cone 0
  ASSERT_NEAR(result1[0].pos.x, 1.31276226, 1e-6);
  ASSERT_NEAR(result1[0].pos.y, 0.0, 1e-6);
  // cone 1
  ASSERT_NEAR(result1[1].pos.x, 3.15364812, 1e-6);
  ASSERT_NEAR(result1[1].pos.y, 2.29125947, 1e-6);
  // cone 2
  ASSERT_NEAR(result1[2].pos.x, 0.090803643, 1e-6);
  ASSERT_NEAR(result1[2].pos.y, 0.279464876, 1e-6);
  // cone 3
  ASSERT_NEAR(result1[3].pos.x, -0.961868802, 1e-6);
  ASSERT_NEAR(result1[3].pos.y, 2.96032778, 1e-6);
  // cone 4
  ASSERT_NEAR(result1[4].pos.x, -2.13773076, 1e-6);
  ASSERT_NEAR(result1[4].pos.y, 1.55315231, 1e-6);
  // cone 5
  ASSERT_NEAR(result1[5].pos.x, -2.73485001, 1e-6);
  ASSERT_NEAR(result1[5].pos.y, 0.0, 1e-6);
  // cone 6
  ASSERT_NEAR(result1[6].pos.x, -1.72798957, 1e-6);
  ASSERT_NEAR(result1[6].pos.y, -1.25545791, 1e-6);
  // cone 7
  ASSERT_NEAR(result1[7].pos.x, -1.08922964, 1e-6);
  ASSERT_NEAR(result1[7].pos.y, -3.35230414, 1e-6);
  // cone 8
  ASSERT_NEAR(result1[8].pos.x, 0.348059268, 1e-6);
  ASSERT_NEAR(result1[8].pos.y, -1.07121628, 1e-6);
  // cone 9
  ASSERT_NEAR(result1[9].pos.x, 0.236123284, 1e-6);
  ASSERT_NEAR(result1[9].pos.y, -0.171553608, 1e-6);

  // Case 2: Angles -90 to 90
  lidar_scan.angle_min = -M_PI / 2;
  lidar_scan.angle_max = M_PI / 2;

  int lidar_points2 = 10;
  double angle_increment2 = M_PI / lidar_points2;

  lidar_scan.angle_increment = angle_increment2;
  lidar_scan.ranges = {0.924365835, 3.413582573, 2.51439135, 1.439572457,
                       1.98431421,  3.49832869,  3.41578217, 0.98761403,
                       2.22435655,  0.43793127,  1.09137246};
  std::vector<utfr_msgs::msg::Cone> result2 =
      convertLidarToCoordinates(lidar_scan, threshold_new);

  const long unsigned int expected_size2 = 11;
  ASSERT_EQ(result2.size(), expected_size2);

  // cone 0
  ASSERT_NEAR(result2[0].pos.x, 0.0, 1e-6);
  ASSERT_NEAR(result2[0].pos.y, -0.924365835, 1e-6);
  // cone 1
  ASSERT_NEAR(result2[1].pos.x, 1.05485503, 1e-6);
  ASSERT_NEAR(result2[1].pos.y, -3.24650995, 1e-6);
  // cone 2
  ASSERT_NEAR(result2[2].pos.x, 1.47792215, 1e-6);
  ASSERT_NEAR(result2[2].pos.y, -2.03418533, 1e-6);
  // cone 3
  ASSERT_NEAR(result2[3].pos.x, 1.16463858, 1e-6);
  ASSERT_NEAR(result2[3].pos.y, -0.84615946, 1e-6);
  // cone 4
  ASSERT_NEAR(result2[4].pos.x, 1.88719496, 1e-6);
  ASSERT_NEAR(result2[4].pos.y, -0.613186813, 1e-6);
  // cone 5
  ASSERT_NEAR(result2[5].pos.x, 3.49832869, 1e-6);
  ASSERT_NEAR(result2[5].pos.y, 0.0, 1e-6);
  // cone 6
  ASSERT_NEAR(result2[6].pos.x, 3.24860189, 1e-6);
  ASSERT_NEAR(result2[6].pos.y, 1.05553474, 1e-6);
  // cone 7
  ASSERT_NEAR(result2[7].pos.x, 0.798996534, 1e-6);
  ASSERT_NEAR(result2[7].pos.y, 0.580504962, 1e-6);
  // cone 8
  ASSERT_NEAR(result2[8].pos.x, 1.30744398, 1e-6);
  ASSERT_NEAR(result2[8].pos.y, 1.79954225, 1e-6);
  // cone 9
  ASSERT_NEAR(result2[9].pos.x, 0.135328205, 1e-6);
  ASSERT_NEAR(result2[9].pos.y, 0.416497388, 1e-6);
  // cone 10
  ASSERT_NEAR(result2[10].pos.x, 0.0, 1e-6);
  ASSERT_NEAR(result2[10].pos.y, 1.09137246, 1e-6);
}

TEST(Math, convertLLAtoNED) {
  // regular test 1
  geometry_msgs::msg::Vector3 lla;
  lla.x = 45.976;
  lla.y = 7.658;
  lla.z = 4531;

  geometry_msgs::msg::Vector3 lla0;
  lla0.x = 46.017;
  lla0.y = 7.750;
  lla0.z = 1673;

  geometry_msgs::msg::Vector3 temp1 = convertLLAtoNED(lla, lla0);
  ASSERT_NEAR(temp1.x, -4556.32151384457, 1e-8);
  ASSERT_NEAR(temp1.y, -7134.75719597987, 1e-8);
  ASSERT_NEAR(temp1.z, -2852.39042394501, 1e-8);

  // regular test 2
  geometry_msgs::msg::Vector3 lla2;
  lla2.x = -7.397357;
  lla2.y = 109.846979;
  lla2.z = 0;

  geometry_msgs::msg::Vector3 lla02;
  lla02.x = -7.405680150087858;
  lla02.y = 109.86242302211231;
  lla02.z = 0;

  geometry_msgs::msg::Vector3 temp2 = convertLLAtoNED(lla2, lla02);
  ASSERT_NEAR(temp2.x, 920.450056632922, 1e-8);
  ASSERT_NEAR(temp2.y, -1705.00636382082, 1e-8);
  ASSERT_NEAR(temp2.z, 0.294731949886639, 1e-8);

  // edge case 1
  geometry_msgs::msg::Vector3 lla_1;
  lla_1.x = 36.162628;
  lla_1.y = 127.833212;
  lla_1.z = 0;

  geometry_msgs::msg::Vector3 lla0_1;
  lla0_1.x = 36.162628;
  lla0_1.y = 127.833212;
  lla0_1.z = 0;

  geometry_msgs::msg::Vector3 edge1 = convertLLAtoNED(lla_1, lla0_1);
  ASSERT_NEAR(edge1.x, 0, 1e-8);
  ASSERT_NEAR(edge1.y, 0, 1e-8);
  ASSERT_NEAR(edge1.z, 0, 1e-8);

  // edge case 2
  geometry_msgs::msg::Vector3 lla_2;
  lla_2.x = 0;
  lla_2.y = 0;
  lla_2.z = 0;

  geometry_msgs::msg::Vector3 lla0_2;
  lla0_2.x = 0.000123;
  lla0_2.y = 0.000213;
  lla0_2.z = 0;

  geometry_msgs::msg::Vector3 edge2 = convertLLAtoNED(lla_2, lla0_2);
  ASSERT_NEAR(edge2.x, -13.600635926045872, 1e-8);
  ASSERT_NEAR(edge2.y, -23.7110515388584, 1e-8);
  ASSERT_NEAR(edge2.z, 5.8672390872005e-05, 1e-8);

  // edge case 3
  geometry_msgs::msg::Vector3 lla_3;
  lla_3.x = 0.000123;
  lla_3.y = 0.000213;
  lla_3.z = 0;

  geometry_msgs::msg::Vector3 lla0_3;
  lla0_3.x = 0;
  lla0_3.y = 0;
  lla0_3.z = 0;

  geometry_msgs::msg::Vector3 edge3 = convertLLAtoNED(lla_3, lla0_3);
  ASSERT_NEAR(edge3.x, 13.600635926045872, 1e-8);
  ASSERT_NEAR(edge3.y, 23.7110515388584, 1e-8);
  ASSERT_NEAR(edge3.z, 5.8672390872005e-05, 1e-8);
}

TEST(Math, convertNEDtoLLA) {
  // Case 1
  geometry_msgs::msg::Vector3 lla01;
  lla01.x = 44.532;
  lla01.y = -72.782;
  lla01.z = 1699;

  geometry_msgs::msg::Vector3 ned1;
  ned1.x = 1334.3;
  ned1.y = -2543.6;
  ned1.z = 359.65;

  geometry_msgs::msg::Vector3 temp1 = convertNEDtoLLA(ned1, lla01);
  ASSERT_NEAR(temp1.x, 44.54400043376972, 1e-8);
  ASSERT_NEAR(temp1.y, -72.81400044707391, 1e-8);
  ASSERT_NEAR(temp1.z, 1339.9960363217026, 1e-8);

  // Case 2
  geometry_msgs::msg::Vector3 lla02;
  lla02.x = 0;
  lla02.y = 0;
  lla02.z = 0;

  geometry_msgs::msg::Vector3 ned2;
  ned2.x = 0;
  ned2.y = 0;
  ned2.z = 0;

  geometry_msgs::msg::Vector3 temp2 = convertNEDtoLLA(ned2, lla02);
  ASSERT_NEAR(temp2.x, 0, 1e-8);
  ASSERT_NEAR(temp2.y, 0, 1e-8);
  ASSERT_NEAR(temp2.z, 0, 1e-8);

  // Case 3
  geometry_msgs::msg::Vector3 lla03;
  lla03.x = 45;
  lla03.y = 50;
  lla03.z = 72;

  geometry_msgs::msg::Vector3 ned3;
  ned3.x = 85;
  ned3.y = 96;
  ned3.z = 45;

  geometry_msgs::msg::Vector3 temp3 = convertNEDtoLLA(ned3, lla03);
  ASSERT_NEAR(temp3.x, 45.000764847953995, 1e-8);
  ASSERT_NEAR(temp3.y, 50.00121756150896, 1e-8);
  ASSERT_NEAR(temp3.z, 27.001288597072374, 1e-8);

  // Case 4
  geometry_msgs::msg::Vector3 lla04;
  lla04.x = 90;
  lla04.y = 45;
  lla04.z = 0;

  geometry_msgs::msg::Vector3 ned4;
  ned4.x = 10000;
  ned4.y = 20007890;
  ned4.z = 350987265;

  geometry_msgs::msg::Vector3 temp4 = convertNEDtoLLA(ned4, lla04);
  ASSERT_NEAR(temp4.x, -86.67780780653034, 1e-8);
  ASSERT_NEAR(temp4.y, 135.02863659023632, 1e-8);
  ASSERT_NEAR(temp4.z, 338853989.64317125, 1e-8);

  // Case 5
  geometry_msgs::msg::Vector3 lla05;
  lla05.x = 90;
  lla05.y = 90;
  lla05.z = 0;

  geometry_msgs::msg::Vector3 ned5;
  ned5.x = -27891;
  ned5.y = -50208;
  ned5.z = -45;

  geometry_msgs::msg::Vector3 temp5 = convertNEDtoLLA(ned5, lla05);
  ASSERT_NEAR(temp5.x, 89.48580196032957, 1e-8);
  ASSERT_NEAR(temp5.y, 29.052569334270082, 1e-8);
  ASSERT_NEAR(temp5.z, 302.72433130156304, 1e-8);
}

TEST(math, getCrosstrackError) {
  utfr_msgs::msg::TrajectoryPoint p1, p2;
  p1.pos.x = 0;
  p1.pos.y = 0;

  p2.pos.x = 1;
  p2.pos.y = 1;

  utfr_msgs::msg::EgoState ego;
  ego.pose.pose.position.x = 0;
  ego.pose.pose.position.y = 1;

  ASSERT_NEAR(getCrosstrackError(p1, p2, ego), 0.707107, 1e-3);

  // Test Case: 1
  p1.pos.x = 791;
  p1.pos.y = 44;
  p2.pos.x = 56;
  p2.pos.y = 699;
  ego.pose.pose.position.x = 965;
  ego.pose.pose.position.y = 531;
  ASSERT_NEAR(getCrosstrackError(p1, p2, ego), 479.342434, 1e-3);

  // Test Case: 2
  p1.pos.x = 618;
  p1.pos.y = 603;
  p2.pos.x = 484;
  p2.pos.y = 477;
  ego.pose.pose.position.x = 699;
  ego.pose.pose.position.y = 842;
  ASSERT_NEAR(getCrosstrackError(p1, p2, ego), 118.629011, 1e-3);

  // Test Case: 3
  p1.pos.x = 791;
  p1.pos.y = 294;
  p2.pos.x = 874;
  p2.pos.y = 117;
  ego.pose.pose.position.x = 485;
  ego.pose.pose.position.y = 214;
  ASSERT_NEAR(getCrosstrackError(p1, p2, ego), 311.016827, 1e-3);

  // Test Case: 4
  p1.pos.x = 284;
  p1.pos.y = 655;
  p2.pos.x = 213;
  p2.pos.y = 810;
  ego.pose.pose.position.x = 155;
  ego.pose.pose.position.y = 56;
  ASSERT_NEAR(getCrosstrackError(p1, p2, ego), 366.736486, 1e-3);

  // Test Case: 5
  p1.pos.x = 246;
  p1.pos.y = 804;
  p2.pos.x = 493;
  p2.pos.y = 401;
  ego.pose.pose.position.x = 357;
  ego.pose.pose.position.y = 44;
  ASSERT_NEAR(getCrosstrackError(p1, p2, ego), 302.508425, 1e-3);

  // Test Case: 6
  p1.pos.x = 151;
  p1.pos.y = 462;
  p2.pos.x = 654;
  p2.pos.y = 440;
  ego.pose.pose.position.x = 666;
  ego.pose.pose.position.y = 460;
  ASSERT_NEAR(getCrosstrackError(p1, p2, ego), 20.505247, 1e-3);

  // Test Case: 7
  p1.pos.x = 756;
  p1.pos.y = 469;
  p2.pos.x = 393;
  p2.pos.y = 46;
  ego.pose.pose.position.x = 986;
  ego.pose.pose.position.y = 770;
  ASSERT_NEAR(getCrosstrackError(p1, p2, ego), 21.479974, 1e-3);

  // Test Case: 8
  p1.pos.x = 42;
  p1.pos.y = 758;
  p2.pos.x = 133;
  p2.pos.y = 810;
  ego.pose.pose.position.x = 158;
  ego.pose.pose.position.y = 95;
  ASSERT_NEAR(getCrosstrackError(p1, p2, ego), 633.197320, 1e-3);

  // Test Case: 9
  p1.pos.x = 139;
  p1.pos.y = 611;
  p2.pos.x = 107;
  p2.pos.y = 257;
  ego.pose.pose.position.x = 602;
  ego.pose.pose.position.y = 176;
  ASSERT_NEAR(getCrosstrackError(p1, p2, ego), 500.282204, 1e-3);

  // Test Case: 10
  p1.pos.x = 924;
  p1.pos.y = 380;
  p2.pos.x = 276;
  p2.pos.y = 420;
  ego.pose.pose.position.x = 972;
  ego.pose.pose.position.y = 973;
  ASSERT_NEAR(getCrosstrackError(p1, p2, ego), 594.830770, 1e-3);
}
TEST(math, egoHelperTest)
{
  ego_state newEgo; 
  newEgo.pose.pose.postion.x = -1;
  newEgo.pose.pose.postion.y = 1;
  newEgo.vel.twist.linear.x = -2;
  newEgo.vel.twist.linear.y = 2;
  ASSERT_EQ(-1, egoHelper(newEgo ,"pos_x"));
  ASSERT_EQ(1, egoHelper(newEgo ,"pos_y"));
  ASSERT_EQ(2, egoHelper(newEgo ,"vel_y"));
  ASSERT_EQ(-2, egoHelper(newEgo ,"vel_x"));
  newEgo.pose.pose.postion.x = 23;
  newEgo.pose.pose.postion.y = -331;
  newEgo.vel.twist.linear.x = 0.2;
  newEgo.vel.twist.linear.y = -0.1012;
  ASSERT_EQ(23, egoHelper(newEgo ,"pos_x"));
  ASSERT_EQ(-331, egoHelper(newEgo ,"pos_y"));
  ASSERT_EQ(-0.1012, egoHelper(newEgo ,"vel_y"));
  ASSERT_EQ(0.2, egoHelper(newEgo ,"vel_x"));
  newEgo.pose.pose.postion.x = 23;
  newEgo.pose.pose.postion.y = -331;
  newEgo.vel.twist.linear.x = 0.2;
  newEgo.vel.twist.linear.y = -0.1012;
  ASSERT_EQ(-FLT_MAX, egoHelper(newEgo ,"os_x"));
  ASSERT_EQ(-FLT_MAX, egoHelper(newEgo ,"posy"));
  ASSERT_EQ(-FLT_MAX, egoHelper(newEgo ,"vl_y"));
  ASSERT_EQ(-FLT_MAX, egoHelper(newEgo ,"vel_"));
  newEgo.steering_angle = 7;
  ASSERT_EQ(7, egoHelper(newEgo ,"steering_angle"));
  newEgo.steering_angle = -97;
  ASSERT_EQ(-97, egoHelper(newEgo ,"steering_angle"));
  newEgo.steering_angle = 7;
  ASSERT_EQ(-FLT_MAX, egoHelper(newEgo ,"steerig_angle"));
  
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
