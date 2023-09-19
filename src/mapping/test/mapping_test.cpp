/*
██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██
*
* file: mapping_test.cpp
* auth: Arthur Xu
* desc: mapping unit tests
*/


#include <gtest/gtest.h>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <build_graph_node.hpp>
#include <compute_graph_node.hpp>
#include <ekf_node.hpp>

using namespace utfr_dv::build_graph;
using namespace utfr_dv::compute_graph;
using namespace utfr_dv::ekf;

TEST(BuildGraphNodeTest, kNNTest1)
{
    // Setup
    utfr_dv::build_graph::BuildGraphNode node;
    utfr_msgs::msg::EgoState ego;
    ego.pose.pose.position.x = 0;
    ego.pose.pose.position.y = 0;
    ego.pose.pose.position.z = 0;
    ego.pose.pose.orientation = utfr_dv::util::yawToQuaternion(0);

    node.current_state_ = ego;
    ASSERT_EQ(0, node.current_state_.pose.pose.position.x);

    // First round of cone detections
    utfr_msgs::msg::ConeDetections cones;

    utfr_msgs::msg::ConeDetection blue_cone;
    cone.pose.pose.position.x = 1;
    cone.pose.pose.position.y = 1;
    cone.pose.pose.position.z = 0;
    cone.pose.pose.type = 1;

    utfr_msgs::msg::ConeDetection yellow_cone;
    cone.pose.pose.position.x = -1;
    cone.pose.pose.position.y = -1;
    cone.pose.pose.position.z = 0;
    cone.pose.pose.type = 2;

    cones.left_cones.push_back(blue_cone);
    cones.right_cones.push_back(yellow_cone);
    node.KNN(cones);
    ASSERT_EQ(1, node.current_cone_map_.left_cones.size());
    ASSERT_EQ(1, node.current_cone_map_.right_cones.size());

    // We detect the same cones again, but nothing should happen, as we
    // already know of their existences.

    node.KNN(cones);
    ASSERT_EQ(1, node.current_cone_map_.left_cones.size());
    ASSERT_EQ(1, node.current_cone_map_.right_cones.size());

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}