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

    // Custom failure message
    ASSERT_EQ(0, node.current_state_.pose.pose.position.x); //<< "Unexpected value for position.x";

    // Additional logging
    //std::cout << "Debug info: " << node.current_state_.pose.pose.position.x << std::endl;

    // First round of cone detections
    // utfr_msgs::msg::ConeDetections cones;

    // utfr_msgs::msg::Cone blue_cone;
    // blue_cone.pos.x = 1;
    // blue_cone.pos.y = 1;
    // blue_cone.pos.z = 0;
    // blue_cone.type = 1;

    //utfr_msgs::msg::Cone yellow_cone;
    //yellow_cone.pos.x = -1;
    //yellow_cone.pos.y = -1;
    //yellow_cone.pos.z = 0;
    //yellow_cone.type = 2;

    // cones.left_cones.push_back(blue_cone);
    //cones.right_cones.push_back(yellow_cone);

    //KNN once
    // node.KNN(cones);

    // Assert after a run
    // ASSERT_EQ(1, node.past_detections_.size());

    // Assert after three more runs
    // ASSERT_EQ(false, globalKDTreePtr == nullptr);

    // Detect a new cone
    // utfr_msgs::msg::Cone yellow_cone2;
    // yellow_cone2.pos.x = -1;
    // yellow_cone2.pos.y = 1;
    // yellow_cone2.pos.z = 0;
    // yellow_cone2.type = 2;
    // cones.right_cones.clear();
    // cones.right_cones.push_back(yellow_cone2);

    // Run once
    // node.KNN(cones);

    // Assert after three more runs
    // ASSERT_EQ(3, node.past_detections_.size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
