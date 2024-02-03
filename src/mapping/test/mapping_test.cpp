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
    node.stateEstimationCB(ego);

    // Custom failure message
    ASSERT_EQ(0, node.current_state_.pose.pose.position.x);

    // First round of cone detections
    utfr_msgs::msg::ConeDetections cones;
    utfr_msgs::msg::ConeDetections empty;

    utfr_msgs::msg::Cone blue_cone;
    blue_cone.pos.x = 1;
    blue_cone.pos.y = 1;
    blue_cone.pos.z = 0;
    blue_cone.type = 1;

    cones.left_cones.push_back(blue_cone);

    //KNN once
    node.KNN(cones);

    // check if tree created
    ASSERT_EQ(false, node.globalKDTreePtr == nullptr);

    // check is added to past_detections_
    ASSERT_EQ(1, node.past_detections_.size());

    cones = empty;

    utfr_msgs::msg::Cone yellow_cone;
    yellow_cone.pos.x = -1;
    yellow_cone.pos.y = -1;
    yellow_cone.pos.z = 0;
    yellow_cone.type = 2;
    cones.right_cones.push_back(yellow_cone);

    // run one time to see if it will not be added to past detections, but added to potential
    node.KNN(cones);

    ASSERT_EQ(2, node.past_detections_.size());
    ASSERT_EQ(0, node.potential_cones_.size());

    cones = empty;

    // run one more time to see if added to potential_cones_ again
    utfr_msgs::msg::Cone yellow_cone2;
    yellow_cone2.pos.x = -1.3;
    yellow_cone2.pos.y = -1.3;
    yellow_cone2.pos.z = 0;
    yellow_cone2.type = 2;
    cones.right_cones.push_back(yellow_cone2);

    node.KNN(cones);

    ASSERT_EQ(1, node.potential_cones_.size());

    cones = empty;

    // run one more time to see if added to past_detections_
    utfr_msgs::msg::Cone yellow_cone3;
    yellow_cone3.pos.x = -1.3;
    yellow_cone3.pos.y = -1.3;
    yellow_cone3.pos.z = 0;
    yellow_cone3.type = 2;
    cones.right_cones.push_back(yellow_cone3);

    node.KNN(cones);

    ASSERT_EQ(2, node.past_detections_.size());

    ASSERT_EQ(2,node.potential_cones_.size());

    cones  = empty;

    // CHECK IF KNN WORKS, CONE SHOULD NOT BE ADDED

    utfr_msgs::msg::Cone yellow_cone4;
    yellow_cone4.pos.x = -1.3;
    yellow_cone4.pos.y = -1.3;
    yellow_cone4.pos.z = 0;
    yellow_cone4.type = 2;
    cones.right_cones.push_back(yellow_cone4);

    node.KNN(cones);

    cones = empty;

    ASSERT_EQ(0,node.potential_cones_.size());

    ASSERT_EQ(3, node.past_detections_.size());

    node.graphSLAM();


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


TEST(BuildGraphNodeTest, loopClosureTest1)
{
   // Setup
   utfr_dv::build_graph::BuildGraphNode node;
   ASSERT_EQ(false, node.loop_closed_);


   // First round of cone detections
   utfr_msgs::msg::Cone blue_cone;
   blue_cone.type = 1;
   utfr_msgs::msg::Cone orange_cone;
   orange_cone.type = 4;
   node.past_detections_.push_back(std::make_pair(1, blue_cone));
   node.past_detections_.push_back(std::make_pair(2, orange_cone));
   std::vector<int> detected_cone_ids;
   detected_cone_ids.push_back(1);
   detected_cone_ids.push_back(2);


   // Run loop closure
   node.loopClosure(detected_cone_ids);
   ASSERT_EQ(false, node.loop_closed_);
   ASSERT_EQ(2, node.landmarkedID_);
   ASSERT_EQ(true, node.landmarked_);


   // Run loop closure again with the same cones
   node.loopClosure(detected_cone_ids);
   ASSERT_EQ(false, node.loop_closed_);
   ASSERT_EQ(2, node.landmarkedID_);
   ASSERT_EQ(true, node.landmarked_);
   ASSERT_EQ(false, node.out_of_frame_);


   // Orange cone is now gone from view   
   detected_cone_ids.erase(std::remove(detected_cone_ids.begin(), detected_cone_ids.end(), 2), detected_cone_ids.end());
   node.loopClosure(detected_cone_ids);
   ASSERT_EQ(false, node.loop_closed_);
   ASSERT_EQ(2, node.landmarkedID_);
   ASSERT_EQ(true, node.landmarked_);
   ASSERT_EQ(true, node.out_of_frame_);


   // Orange cone is back in view, close loop
   detected_cone_ids.push_back(2);
   node.loopClosure(detected_cone_ids);
   ASSERT_EQ(false, node.loop_closed_);
   ASSERT_EQ(2, node.landmarkedID_);
   ASSERT_EQ(true, node.landmarked_);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
