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
#include <limits>

using namespace utfr_dv::build_graph;
using namespace utfr_dv::compute_graph;
using namespace utfr_dv::ekf;

TEST(KNNSearchNodeTest, kNNTest1)
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
    ASSERT_EQ(false, node.globalKDTreePtr_ == nullptr);

    // check is added to past_detections_
    ASSERT_EQ(1, node.past_detections_.size());

    // check to see that map works

    ASSERT_EQ(1, node.cone_id_to_color_map_[0]);

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

TEST(EkfNodeTest, converlatlonalt2){
  //utfr_dv::build_graph::BuildGraphNode node;
    utfr_dv::ekf::EkfNode node;
    std::vector<double> resultPose = {2,3,100};
    std::vector<double> resultVector = {0,0,0};
    std::vector<double> vec2 = {4,2,70};
    std::vector<double> vec3 = {2.01,2.99,101};
    std::vector<double> vec2a = {-3514822.991557160,-4035041.925221907,9789303.558496917};
    std::vector<double> vec3a = {-27194.83195880818,-63755.89178277459,376.98191554099};
    ASSERT_EQ(resultVector,node.lla2enu(resultPose));
    ASSERT_EQ(resultVector,node.lla2enu(resultPose));
    std::vector<double> res1 = node.lla2enu(vec2);
    ASSERT_NEAR(vec2a[0],res1[0],100);
    ASSERT_NEAR(vec2a[1],res1[1],100);
    ASSERT_NEAR(vec2a[2],res1[2],100);
    std::vector<double> res2 = node.lla2enu(vec3);
    ASSERT_NEAR(vec3a[0],res2[0],1e-8);
    ASSERT_NEAR(vec3a[1],res2[1],1e-8);
    ASSERT_NEAR(vec3a[2],res2[2],1e-8);
}

TEST(EkfNodeTest, updatePositionTest){ 
    std::cout << "updatePositionTest" << std::endl;
    utfr_dv::ekf::EkfNode node;
    //Set random vehicle 
    node.vehicle_params_.inertia.mass = 700; // kg
    node.vehicle_params_.inertia.Izz = 1300; // kg*m^2
    node.vehicle_params_.tire.c_r = 50; // kN/rad
    node.vehicle_params_.tire.c_f = 50; // kN/rad
    node.vehicle_params_.kinematics.l_f = 1.5; // m
    node.vehicle_params_.kinematics.l_r = 1.5; // m
    node.vehicle_params_.kinematics.drag_coefficient = 0.9; // N*s/m
    node.vehicle_params_.kinematics.rolling_resistance_coefficient = 0.01; // N*s/m
    node.vehicle_params_.kinematics.cornering_stiffness = 100000; // N*m/rad
    node.vehicle_params_.kinematics.down_force_coefficient = -2.0; // N

    //Set random state
    node.current_state_.pose.pose.position.x = 0;
    node.current_state_.pose.pose.position.y = 0;
    node.current_state_.pose.pose.orientation = utfr_dv::util::yawToQuaternion(0);
    node.current_state_.vel.twist.linear.x = 0;
    node.current_state_.vel.twist.linear.y = 0;
    node.current_state_.vel.twist.angular.z = 0;
    // Sanity Test 1: No throttle, no brake, no steering (no movement)
    double throttle = 0;
    double brake = 0;
    double steering = 0;
    double dt = 0.1;
    node.kinematicBicycleModel(throttle, brake, steering, dt);

    std::cout << "Test 1: No throttle, no brake, no steering (no movement)"
          << "x: " << node.current_state_.pose.pose.position.x 
          << " y: " << node.current_state_.pose.pose.position.y 
          << " speed_x: " << node.current_state_.vel.twist.linear.x 
          << " speed_y: " << node.current_state_.vel.twist.linear.y << std::endl;


    // Sanity Test 2: No steering angle, throttle of 2 m/s^2 for 4 second
    std::cout <<"Test 2: No steering angle, throttle of 2 m/s^2 for 4 second" << std::endl;
    throttle = 2.0;
    brake = 0;
    steering = 0;
    dt = 0.1;
    for(int i = 0; i < 40; i++){
        if(node.current_state_.vel.twist.linear.x * node.current_state_.vel.twist.linear.x + node.current_state_.vel.twist.linear.y * node.current_state_.vel.twist.linear.y < 8.3 * 8.3)
            node.kinematicBicycleModel(throttle, brake, steering, dt);
        else
            node.dynamicBicycleModel(throttle, brake, steering, dt);
        std::cout<<"Time : "<<i*dt<<std::endl; 
        std::cout << "x: " << node.current_state_.pose.pose.position.x 
          << " y: " << node.current_state_.pose.pose.position.y 
          << " speed_x: " << node.current_state_.vel.twist.linear.x 
          << " speed_y: " << node.current_state_.vel.twist.linear.y  
          << " yaw: " << utfr_dv::util::quaternionToYaw(node.current_state_.pose.pose.orientation)
          << " velocty:" << sqrt(node.current_state_.vel.twist.linear.x * node.current_state_.vel.twist.linear.x + node.current_state_.vel.twist.linear.y * node.current_state_.vel.twist.linear.y) << std::endl;
    }

    
    // Test 3: 
    //Throttle of 2 m/s^2,  steering of pi/4 degrees for  4 seconds, should be around (-2.35, 4.48) with a yaw of -2.36 rad, vel of 3.15m/s
    std::cout<<"Test 3: Throttle of 2 m/s^2,  steering of pi/4 degrees for  4 seconds"<<std::endl;
    node.current_state_.pose.pose.position.x = 0;
    node.current_state_.pose.pose.position.y = 0;
    node.current_state_.pose.pose.orientation = utfr_dv::util::yawToQuaternion(0);
    node.current_state_.vel.twist.linear.x = 0;
    node.current_state_.vel.twist.linear.y = 0;
    node.current_state_.vel.twist.angular.z = 0;

    throttle = 2.0;
    brake = 0;
    steering = M_PI/4;
    dt = 0.1;
    for(int i = 0; i < 40; i++){
        if(node.current_state_.vel.twist.linear.x * node.current_state_.vel.twist.linear.x + node.current_state_.vel.twist.linear.y * node.current_state_.vel.twist.linear.y < 8.3 * 8.3)
            node.kinematicBicycleModel(throttle, brake, steering, dt);
        else
            node.dynamicBicycleModel(throttle, brake, steering, dt);
        std::cout<<"Time : "<<i*dt<<std::endl; 
        std::cout << "x: " << node.current_state_.pose.pose.position.x 
          << " y: " << node.current_state_.pose.pose.position.y 
          << " speed_x: " << node.current_state_.vel.twist.linear.x 
          << " speed_y: " << node.current_state_.vel.twist.linear.y  
          << " yaw: " << utfr_dv::util::quaternionToYaw(node.current_state_.pose.pose.orientation)
          << " velocty:" << sqrt(node.current_state_.vel.twist.linear.x * node.current_state_.vel.twist.linear.x + node.current_state_.vel.twist.linear.y * node.current_state_.vel.twist.linear.y) << std::endl;
    }
    



    //Throttle of 4 m/s^2, no steering, starting velocity is 10m/s 10+16 = 26m/s
    std::cout<<"Test 4: Throttle of 4 m/s^2, no steering, starting velocity is 10m/s"<<std::endl;
   
    node.current_state_.pose.pose.position.x = 0;
    node.current_state_.pose.pose.position.y = 0;
    node.current_state_.pose.pose.orientation = utfr_dv::util::yawToQuaternion(0);
    node.current_state_.vel.twist.linear.x = 10;
    node.current_state_.vel.twist.linear.y = 0;
    node.current_state_.vel.twist.angular.z = 0;

    throttle = 4.0;
    brake = 0;
    steering = 0;
    dt = 0.1;

    for(int i = 0; i < 40; i++){
        if(node.current_state_.vel.twist.linear.x * node.current_state_.vel.twist.linear.x + node.current_state_.vel.twist.linear.y * node.current_state_.vel.twist.linear.y < 8.3 * 8.3)
            node.kinematicBicycleModel(throttle, brake, steering, dt);
        else
            node.dynamicBicycleModel(throttle, brake, steering, dt);
        std::cout<<"Time : "<<(i + 1)*dt <<std::endl; 
        std::cout << "x: " << node.current_state_.pose.pose.position.x 
          << " y: " << node.current_state_.pose.pose.position.y 
          << " speed_x: " << node.current_state_.vel.twist.linear.x 
          << " speed_y: " << node.current_state_.vel.twist.linear.y  
          << " yaw: " << utfr_dv::util::quaternionToYaw(node.current_state_.pose.pose.orientation)
          << " velocty:" << sqrt(node.current_state_.vel.twist.linear.x * node.current_state_.vel.twist.linear.x + node.current_state_.vel.twist.linear.y * node.current_state_.vel.twist.linear.y) << std::endl;
    }

    //Throttle of 3 m/s^2,  steering of pi/6 degrees for 10 seconds 3 * 10 = 30 
    std::cout<<"Test 5: Throttle of 3 m/s^2,  steering of pi/3 degrees for  10 seconds"<<std::endl;
    node.current_state_.pose.pose.position.x = 0;
    node.current_state_.pose.pose.position.y = 0;
    node.current_state_.pose.pose.orientation = utfr_dv::util::yawToQuaternion(0);
    node.current_state_.vel.twist.linear.x = 0;
    node.current_state_.vel.twist.linear.y = 0;
    node.current_state_.vel.twist.angular.z = 0;

    throttle = 3.0;
    brake = 0;
    steering = M_PI/6;
    dt = 0.1;     
    for(int i = 0; i < 100; i++){
        if(node.current_state_.vel.twist.linear.x * node.current_state_.vel.twist.linear.x + node.current_state_.vel.twist.linear.y * node.current_state_.vel.twist.linear.y < 2 * 2)
            node.kinematicBicycleModel(throttle, brake, steering, dt);
        else
            node.dynamicBicycleModel(throttle, brake, steering, dt);
        std::cout<<"Time : "<<(i + 1)*dt <<std::endl; 
        std::cout << "x: " << node.current_state_.pose.pose.position.x 
          << " y: " << node.current_state_.pose.pose.position.y 
          << " speed_x: " << node.current_state_.vel.twist.linear.x 
          << " speed_y: " << node.current_state_.vel.twist.linear.y  
          << " yaw: " << utfr_dv::util::quaternionToYaw(node.current_state_.pose.pose.orientation)
          << " velocty:" << sqrt(node.current_state_.vel.twist.linear.x * node.current_state_.vel.twist.linear.x + node.current_state_.vel.twist.linear.y * node.current_state_.vel.twist.linear.y) << std::endl;
    }


}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
