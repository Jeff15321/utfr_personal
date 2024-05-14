/*
██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██
*
* file: mapping_test.cpp
* auth: Arthur Xu
* desc: graph slam unit tests
*/

#include <build_graph_node.hpp>
#include <compute_graph_node.hpp>
#include <ekf_node.hpp>
#include <gtest/gtest.h>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <Eigen/Core>

using namespace utfr_dv::build_graph;
using namespace utfr_dv::compute_graph;
using namespace utfr_dv::ekf;

// initialize ROS
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}