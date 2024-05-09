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

#include <build_graph_node.hpp>
#include <compute_graph_node.hpp>
#include <ekf_node.hpp>
#include <gtest/gtest.h>
#include <math.h>
#include <rclcpp/rclcpp.hpp>

using namespace utfr_dv::build_graph;
using namespace utfr_dv::compute_graph;
using namespace utfr_dv::ekf;

// initialize ROS
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}