/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: build_graph_node.cpp
* auth: Arthur Xu
* desc: build graph node class
*/

#include <build_graph_node.hpp>

namespace utfr_dv {
namespace build_graph {

BuildGraphNode::BuildGraphNode() : Node("build_graph_node") {
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

void BuildGraphNode::initParams() {}

void BuildGraphNode::initSubscribers() {
  cone_detection_subscriber_ =
      this->create_subscription<utfr_msgs::msg::ConeDetections>(
          topics::kConeDetections, 1,
          std::bind(&BuildGraphNode::coneDetectionCB, this,
                    std::placeholders::_1));

  state_estimation_subscriber_ =
      this->create_subscription<utfr_msgs::msg::EgoState>(
          topics::kEgoState, 1,
          std::bind(&BuildGraphNode::stateEstimationCB, this,
                    std::placeholders::_1));
}

void BuildGraphNode::initPublishers() {
  pose_graph_publisher_ =
      this->create_publisher<utfr_msgs::msg::PoseGraph>(topics::kPoseGraph, 10);
}

void BuildGraphNode::initTimers() {}

void BuildGraphNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kMappingBuildHeartbeat, 10);
}

void BuildGraphNode::coneDetectionCB(const utfr_msgs::msg::ConeDetections msg) {
}

void BuildGraphNode::stateEstimationCB(const utfr_msgs::msg::EgoState msg) {}

std::vector<int> BuildGraphNode::KNN(const utfr_msgs::msg::ConeDetections &cones) {}

bool isLargeOrangeCone(uint8 coneID) {
    if (coneID == 4) {
        return true;
    }
    return false;
}

void BuildGraphNode::loopClosure(const std::vector<int> &cones) {

    while (loop_closed == false) {
        for (int coneID : cones) { // Go through cone list, 
            if (isLargeOrangeCone(coneID) && !landmarked) { // No landmark cone yet  
                landmarkedID = coneID; // Set first large orange cone listed to be landmark cone
                landmarked = true; 
            }

            if (landmarked == true && out_of_frame == false) { // Cone has been landmarked and still in frame for the first time 
                auto seen_status = std::find(coneIDs.begin(), coneIDs.end(), landmarkedID);

                if (seen_status == coneIDs.end()) { // If cone not in frame anymore
                    out_of_frame = true; // Set out of frame to be true
                }
            }

            if (landmarked == true && out_of_frame == true) { // Cone has been landmarked and is no longer in frame
                auto seen_status = std::find(coneIDs.begin(), coneIDs.end(), landmarkedID); // Check if cone appears in frame again

                if (seen_status != coneIDs.end()) { // If cone in frame again
                    loop_closed = true; // Car has returned back to landmark cone position, made full loop
                }
            }
        }
    }
}


void BuildGraphNode::buildGraph() {}

} // namespace build_graph
} // namespace utfr_dv
