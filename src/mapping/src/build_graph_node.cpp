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

int landmarkedID = -1;
bool landmarked = false;

bool isLargeOrangeCone(uint8 coneID) {
    if (coneID == 4) {
        return true;
    }
    return false;
}

void BuildGraphNode::loopClosure(const std::vector<int> &cones) {
    bool out_of_frame = false;
    bool loop_closed = false;

    while (loop_closed == false) {
        for (int coneID : cones) {
            if (isLargeOrangeCone(coneID) && !landmarked) {
                landmarkedID = coneID;
                landmarked = true;
            }

            if (landmarked == true && out_of_frame == false) {
                auto seen_status = std::find(coneIDs.begin(), coneIDs.end(), landmarkedID);

                if (seen_status == coneIDs.end()) {
                    out_of_frame = true;
                }

                if (landmarked == true && out_of_frame == true) {
                    auto seen_status = std::find(coneIDs.begin(), coneIDs.end(), landmarkedID);

                    if (seen_status != coneIDs.end()) {
                        loop_closed = true;
                    }
                }
            }
        }
    }

    private:
        int landmarkedID;
        bool landmarked;
}


void BuildGraphNode::buildGraph() {}

} // namespace build_graph
} // namespace utfr_dv
