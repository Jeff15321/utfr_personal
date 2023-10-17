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

void BuildGraphNode::initParams() {
  bool loop_closed_ = false;
  bool landmarked_ = false;
  int landmarkedID_ = -1;
  bool out_of_frame_ = false;
  int cones_found_ = 0;
}

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

std::vector<int> KNN(const utfr_msgs::msg::ConeDetections &cones, std::vector<std::pair<float, utfr_msgs::msg::Cone>> &past_detections_, int cones_found_,const utfr_msgs::msg::EgoState &ego_state_){
    std::vector<int> cones_id_list_;  
    std::vector<utfr_msgs::msg::Cone> all_cones;
    //adding all cones to one vector
    all_cones.insert(all_cones.end(), cones.left_cones.begin(), cones.left_cones.end());
    all_cones.insert(all_cones.end(), cones.right_cones.begin(), cones.right_cones.end());
    all_cones.insert(all_cones.end(), cones.large_orange_cones.begin(), cones.large_orange_cones.end());
    all_cones.insert(all_cones.end(), cones.small_orange_cones.begin(), cones.small_orange_cones.end());
    //iterating through all detected cones
    for (const auto& pastCone : all_cones){
      utfr_msgs::msg::Cone newCone = pastCone;
      //updating detected position to global frame
      double ego_x = ego_state_.pose.x;
      double ego_y = ego_state_.pose.y;
      newCone.pose.x += ego_x;
      newCone.pose.y += ego_y;
      bool adding_to_past = true;      
      //iterating through old cones
      for (size_t i=0; i <past_detections_.size(); ++i){
        const utfr_msgs::msg::Cone&pastDetectionsCone=past_detections_[i].second;    
        //finding displacement between detected cone and past cone    
        double displacement = utfr_dv::util::euclidianDistance2D(newCone.pos.x, pastDetectionsCone.pos.x, newCone.pos.y, pastDetectionsCone.pos.y);    
        //not adding if already detected (within error)
        if (displacement <= 0.3){
          adding_to_past=false;
          break;
        }
      }
      if (adding_to_past){
        // adding to id_list_ and past_detections_ if past error for all previously detected cones
        cones_id_list_.push_back(cones_found_);
        past_detections_.emplace_back(cones_found_,newCone);
        cones_found_+=1;
      }
    }
    return cones_id_list_;  }

void BuildGraphNode::loopClosure(const std::vector<int> &cones) {

  // Loop hasn't been completed yet
  while (loop_closed_ == false) {
    // Go through cone list
    for (int coneID : cones) {
      // No landmark cone yet
      if (utfr_dv::util::isLargeOrangeCone(coneID) && !landmarked_) {
        // Set first large orange cone listed to be landmark cone
        landmarkedID_ = coneID;
        landmarked_ = true;
      }
      // Cone has been landmarked and still in frame for the first time
      if (landmarked_ == true && out_of_frame_ == false) {
        auto seen_status = std::find(cones.begin(), cones.end(), landmarkedID_);
        // If cone not in frame anymore
        if (seen_status == cones.end()) {
          // Set out of frame to be true
          out_of_frame_ = true;
        }
      }
      // Cone has been landmarked and is no longer in frame
      if (landmarked_ == true && out_of_frame_ == true) {
        // Check if cone appears in frame again
        auto seen_status = std::find(cones.begin(), cones.end(), landmarkedID_);
        // If cone in frame again
        if (seen_status != cones.end()) {
          // Car has returned back to landmark cone position, made full loop
          loop_closed_ = true;
        }
      }
    }
  }
}

void BuildGraphNode::buildGraph() {}

} // namespace build_graph
} // namespace utfr_dv
