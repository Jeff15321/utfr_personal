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
  loop_closed_ = false;
  landmarked_ = false;
  landmarkedID_ = -1;
  out_of_frame_ = false;
  cones_found_ = 0;
  current_pose_id_ = 0;
  first_detection_pose_id_ = 0;
  count_ = 0;
  cones_potential_= 0;
  globalKDTreePtr = nullptr;

  // Will have to tune these later depending on the accuracy of our sensors
  P2PInformationMatrix_ = Eigen::Matrix3d::Identity();
  P2CInformationMatrix_ = Eigen::Matrix2d::Identity();
  LoopClosureInformationMatrix_ = Eigen::Matrix3d::Identity();
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

void BuildGraphNode::initTimers() {
}
void BuildGraphNode::initHeartbeat() {
  heartbeat_publisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
      topics::kMappingBuildHeartbeat, 10);
}

void BuildGraphNode::coneDetectionCB(const utfr_msgs::msg::ConeDetections msg) {
  std::vector<int> cones = KNN(msg);
  loopClosure(cones);
}

void BuildGraphNode::stateEstimationCB(const utfr_msgs::msg::EgoState msg) {
  current_state_ = msg;

  current_pose_id_ += 1;
  id_to_ego_map_[current_pose_id_] = current_state_;
  
  g2o::VertexSE2* poseVertex = createPoseNode(current_pose_id_, msg.pose.pose.position.x, msg.pose.pose.position.y, utfr_dv::util::quaternionToYaw(msg.pose.pose.orientation));
  pose_nodes_.push_back(poseVertex);
  id_to_pose_map_[current_pose_id_] = poseVertex;

  utfr_msgs::msg::EgoState prevPoseVertex = id_to_ego_map_[current_pose_id_ - 1];

  double dx = current_state_.pose.pose.position.x - prevPoseVertex.pose.pose.position.x;
  double dy = current_state_.pose.pose.position.y - prevPoseVertex.pose.pose.position.y;
  double dtheta = utfr_dv::util::quaternionToYaw(current_state_.pose.pose.orientation) - utfr_dv::util::quaternionToYaw(prevPoseVertex.pose.pose.orientation);

  g2o::EdgeSE2* edge = addPoseToPoseEdge(id_to_pose_map_[current_pose_id_ - 1], poseVertex, dx, dy, dtheta, false);
}

KDTree generateKDTree(std::vector<std::tuple<double, double>> points_tuple) {
    // Convert tuples to points
    std::vector<Point> points;
    for (const auto& tuple : points_tuple) {
        points.emplace_back(std::get<0>(tuple), std::get<1>(tuple));
    }

    // Create a KD tree
    KDTree globalKDTree(points);

    // Now you can use the KD tree for various operations
    return globalKDTree;
}


std::vector<int> BuildGraphNode::KNN(const utfr_msgs::msg::ConeDetections &cones){
    std::vector<int> cones_id_list_;  
    std::vector<utfr_msgs::msg::Cone> all_cones;
    std::vector<std::tuple<double, double>> current_round_cones_;

    //adding all cones to one vector
    all_cones.insert(all_cones.end(), cones.left_cones.begin(), cones.left_cones.end());
    all_cones.insert(all_cones.end(), cones.right_cones.begin(), cones.right_cones.end());
    all_cones.insert(all_cones.end(), cones.large_orange_cones.begin(), cones.large_orange_cones.end());
    all_cones.insert(all_cones.end(), cones.small_orange_cones.begin(), cones.small_orange_cones.end());

    // Check if the KD tree is not created, and create it
    if (globalKDTreePtr == nullptr) {
        
        // Use first detection as root of tree and generate
        double position_x_ = all_cones[0].pos.x + current_state_.pose.pose.position.x;
        double position_y_ = all_cones[0].pos.y + current_state_.pose.pose.position.y;

        // Update vars
        past_detections_.emplace_back(cones_found_,all_cones[0]);
        cones_found_ += 1;
        all_cones.erase(all_cones.begin() + 0);
        globalKDTreePtr = std::make_unique<KDTree>(generateKDTree({std::make_tuple(position_x_, position_y_)}));
      }
    

    // Iterating through all detected cones
    for (const auto& newCone : all_cones) {
        // Updating detected position to global frame
        double ego_x = current_state_.pose.pose.position.x;
        double ego_y = current_state_.pose.pose.position.y;
        double position_x_ = newCone.pos.x + ego_x;
        double position_y_ = newCone.pos.y + ego_y;

        // Use KNN search to find the nearest cone
        std::vector<int> knnResult = globalKDTreePtr->KNN({position_x_,position_y_}, 1);

        // Check the result of the nearest neighbour search and calculate displacement
        if (!knnResult.empty()) {
            const Point& nearestCone = globalKDTreePtr->getPoint(knnResult.front());
            double displacement = utfr_dv::util::euclidianDistance2D(position_x_, nearestCone.x, position_y_, nearestCone.y);

            // Do not add if its within 0.3 of an already seen cone
            if (displacement <= 0.3) {
                continue;
            }

            // Do not add if its within 0.3 of another cone deteced within the current call of the function (rejecting duplicate detections)
            double comparative_displacement = 0.0;
            bool is_duplicate_ = false;
            for (const auto& duplicates_potential : current_round_cones_){
              comparative_displacement = utfr_dv::util::euclidianDistance2D(position_x_, std::get<0>(duplicates_potential), position_y_, std::get<1>(duplicates_potential));
              if (comparative_displacement <= 0.3){
                is_duplicate_ = true;
                break;
              }
            }
            if (is_duplicate_){
              continue;
            }
        }
        // Add to duplicate checker
        current_round_cones_.emplace_back(std::make_tuple(position_x_, position_y_));

        // Add to potential_cones (used for checking if cone can be added to past_detections)
        potential_cones_.insert(std::make_pair(cones_potential_, std::make_tuple(position_x_, position_y_)));

        // Initialize keys
        std::vector<int> keys{};

        // Check if same cone detected in three different time instances
        for (const auto& pair : potential_cones_) {
            int key_ = pair.first;
            const std::tuple<double,double> potentialPoint = pair.second;
            double temp_displacement_ = utfr_dv::util::euclidianDistance2D(position_x_, std::get<0>(potentialPoint), position_y_, std::get<1>(potentialPoint));

            // Check if cone within 0.3 of any other new detected cone
            if (temp_displacement_ <= 0.3) {
                    count_ += 1;
                    keys.push_back(key_);

                    // Check if three of same detected
                    if (count_ == 3) {

                        // FIX: Delete all of the same cone from potential_cones_ if three detected
                        //for (auto& key_number_ : keys) {
                            //potential_cones_.erase(key_number_);
                            //potential_cones_.erase(cones_potential_);
                        //}

                        // Update cone_id_list_ and past_detections_ and KD tree
                        cones_id_list_.push_back(cones_found_);
                        Point newPoint(position_x_,position_y_);
                        past_detections_.emplace_back(cones_found_,newCone);
                        globalKDTreePtr->insert(newPoint);
                        cones_found_ += 1;
                        }
                    }
            }

        count_ = 0;
        cones_potential_ += 1;
      }
      return cones_id_list_;
  }

void BuildGraphNode::loopClosure(const std::vector<int> &cones) {
  // Loop hasn't been completed yet
 
  if (!loop_closed_) {
    // Go through cone list
    for (int coneID : cones) {
      // No landmark cone yet
      // We gotta fix this later when Mark finishes his thing.
      // IDK why I chose past_detections_ to be a vector of pairs
      // but it should probably be a map :(
      for (auto cone : past_detections_) {
        if (cone.first == coneID && utfr_dv::util::isLargeOrangeCone(cone.second.type) && !landmarked_) {
          // Set first large orange cone listed to be landmark cone
          landmarkedID_ = coneID;
          landmarked_ = true;
          first_detection_pose_id_ = current_pose_id_;
        }
      }

      // if (utfr_dv::util::isLargeOrangeCone(coneID) && !landmarked_) {
      //   // Set first large orange cone listed to be landmark cone
      //   landmarkedID_ = coneID;
      //   landmarked_ = true;
      // }
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
          double dx = id_to_ego_map_[first_detection_pose_id_].pose.pose.position.x - 
          	      id_to_ego_map_[current_pose_id_].pose.pose.position.x;
          double dy = id_to_ego_map_[first_detection_pose_id_].pose.pose.position.y-
          	      id_to_ego_map_[current_pose_id_].pose.pose.position.y;
          double dtheta = current_state_.pose.pose.orientation.z;
          loop_closed_ = true;
          // Create node objects for each pose
          // Get the state estimate at a pose using id_to_state_map_[pose_id_]

          g2o::VertexSE2* first_pose_node = id_to_pose_map_[first_detection_pose_id_];
          g2o::VertexSE2* second_pose_node = id_to_pose_map_[current_pose_id_];

          // add an edge using the pose ids at initial detection and loop closure detection
          g2o::EdgeSE2* edge = addPoseToPoseEdge(first_pose_node, second_pose_node, dx, dy, dtheta, loop_closed_);
          pose_to_pose_edges_.push_back(edge);

        }
      }
    }
  }
}

g2o::VertexSE2* BuildGraphNode::createPoseNode(int id, double x, double y,
                                              double theta) {
  g2o::VertexSE2* poseVertex = new g2o::VertexSE2();
  poseVertex->setId(id);

  g2o::SE2 poseEstimate;
  poseEstimate.fromVector(Eigen::Vector3d(x, y, theta));
  poseVertex->setEstimate(poseEstimate);

  return poseVertex;
}

g2o::VertexPointXY* BuildGraphNode::createConeVertex(int id, double x, double y) {

  g2o::VertexPointXY* coneVertex = new g2o::VertexPointXY();
  coneVertex->setId(id);

  Eigen::Vector2d position(x, y);
  coneVertex->setEstimate(position);
  return coneVertex;
}

g2o::EdgeSE2* BuildGraphNode::addPoseToPoseEdge(g2o::VertexSE2* pose1, g2o::VertexSE2* pose2,
                                               double dx, double dy,
                                               double dtheta, bool loop_closure) {
  g2o::EdgeSE2* edge = new g2o::EdgeSE2();

  g2o::SE2 measurement(dx, dy, dtheta);
  edge->setMeasurement(measurement);
  if (loop_closure) {
    edge->setInformation(LoopClosureInformationMatrix_);
  } else {
    edge->setInformation(P2PInformationMatrix_);
  }

  edge->setVertex(0, pose1);
  edge->setVertex(1, pose2);
  
  return edge;
}

g2o::EdgeSE2PointXY* BuildGraphNode::addPoseToConeEdge(g2o::VertexSE2* pose,
                                                      g2o::VertexPointXY* cone,
                                                      double dx, double dy) {
  g2o::EdgeSE2PointXY* edge = new g2o::EdgeSE2PointXY();

  Eigen::Vector2d measurement(dx, dy);
  edge->setMeasurement(measurement);
  edge->setInformation(P2CInformationMatrix_);

  edge->setVertex(0, pose);
  edge->setVertex(1, cone);

  return edge;
}

void BuildGraphNode::buildGraph() {}

} // namespace build_graph
} // namespace utfr_dv