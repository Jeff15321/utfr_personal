/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: build_graph_node.hpp
* auth: Arthur Xu
* desc: build graph node header
*/
#pragma once

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// System Requirements
#include <chrono>
#include <fstream>
#include <functional>
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string>
#include <vector>

// Message Requirements
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <utfr_msgs/msg/cone_detections.hpp>
#include <utfr_msgs/msg/cone_map.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/pose_graph.hpp>
#include <utfr_msgs/msg/system_status.hpp>

// Import G2O 2D Slam types
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/types_slam2d.h>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

#include <g2o/core/sparse_optimizer.h>

// Misc Requirements:
using std::placeholders::_1; // for std::bind

namespace utfr_dv {
namespace build_graph {

//KD TREE IMPLEMENTATION

//point struct defining point
struct Point {
    double x;
    double y;

    Point(double x_val, double y_val) : x(x_val), y(y_val) {}

    // Define the operator!= for Point class
    bool operator!=(const Point& other) const {
        return (x != other.x) || (y != other.y);
    }
};
//KDNode struct storing axis info and defining node
struct KDNode {
    Point point;
    int index;
    KDNode* left;
    KDNode* right;
    int axis;  // 0 for x-axis, 1 for y-axis

    KDNode(const Point& p, int i, int a) : point(p), index(i), left(nullptr), right(nullptr), axis(a) {}
};

class KDTree {
private:
    KDNode* root;



    // Recursive function to build the KD tree
    KDNode* buildTree(std::vector<Point> points, int depth) {
    if (points.empty()) {
        return nullptr;
    }

    int axis = depth % 2; // Alternating between x and y coordinates

    // Sort points and choose median as pivot element
    if (axis == 0) {
        std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
            return a.x < b.x;
        });
    } else {
        std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
            return a.y < b.y;
        });
    }

    int median = points.size() / 2;
    KDNode* node = new KDNode(points[median], median, axis);

    // Recursively build left and right subtrees
    std::vector<Point> leftPoints(points.begin(), points.begin() + median);
    std::vector<Point> rightPoints(points.begin() + median + 1, points.end());

    node->left = buildTree(leftPoints, depth + 1);
    node->right = buildTree(rightPoints, depth + 1);

    return node;
}
    void printKDCoordinates(KDNode* node) {
    if (node == nullptr) {
        return;
    }

    // Print the coordinates of the current node
    std::cout << "Node coordinates: (" << node->point.x << ", " << node->point.y << ")" << std::endl;

    // Recursively print left and right subtrees
    printKDCoordinates(node->left);
    printKDCoordinates(node->right);
}


   // Function to find the nearest neighbor and return its x, y coordinates
    Point findNearestNeighbor(KDNode* node, const Point& target, int depth, KDTree* globalKDTreePtr) {
    if (node == nullptr) {
        return Point(0.0, 0.0);
    }

    int axis = node->axis;

    KDNode* nextBranch = nullptr;
    KDNode* otherBranch = nullptr;

    if ((axis == 0 && target.x < node->point.x) || (axis == 1 && target.y < node->point.y)) {
        nextBranch = node->left;
        otherBranch = node->right;
    } else {
        nextBranch = node->right;
        otherBranch = node->left;
    }

    // Recursively search the appropriate branch
    Point nearestInNext = findNearestNeighbor(nextBranch, target, depth + 1, globalKDTreePtr);

    // Check current node
    double distanceToCurrent = utfr_dv::util::euclidianDistance2D(target.x, node->point.x, target.y, node->point.y);
    double distanceToNearestInNext = utfr_dv::util::euclidianDistance2D(target.x, nearestInNext.x, target.y, nearestInNext.y);

    // Decide whether to update the nearest point
    if (distanceToCurrent < distanceToNearestInNext) {
        nearestInNext = node->point;
    }

    // Recursively search the other branch if needed
    if (std::abs((axis == 0 ? target.x : target.y) - (axis == 0 ? node->point.x : node->point.y)) < distanceToNearestInNext) {
        Point nearestInOther = findNearestNeighbor(otherBranch, target, depth + 1, globalKDTreePtr);
        double distanceToNearestInOther = utfr_dv::util::euclidianDistance2D(target.x, nearestInOther.x, target.y, nearestInOther.y);

        // Decide whether to update the nearest point
        if (distanceToNearestInOther < distanceToNearestInNext) {
            nearestInNext = nearestInOther;
        }
    }

    return nearestInNext;
}
    // Recursive function to insert a new node into KD tree
    KDNode* insertNode(KDNode* node, const Point& newPoint, int depth, int& currentIndex) {
        if (node == nullptr) {
            return new KDNode(newPoint, currentIndex++, depth % 2);
        }

        int axis = depth % 2;

        if ((axis == 0 && newPoint.x < node->point.x) || (axis == 1 && newPoint.y < node->point.y)) {
            node->left = insertNode(node->left, newPoint, depth + 1, currentIndex);
        } else {
            node->right = insertNode(node->right, newPoint, depth + 1, currentIndex);
        }

        return node;
    }
    // Recursive function to count the number of cones in the tree
    int countCones(KDNode* node) const {
        if (node == nullptr) {
            return 0;
        }

        // Count the cone in the current node and recursively count in left and right subtrees
        return 1 + countCones(node->left) + countCones(node->right);
    }
    // Recursive function to get the point for a given index
    Point getPointFromIndex(KDNode* node, int targetIndex) const {
        if (node == nullptr) {
            // Handle the case where the index is not found (you can throw an exception or return a default Point)
            throw std::out_of_range("Index not found in KD tree");
        }

        if (node->index == targetIndex) {
            return node->point;
        }

        // Decide which branch to search based on the axis
        if (targetIndex < node->index) {
            return getPointFromIndex(node->left, targetIndex);
        } else {
            return getPointFromIndex(node->right, targetIndex);
        }
    }

public:

    int getNumberOfCones() const {
        return countCones(root);
    }
    // Constructor to build the KD tree
     KDTree(std::vector<Point> points) : root(buildTree(std::move(points), 0)) {}

    // Wrapper function for KNN search (always searches for 1 nearest neighbor)
    Point KNN(const Point& target) {
    return findNearestNeighbor(root, target, 0, this);
    }


    //insert a new point into the KD tree
    void insert(const Point& newPoint) {
        int currentIndex = 0;  // Start with index 0 for the root node
        root = insertNode(root, newPoint, 0, currentIndex);
    }


    // Public method to get the point for a given index
    Point getPoint(int targetIndex) const {
        return getPointFromIndex(root, targetIndex);
    }
    void printKDCoordinates() {
        printKDCoordinates(root);
    }
};

class BuildGraphNode : public rclcpp::Node {
public:
  /*! Constructor, calls loadParams, initPublishers and initTimers.
   */
  BuildGraphNode();

  /*! Initialize and load params from config.yaml:
   */
  void initParams();

  /*! Initialize Subscribers:
   */
  void initSubscribers();

  /*! Initialize Publishers:
   */
  void initPublishers();

  /*! Initialize Timers:
   */
  void initTimers();

  /*! Initialize Heartbeat:
   */
  void initHeartbeat();

  /*! Cone detection callback function
   */
  void coneDetectionCB(const utfr_msgs::msg::ConeDetections msg);

  /*! State Estimation callback function
   */
  void stateEstimationCB(const utfr_msgs::msg::EgoState msg);

  /*! Implement a KNN algorithm to match cones to previous detections
   *  @param[in] cones utfr_msgs::msg::ConeDetecions&, cone detections
   *  @param[in] past_detections_ std::vector<std::pair<float,
   * utfr_msgs::msg::Cone>>&, current cone id mapping
   *  @param[out] past_detections_ std::vector<std::pair<float,
   * utfr_msgs::msg::Cone>>&, updated cone id mapping
   *  @param[out] detected_cone_ids std::vector<int>&, ids of cones detected
   */

  std::vector<int> KNN(const utfr_msgs::msg::ConeDetections &cones);

  /*! Implement functionalty to detect loop closures
   *  @param[in] cones std::vector<int>&, ids of detected cones
   *  @param[out] loop_closed boolean&, true if loop is closed
   */
  void loopClosure(const std::vector<int> &cones);

  /*! Create a pose node for G2O
    * @param[in] id int, id of the pose node
    * @param[in] x double, x position of the pose node
    * @param[in] y double, y position of the pose node
    * @param[in] theta double, rotation of the pose node (radians)
    * @param[out] vertex g2o::VertexSE2*, pose node pointer
   */
  g2o::VertexSE2* createPoseNode(int id, double x, double y, double theta);

  /*! Create a cone node for G2O
    * @param[in] id int, id of the cone node
    * @param[in] x double, x position of the cone node
    * @param[in] y double, y position of the cone node
    * @param[out] vertex g2o::VertexSE2, cone node pointer
   */
  g2o::VertexPointXY* createConeVertex(int id, double x, double y);

  /*! Add a pose to pose edge to G2O
    * @param[in] pose1 g2o::VertexSE2*, pointer to the first pose node
    * @param[in] pose2 g2o::VertexSE2*, pointer to the second pose node
    * @param[in] dx double, x distance between poses
    * @param[in] dy double, y distance between poses
    * @param[in] dtheta double, rotation between poses
    * @param[in] loop_closure bool, true if loop closure
    * @param[out] edge g2o::EdgeSE2PointXY*, pose to pose edge pointer
   */
  g2o::EdgeSE2* addPoseToPoseEdge(g2o::VertexSE2* pose1, g2o::VertexSE2* pose2,
                         double dx, double dy, double dtheta, bool loop_closure);

  /*! Add a pose to cone edge to G2O
    * @param[in] pose g2o::VertexSE2*, pointer to the pose node
    * @param[in] cone g2o::VertexPointXY*, pointer to the cone node
    * @param[in] dx double, x distance between pose and cone
    * @param[in] dy double, y distance between pose and cone
    * @param[out] edge g2o::EdgeSE2PointXY*, pose to cone edge pointer
   */
  g2o::EdgeSE2PointXY* addPoseToConeEdge(g2o::VertexSE2* pose, g2o::VertexPointXY* cone,
                               double dx, double dy);

  /*! Compose a graph for G2O to optimize.
   *  @param[in] states std::vector<utfr_msgs::msg::EgoState>&, past states
   *  @param[in] cones std::vector<std::pair<float, utfr_msgs::msg:Cone>>&, list
   * of cones, with their associated ID's
   *  @param[out] cone_map utfr_msgs::msg::ConeMap&, updated cone map
   */
  void buildGraph();

  /*! Graph SLAM function
   *   @param[in] pose_graph utfr_msgs::msg::PoseGraph Pose graph message
   *   @param[out] cone_map utfr_msgs::msg::ConeMap Cone map message
   */
  void graphSLAM();

  // Publisher
  rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<utfr_msgs::msg::PoseGraph>::SharedPtr pose_graph_publisher_;

  // Subscribers
  rclcpp::Subscription<utfr_msgs::msg::ConeDetections>::SharedPtr
      cone_detection_subscriber_;
  rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr
      state_estimation_subscriber_;

  // Global variables
  std::vector<std::pair<float, utfr_msgs::msg::Cone>>
      past_detections_;                      // Previous cone detections
  std::map<int, g2o::VertexPointXY*> cone_id_to_vertex_map_;
  utfr_msgs::msg::ConeMap current_cone_map_; // Current cone map estimate
  utfr_msgs::msg::EgoState current_state_;   // Current state estimate
  std::map<int, utfr_msgs::msg::Cone> id_to_cone_map_; // Maps cone detection to id
  std::map<int, utfr_msgs::msg::EgoState> id_to_ego_map_; // Maps state estimate to id
  std::map<int, g2o::VertexSE2*> id_to_pose_map_; // Maps state estimate to pose node
  std::map<int, std::tuple<double, double>> potential_cones_;
  int cones_potential_;
  int count_;
  bool loop_closed_;                         // True if loop is closed
  bool landmarked_;
  int landmarkedID_;
  int out_of_frame_;
  int cones_found_;
  int current_pose_id_;
  int first_detection_pose_id_;
  std::unique_ptr<KDTree> globalKDTreePtr;
  

  // Lists for poses, cones, and edges
  std::vector<g2o::VertexSE2*> pose_nodes_;
  std::vector<g2o::VertexPointXY*> cone_nodes_;
  std::vector<g2o::EdgeSE2*> pose_to_pose_edges_;
  std::vector<g2o::EdgeSE2PointXY*> pose_to_cone_edges_;

  // G2O Information matricies
  Eigen::Matrix3d P2PInformationMatrix_;
  Eigen::Matrix2d P2CInformationMatrix_;
  Eigen::Matrix3d LoopClosureInformationMatrix_;

  g2o::SparseOptimizer optimizer_;
  utfr_msgs::msg::ConeMap cone_map_;
};
} // namespace build_graph
} // namespace utfr_dv