/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: kd_tree_knn.hpp
* auth: Mark Pirner
* desc: build graph node knn header
*/
#pragma once // prevent multiple inclusion

// System Requirements
#include <chrono>
#include <fstream>
#include <functional>
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string>
#include <vector>
#include <limits>

// UTFR Common Requirements
#include <utfr_common/frames.hpp>
#include <utfr_common/math.hpp>
#include <utfr_common/topics.hpp>

namespace utfr_dv {

// KD TREE IMPLEMENTATION

namespace kd_tree_knn {

// Define Point Struct

struct Point {
  double x;
  double y;
  double id;

  Point(double x_val, double y_val, double id_val)
      : x(x_val), y(y_val), id(id_val) {}

    // Define the operator!= for Point class
    bool operator!=(const Point& other) const {
        return (x != other.x) || (y != other.y);
    }
    };

    // Create KDNode, Store Axis Info and Define Node
    struct KDNode {
      Point point;
      int index;
      KDNode * left;
      KDNode * right;
      int axis; // 0 for x-axis, 1 for y-axis

      KDNode(const Point & p, int i, int a): point(p), index(i), left(nullptr), right(nullptr), axis(a) {}
    };

    class KDTree {
      private: KDNode * root;

      // Build the KD tree (recursive)

      KDNode * buildTree(std::vector < Point > points, int depth) {
        if (points.empty()) {
          return nullptr;
        }

        int axis = depth % 2; // Alternating between x and y coordinates

        // Sort points, choose median as pivot element
        if (axis == 0) {
          std::sort(points.begin(), points.end(), [](const Point & a,
            const Point & b) {
            return a.x < b.x;
          });
        } else {
          std::sort(points.begin(), points.end(), [](const Point & a,
            const Point & b) {
            return a.y < b.y;
          });
        }

        int median = points.size() / 2;
        KDNode * node = new KDNode(points[median], median, axis);

        // Recursively build left and right subtrees
        std::vector < Point > leftPoints(points.begin(), points.begin() + median);
        std::vector < Point > rightPoints(points.begin() + median + 1, points.end());

        node -> left = buildTree(leftPoints, depth + 1);
        node -> right = buildTree(rightPoints, depth + 1);

        return node;
      }
      void printKDCoordinates(KDNode * node) {
        if (node == nullptr) {
          return;
        }

        // Print the coordinates of the current node
        std::cout << "Node coordinates: (" << node -> point.x << ", " << node -> point.y << ")" << std::endl;

        // Recursively print left and right subtree node coordinates
        printKDCoordinates(node -> left);
        printKDCoordinates(node -> right);
      }

      // Find the nearest neighbor and return its x, y coordinates
      Point findNearestNeighbor(KDNode * node,
        const Point & target, int depth, KDTree * globalKDTreePtr) {
        if (node == nullptr) {
          return Point(0.0, 0.0, 0.0);
        }

        int axis = node -> axis;

        KDNode * nextBranch = nullptr;
        KDNode * otherBranch = nullptr;

        if ((axis == 0 && target.x < node -> point.x) || (axis == 1 && target.y < node -> point.y)) {
          nextBranch = node -> left;
          otherBranch = node -> right;
        } else {
          nextBranch = node -> right;
          otherBranch = node -> left;
        }

        // Recursively search the appropriate branch
        Point nearestInNext = findNearestNeighbor(nextBranch, target, depth + 1, globalKDTreePtr);

        // Check current node
        double distanceToCurrent = utfr_dv::util::euclidianDistance2D(target.x, node -> point.x, target.y, node -> point.y);
        double distanceToNearestInNext = utfr_dv::util::euclidianDistance2D(target.x, nearestInNext.x, target.y, nearestInNext.y);

        // Decide whether to update the nearest point
        if (distanceToCurrent < distanceToNearestInNext) {
          nearestInNext = node -> point;
        }

        // Recursively search the other branch if needed
        if (std::abs((axis == 0 ? target.x : target.y) - (axis == 0 ? node -> point.x : node -> point.y)) < distanceToNearestInNext) {
          Point nearestInOther = findNearestNeighbor(otherBranch, target, depth + 1, globalKDTreePtr);
          double distanceToNearestInOther = utfr_dv::util::euclidianDistance2D(target.x, nearestInOther.x, target.y, nearestInOther.y);

          // Decide whether to update the nearest point
          if (distanceToNearestInOther < distanceToNearestInNext) {
            nearestInNext = nearestInOther;
          }
        }

        return nearestInNext;
      }
      
      // Insert a new node into KD tree (recursive)
      KDNode * insertNode(KDNode * node,
        const Point & newPoint, int depth, int & currentIndex) {
        if (node == nullptr) {
          return new KDNode(newPoint, currentIndex++, depth % 2);
        }

        int axis = depth % 2;

        if ((axis == 0 && newPoint.x < node -> point.x) || (axis == 1 && newPoint.y < node -> point.y)) {
          node -> left = insertNode(node -> left, newPoint, depth + 1, currentIndex);
        } else {
          node -> right = insertNode(node -> right, newPoint, depth + 1, currentIndex);
        }

        return node;
      }

      // Count the number of cones in the tree (recursive)
      int countCones(KDNode * node) const {
        if (node == nullptr) {
          return 0;
        }

        // Count the cone in the current node and recursively count in left and right subtrees
        return 1 + countCones(node -> left) + countCones(node -> right);
      }

      // Get point for given index (recursive)
      Point getPointFromIndex(KDNode * node, int targetIndex) const {
        if (node == nullptr) {
          // Handle the case where the index is not found (you can throw an exception or return a default Point)
          throw std::out_of_range("Index not found in KD tree");
        }

        if (node -> index == targetIndex) {
          return node -> point;
        }

        // Decide which branch to search based on the axis
        if (targetIndex < node -> index) {
          return getPointFromIndex(node -> left, targetIndex);
        } else {
          return getPointFromIndex(node -> right, targetIndex);
        }
      }

          // Utility function to calculate distance between two points
      double distance(const Point& point1, const Point& point2) {
          return sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
      }

      // Recursive function to find the nearest neighbor
      void nearestUtil(KDNode* root, const Point& point, unsigned depth, KDNode*& best, double& bestDist) {
          if (root == nullptr)
              // return Point(0.0, 0.0, 0.0);
              // Rewrite the above line to work
              return;

          double d = distance(root->point, point);
          unsigned cd = depth % 2;

          // Update best neighbor if closer point is found
          if (d < bestDist) {
              bestDist = d;
              best = root;
          }

          // Recurse down the tree
          // Node* goodSide = point[cd] < root->point[cd] ? root->left : root->right;
          // Node* badSide = point[cd] < root->point[cd] ? root->right : root->left;
          KDNode* goodSide = nullptr;
          KDNode* badSide = nullptr;
          // Point has the x and y values, rewrite the above line to use the x and y values
          if (cd == 0) {
            if (point.x < root->point.x) {
              goodSide = root->left;
              badSide = root->right;
            } else {
              goodSide = root->right;
              badSide = root->left;
            }
          } else {
            if (point.y < root->point.y) {
              goodSide = root->left;
              badSide = root->right;
            } else {
              goodSide = root->right;
              badSide = root->left;
            }
          }

          nearestUtil(goodSide, point, depth + 1, best, bestDist);

          // Check if we need to explore the other side
          // if ((point[cd] - root->point[cd]) * (point[cd] - root->point[cd]) < bestDist) {
          //     nearestUtil(badSide, point, depth + 1, best, bestDist);
          // }
          if (cd == 0) {
            if ((point.x - root->point.x) * (point.x - root->point.x) < bestDist) {
              nearestUtil(badSide, point, depth + 1, best, bestDist);
            }
          } else {
            if ((point.y - root->point.y) * (point.y - root->point.y) < bestDist) {
              nearestUtil(badSide, point, depth + 1, best, bestDist);
            }
          }
      }


      public:

        int getNumberOfCones() const {
          return countCones(root);
        }

      // Kd Tree Constructor
      KDTree(std::vector < Point > points): root(buildTree(std::move(points), 0)) {}

      // Wrapper function for KNN search (always searches for 1 nearest neighbor)
      Point KNN(const Point & target) {
        KDNode* best = nullptr;
        double bestDist = std::numeric_limits<double>::max();
        nearestUtil(root, target, 0, best, bestDist);
        if (best == nullptr) {
          return Point(0.0, 0.0, 0.0);
        }
        return best->point;
      }

      // Insert a new point into the KD tree
      void insert(const Point & newPoint) {
        int currentIndex = 0; // Start with index 0 for the root node
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
  }
};
