/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: knn_node.cpp
* auth: Arthur Xu
* desc: knn node class
*/

#include <knn_node.hpp>

namespace utfr_dv {
namespace knn {

KnnNode::KnnNode() : Node("controller_node") {
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

void KnnNode::initParams() {}

void KnnNode::initSubscribers() {}

void KnnNode::initPublishers() {}

void KnnNode::initTimers() {}

void KnnNode::initHeartbeat() {}

} // namespace knn
} // namespace utfr_dv

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utfr_dv::knn::KnnNode>());
  rclcpp::shutdown();
  return 0;
}
