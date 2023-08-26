/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: preprocess_graph_node.cpp
* auth: Arthur Xu
* desc: preproccess graph node class
*/

#include <preprocess_graph_node.hpp>

namespace utfr_dv {
namespace preprocess_graph {

PreprocessGraphNode::PreprocessGraphNode() : Node("preprocess_graph_node") {
  this->initParams();
  this->initSubscribers();
  this->initPublishers();
  this->initTimers();
  this->initHeartbeat();
}

void PreprocessGraphNode::initParams() {}

void PreprocessGraphNode::initSubscribers() {}

void PreprocessGraphNode::initPublishers() {}

void PreprocessGraphNode::initTimers() {}

void PreprocessGraphNode::initHeartbeat() {}

} // namespace preprocess_graph
} // namespace utfr_dv

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<utfr_dv::preprocess_graph::PreprocessGraphNode>());
  rclcpp::shutdown();
  return 0;
}
