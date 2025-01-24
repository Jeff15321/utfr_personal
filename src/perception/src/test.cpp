#include <rclcpp/rclcpp.hpp>
#include <memory>

class HelloWorldNode : public rclcpp::Node {
public:
    HelloWorldNode() : Node("test_node"), count_(0) {
        // Create timer for periodic execution (1 second interval)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HelloWorldNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Hello World Counter initialized");
    }

private:
    void timer_callback() {
        count_++;
        RCLCPP_INFO(this->get_logger(), "Hello World %d!", count_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloWorldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

