#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ultralytics/yolo.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utfr_msgs/msg/cone_detections.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/ego_state.hpp>

class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode();

private:
    // Parameter loading
    void loadParams();

    // Initialization methods
    void initVariables();
    void initSubscribers();
    void initPublishers();
    void initTimers();
    
    // Callback methods
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void egoStateCallback(const utfr_msgs::msg::EgoState::SharedPtr msg);
    void cameraTimerCallback();
    void preprocessCallback();
    void heartbeatCallback();

    // Core processing methods
    std::tuple<std::vector<cv::Rect>, std::vector<std::string>, std::vector<float>> 
    processImage(const cv::Mat& image);

    void fuseDetections(const sensor_msgs::msg::PointCloud2::SharedPtr lidarMsg);
    
    // Utility methods
    cv::VideoCapture findCamera();
    void transformPointCloud(/* params */);
    void projectPointsToImage(/* params */);
    
    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidarSubscription_;
    rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr egoStateSubscription_;
    
    rclcpp::Publisher<utfr_msgs::msg::ConeDetections>::SharedPtr coneDetectionsPublisher_;
    rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeatPublisher_;
    
    rclcpp::TimerBase::SharedPtr cameraTimer_;
    rclcpp::TimerBase::SharedPtr preprocessTimer_;
    rclcpp::TimerBase::SharedPtr heartbeatTimer_;

    // Model and processing variables
    cv::VideoCapture camera_;
    YOLO model_;
    cv::Mat currentImage_;
    
    // Configuration parameters
    double baseline_;
    std::string leftCameraTopic_;
    double confidence_;
    bool debugMode_;

    // TF2 transform handling
    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
};

PerceptionNode::PerceptionNode() : Node("perception_node") {
    // Initialize TF2 buffer and listener
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    // Load parameters
    loadParams();
    
    // Initialize components
    initVariables();
    initSubscribers();
    initPublishers();
    initTimers();
}

void PerceptionNode::loadParams() {
    // Declare and get ROS2 parameters
    this->declare_parameter("baseline", 10.0);
    this->declare_parameter("camera_topic", "/camera/images");
    this->declare_parameter("confidence", 0.7);
    
    baseline_ = this->get_parameter("baseline").as_double();
    leftCameraTopic_ = this->get_parameter("camera_topic").as_string();
    confidence_ = this->get_parameter("confidence").as_double();
}

void PerceptionNode::initSubscribers() {
    // LiDAR point cloud subscriber
    lidarSubscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lidar/points", 
        10, 
        std::bind(&PerceptionNode::lidarCallback, this, std::placeholders::_1)
    );

    // Ego state subscriber
    egoStateSubscription_ = this->create_subscription<utfr_msgs::msg::EgoState>(
        "/ego_state", 
        10, 
        std::bind(&PerceptionNode::egoStateCallback, this, std::placeholders::_1)
    );
}

void PerceptionNode::initPublishers() {
    // Cone detections publisher
    coneDetectionsPublisher_ = this->create_publisher<utfr_msgs::msg::ConeDetections>(
        "/perception/cone_detections", 
        10
    );

    // Heartbeat publisher
    heartbeatPublisher_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
        "/perception/heartbeat", 
        10
    );
}

void PerceptionNode::initTimers() {
    // Camera capture timer
    cameraTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),  // ~30 FPS
        std::bind(&PerceptionNode::cameraTimerCallback, this)
    );

    // Preprocess timer
    preprocessTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PerceptionNode::preprocessCallback, this)
    );

    // Heartbeat timer
    heartbeatTimer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&PerceptionNode::heartbeatCallback, this)
    );
}