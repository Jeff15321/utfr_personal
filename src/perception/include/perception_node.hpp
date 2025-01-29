// perception_node.hpp
#ifndef PERCEPTION_NODE_HPP_
#define PERCEPTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <chrono>
#include <memory>

namespace perception {

class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode();
    virtual ~PerceptionNode() = default;

private:
    // Parameters
    double baseline_;
    std::string left_camera_topic_;
    std::string right_camera_topic_;
    std::string cone_detections_topic_;
    double camera_capture_rate_;
    bool debug_;
    cv::Mat distortion_;
    cv::Mat intrinsics_;
    cv::Mat rotation_;
    cv::Mat translation_;
    bool save_pic_;
    double confidence_;
    std::vector<double> cone_heights_;
    bool is_cuda_cv_;
    bool is_cuda_deep_;
    
    // Variables
    bool new_cam_received_;
    bool first_img_arrived_;
    cv::Mat img_;
    cv::Mat img_raw_;
    cv::Size img_size_;
    cv::Mat mapx_, mapy_;
    cv::VideoCapture cam_capture_;
    cv::cuda::GpuMat mapx_gpu_, mapy_gpu_;
    cv::cuda::GpuMat img_gpu_;
    cv::Mat previous_img_;
    double last_cam_capture_time_;
    
    // ROS timers
    rclcpp::TimerBase::SharedPtr camera_capture_timer_;
    
    // Methods
    void loadParams();
    void initVariables();
    void cameraCaptureTimerCallback();
    bool findCamera();
    
    // Helper methods
    bool frameChanged(const cv::Mat& frame, const cv::Mat& prev_frame);
    bool hasVisualArtifacts(const cv::Mat& frame);
};

} // namespace perception

#endif // PERCEPTION_NODE_HPP_