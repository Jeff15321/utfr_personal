// perception_node.hpp
#ifndef PERCEPTION_NODE_HPP_
#define PERCEPTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <chrono>
#include <memory>
#include <vector>

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
    
    // Add these member variables
    cv::Mat hsv_;
    cv::Mat mask_;
    std::vector<cv::Mat> hsv_channels_;
    
    // Add these parameters
    int hue_low_;
    int hue_high_;
    int sat_low_;
    int sat_high_;
    int val_low_;
    int val_high_;
    
    // Add these member variables
    struct TrackedCone {
        cv::Point2f position;
        int tracking_id;
        std::string color;  // "yellow" or "blue"
        int frames_tracked;
    };
    
    std::vector<TrackedCone> tracked_cones_;
    int next_tracking_id_ = 0;
    double max_tracking_distance_ = 50.0;  // pixels
    int min_frames_tracked_ = 3;
    
    // Methods
    void loadParams();
    void initVariables();
    void cameraCaptureTimerCallback();
    bool findCamera();
    
    // Add these methods
    void processImage();
    std::vector<cv::Point2f> detectCones(const cv::Mat& frame);
    bool isValidContour(const std::vector<cv::Point>& contour);
    cv::Point2f getContourCentroid(const std::vector<cv::Point>& contour);
    
    // Helper methods
    bool frameChanged(const cv::Mat& frame, const cv::Mat& prev_frame);
    bool hasVisualArtifacts(const cv::Mat& frame);
    void loadColorParams();
    void applyColorMask(const cv::Mat& frame);
    
    // Add these methods
    void trackAndClassifyCones(const std::vector<cv::Point2f>& detected_centers);
    std::string classifyConeColor(const cv::Point2f& position);
    double getAverageHue(const cv::Point2f& position, int region_size = 10);
};

} // namespace perception

#endif // PERCEPTION_NODE_HPP_