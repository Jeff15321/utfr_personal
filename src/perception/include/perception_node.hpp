#ifndef PERCEPTION_NODE_HPP_
#define PERCEPTION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <utfr_msgs/msg/cone_detections.hpp>
#include <utfr_msgs/msg/heartbeat.hpp>
#include <utfr_msgs/msg/perception_debug.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <utfr_msgs/msg/ego_state.hpp>
#include <chrono>
#include <memory>
#include <vector>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tuple>

namespace perception {

class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode();
    virtual ~PerceptionNode() = default; 
    // Jeff: Purpose of defining a virtual destructor is to ensure that the destructor of the derived class is called when an object of the derived class is destroyed.

private:
    // Parameters
    std::string camera_topic_; 
    double camera_capture_rate_;
    bool debug_;
    cv::Mat distortion_;
    cv::Mat intrinsics_;
    cv::Mat rotation_;      // 3x3 rotation matrix
    cv::Mat rectify_;       // 3x3 rectification matrix
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
    
    // Additional parameters needed
    std::string heartbeat_topic_;
    std::string debug_topic_;
    double heartbeat_rate_;
    double min_cone_area_;
    double max_cone_area_;
    int min_cone_width_;
    int max_cone_width_;
    int min_cone_height_;
    int max_cone_height_;
    
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr undistorted_pub_;
    rclcpp::Publisher<utfr_msgs::msg::PerceptionDebug>::SharedPtr lidar_projection_pub_;
    rclcpp::Publisher<utfr_msgs::msg::PerceptionDebug>::SharedPtr lidar_projection_matched_pub_;
    rclcpp::Publisher<utfr_msgs::msg::PerceptionDebug>::SharedPtr debug_pub_;
    rclcpp::Publisher<utfr_msgs::msg::ConeDetections>::SharedPtr cone_detections_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr processed_lidar_subscriber_;
    rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr ego_state_subscriber_;
    
    // Callback methods
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void egoStateCallback(const utfr_msgs::msg::EgoState::SharedPtr msg);
    
    // Add these variables
    std::string processed_lidar_topic_;
    utfr_msgs::msg::EgoState ego_state_;
    
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
    
    // Additional methods needed
    void initPublishers();
    void initTimers();
    void publishHeartbeat();
    void publishDebugInfo();
    
    // Cone detection methods
    bool isValidCone(const std::vector<cv::Point>& contour);
    void filterContours(std::vector<std::vector<cv::Point>>& contours);
    cv::Rect getBoundingBox(const std::vector<cv::Point>& contour);
    double getAspectRatio(const cv::Rect& bbox);
    
    // Performance tracking
    struct PerformanceMetrics {
        double fps;
        double processing_time;
        int frame_count;
        std::chrono::time_point<std::chrono::steady_clock> last_frame_time;
    };
    PerformanceMetrics metrics_;
    void updateMetrics();
    
    // Debug visualization
    void drawDebugInfo(cv::Mat& debug_frame);
    void showDebugWindows();
    
    // Testing variables
    int lidar_msg_count_ = 0;
    int ego_state_msg_count_ = 0;
    double last_lidar_time_ = 0.0;
    
    // Testing methods
    void printSubscriberStats();

    // Heartbeat status constants
    static const int STATUS_UNINITIALIZED = 0;
    static const int STATUS_ACTIVE = 1;
    static const int STATUS_FATAL = 2;
    
    // Add these member variables
    utfr_msgs::msg::Heartbeat heartbeat_msg_;
    static const int MIN_HEIGHT = 100;  // For camera status checks
    static const int MIN_WIDTH = 100;

    // Transform handling
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string lidar_frame_;
    std::string camera_frame_;

    // Core perception pipeline methods
    std::vector<std::tuple<cv::Point2f, std::string, float>> 
    process(const cv::Mat& img);
    
    // Detection publishing methods
    void publish_cone_dets(
        const std::vector<std::tuple<cv::Point3f, std::string, float>>& cone_detections);
    void publish_cone_dets_lidar(
        const sensor_msgs::msg::PointCloud2::SharedPtr& lidar_msg);
    
    // Visualization methods
    void publish_2d_projected_det(
        const std::vector<cv::Point2f>& projected_pts,
        const rclcpp::Time& stamp);
    void publish_2d_projected_det_matched(
        const std::vector<cv::Point2f>& projected_pts,
        const rclcpp::Time& stamp);
    void displayBoundingBox(
        const std::vector<cv::Rect>& results_left,
        const std::vector<std::string>& classes_left,
        const std::vector<float>& scores_left,
        const rclcpp::Time& img_stamp);

    // Add these method declarations in the private section
    // Initialization methods
    void initSubscribers();
    void initHeartbeat();
    int cameraStatus();
    void deepAndMatching(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

    // Add heartbeat publisher
    rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_pub_;
};

} // namespace perception

#endif // PERCEPTION_NODE_HPP_