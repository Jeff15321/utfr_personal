#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "utfr_msgs/msg/cone_detections.hpp"
#include "utfr_msgs/msg/heartbeat.hpp"
#include "utfr_msgs/msg/cone.hpp"
#include "utfr_msgs/msg/bounding_box.hpp"
#include "utfr_msgs/msg/perception_debug.hpp"
#include "utfr_msgs/msg/ego_state.hpp"

using namespace std::chrono_literals;

class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode() : Node("perception_node") {
        loadParams();
        initVariables();
        initHeartbeat();
        initSubscribers();
        initPublishers();
        initServices();
        initConeTemplate();
        initTimers();
    }

private:
    // Parameters
    double baseline_;
    std::string left_camera_topic_;
    std::string right_camera_topic_;
    std::string cone_detections_topic_;
    std::string heartbeat_topic_;
    std::string processed_lidar_topic_;
    double camera_capture_rate_;
    double heartbeat_rate_;
    bool debug_;
    cv::Mat distortion_;
    cv::Mat intrinsics_;
    cv::Mat rotation_;
    cv::Mat translation_;
    std::vector<double> cone_heights_;
    bool lidar_only_detection_;
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
    cv::cuda::GpuMat mapx_gpu_, mapy_gpu_, img_gpu_;
    cv::Mat previous_img_;
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Publishers
    rclcpp::Publisher<utfr_msgs::msg::ConeDetections>::SharedPtr cone_detections_publisher_;
    rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr heartbeat_publisher_;
    rclcpp::Publisher<utfr_msgs::msg::EgoState>::SharedPtr ego_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr undistorted_publisher_;
    rclcpp::Publisher<utfr_msgs::msg::PerceptionDebug>::SharedPtr perception_debug_publisher_;
    rclcpp::Publisher<utfr_msgs::msg::PerceptionDebug>::SharedPtr lidar_projection_publisher_;
    rclcpp::Publisher<utfr_msgs::msg::PerceptionDebug>::SharedPtr lidar_projection_publisher_matched_;
    rclcpp::Publisher<utfr_msgs::msg::Heartbeat>::SharedPtr cam_processed_publisher_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr processed_lidar_subscriber_;
    rclcpp::Subscription<utfr_msgs::msg::EgoState>::SharedPtr ego_state_subscriber_;

    // Timers
    rclcpp::TimerBase::SharedPtr camera_capture_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr preproc_timer_;

    // Additional member variables
    std::string lidar_frame_;
    std::string camera_frame_;
    int right_frame_id_;
    double confidence_;
    std::string save_pic_;
    cv_bridge::CvImage::Ptr bridge_;
    std::chrono::system_clock::time_point last_cam_capture_time_;
    std::chrono::system_clock::time_point last_preproc_time_;

    // YOLO model interface
    // TODO: Define appropriate type/wrapper for YOLO model
    void* model_; // This will need to be replaced with actual YOLO model type

    // Additional helper functions
    void publishConeDetectionsLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& lidar_msg);
    std::vector<cv::Point3f> readPointCloudData(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    
    // Callback group members
    rclcpp::CallbackGroup::SharedPtr lidar_group_;
    rclcpp::CallbackGroup::SharedPtr cam_timer_group_;
    rclcpp::CallbackGroup::SharedPtr preproc_timer_group_;
    rclcpp::CallbackGroup::SharedPtr heartbeat_timer_group_;

    // Deep learning related functions
    std::string labelColor(const std::string& class_name);
    bool checkForCuda();
    void loadModel();

    // Additional initialization functions
    void initCallbackGroups() {
        lidar_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cam_timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        preproc_timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        heartbeat_timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    }

    // Parameter loading helpers
    void loadCameraParams();
    void loadDetectionParams();
    void loadTopicParams();
    void loadTimingParams();

    // Image processing utilities
    void rectifyImage(const cv::Mat& input, cv::Mat& output);
    void rectifyImageGPU(const cv::Mat& input, cv::Mat& output);
    
    // Validation functions
    bool validateImageSize(const cv::Mat& img) const;
    bool validateTransform(const geometry_msgs::msg::TransformStamped& transform) const;
    
    // Debug utilities
    void logTimingInfo(const std::string& operation, 
                      const std::chrono::system_clock::time_point& start_time);
    void publishDebugInfo(const std::string& debug_info);

    // Initialize functions
    void initHeartbeat();
    void initSubscribers();
    void initPublishers();
    void initServices();
    void initConeTemplate();
    void initTimers();

    // Callback functions
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void egoStateCallback(const utfr_msgs::msg::EgoState::SharedPtr msg);
    void cameraCaptureTimerCallback();
    void heartbeatCallback();
    void preProcCallback();

    // Processing functions
    void deepAndMatching(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    std::tuple<std::vector<cv::Rect>, std::vector<std::string>, std::vector<float>> 
    process(const cv::Mat& img);

    // Helper functions
    void publishHeartbeat();
    void publishConeDetections(const std::vector<std::vector<double>>& cone_detections);
    void publish2DProjectedDet(const std::vector<cv::Point3f>& projected_pts, 
                             const rclcpp::Time& stamp);
    void publish2DProjectedDetMatched(const std::vector<cv::Point3f>& projected_pts, 
                                    const rclcpp::Time& stamp);
    void publishUndistorted(const cv::Mat& frame);
    void displayBoundingBox(const std::vector<cv::Rect>& results_left,
                          const std::vector<std::string>& classes_left,
                          const std::vector<float>& scores_left,
                          const rclcpp::Time& img_stamp);

    // Transformation functions
    cv::Point3f tfCamAxisSwap(const cv::Point3f& point);
    cv::Point3f tfCamAxisSwapInv(const cv::Point3f& point);
    cv::Point3f point3DToImage(const cv::Point3f& point, const cv::Mat& camera_matrix);
    cv::Point3f imageTo3DPoint(const cv::Point3f& image_point, const cv::Mat& camera_matrix);
    std::vector<cv::Point3f> transformDetLidar(const std::vector<cv::Point3f>& lidar_points,
                                             const geometry_msgs::msg::TransformStamped& transform);

    // Matching functions
    cv::Mat costMtxFromBbox(const std::vector<cv::Point3f>& projected_lidar_detections,
                           const std::vector<cv::Rect>& camera_detections,
                           float depth_factor);
    std::tuple<std::vector<cv::Point3f>, std::vector<std::string>> 
    hMatching(const std::vector<cv::Point3f>& left_projected_lidar_pts,
             const std::vector<cv::Rect>& left_cam_dets,
             const std::vector<std::string>& classes_left,
             float duplicate_threshold,
             float cost_threshold,
             float depth_weight,
             float alpha);

    // Status check functions
    std::vector<int> filterPointsInFov(const std::vector<cv::Point3f>& projected_points);
    bool frameChanged(const cv::Mat& frame, const cv::Mat& prev_frame);
    bool hasVisualArtifacts(const cv::Mat& frame);
    int cameraStatus();

    // Additional member variables needed
    utfr_msgs::msg::Heartbeat heartbeat_;
    utfr_msgs::msg::Cone cone_template_;
    utfr_msgs::msg::EgoState ego_state_;
    rclcpp::Time img_stamp_;
    const int MIN_HEIGHT = 720;
    const int MIN_WIDTH = 1020;
    const float MAX_FRAME_RATE = 1.0;

    void loadParams() {
        // Declare and get parameters
        this->declare_parameter("baseline", 10.0);
        this->declare_parameter("left_camera_topic", "/left_camera/images");
        // ... Add all other parameters ...

        baseline_ = this->get_parameter("baseline").as_double();
        left_camera_topic_ = this->get_parameter("left_camera_topic").as_string();
        // ... Get all other parameters ...
    }

    void initVariables() {
        new_cam_received_ = false;
        first_img_arrived_ = false;
        img_size_ = cv::Size(1024, 576);

        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize camera capture
        if (!initCamera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
            throw std::runtime_error("Camera initialization failed");
        }

        // Initialize rectification maps
        cv::initUndistortRectifyMap(
            intrinsics_, distortion_, cv::Mat(), 
            intrinsics_, img_size_, CV_32FC1, 
            mapx_, mapy_
        );

        if (is_cuda_cv_) {
            mapx_gpu_.upload(mapx_);
            mapy_gpu_.upload(mapy_);
        }
    }

    bool initCamera() {
        for (int index = 0; index < 10; ++index) {
            cam_capture_.open(index, cv::CAP_V4L2);
            if (cam_capture_.isOpened()) {
                RCLCPP_INFO(this->get_logger(), "Connected to camera at index %d", index);
                cam_capture_.set(cv::CAP_PROP_FRAME_WIDTH, img_size_.width);
                cam_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, img_size_.height);
                cam_capture_.set(cv::CAP_PROP_FPS, 30);
                return true;
            }
        }
        return false;
    }

    // ... Add other initialization methods and processing functions ...
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerceptionNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
