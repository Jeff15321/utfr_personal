

// perception_node.cpp
#include "perception_node.hpp"

namespace perception {

PerceptionNode::PerceptionNode() 
    : Node("perception_node")
{
    loadParams();
    initVariables();
    
    // Set up camera capture timer
    using namespace std::chrono_literals;
    camera_capture_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(camera_capture_rate_)),
        std::bind(&PerceptionNode::cameraCaptureTimerCallback, this)
    );
}

void PerceptionNode::loadParams() {
    // Declare parameters with defaults
    this->declare_parameter("baseline", 10.0);
    this->declare_parameter("left_camera_topic", "/left_camera/images");
    this->declare_parameter("right_camera_topic", "/right_camera/images");
    this->declare_parameter("cone_detections_topic", "/perception/cone_detections");
    this->declare_parameter("camera_capture_rate", 30.0);
    this->declare_parameter("debug", true);
    this->declare_parameter("save_pic", false);
    this->declare_parameter("confidence", 0.70);
    this->declare_parameter("is_cuda_cv", true);
    this->declare_parameter("is_cuda_deep", true);
    
    // Get parameter values
    baseline_ = this->get_parameter("baseline").as_double();
    left_camera_topic_ = this->get_parameter("left_camera_topic").as_string();
    right_camera_topic_ = this->get_parameter("right_camera_topic").as_string();
    cone_detections_topic_ = this->get_parameter("cone_detections_topic").as_string();
    camera_capture_rate_ = this->get_parameter("camera_capture_rate").as_double();
    debug_ = this->get_parameter("debug").as_bool();
    save_pic_ = this->get_parameter("save_pic").as_bool();
    confidence_ = this->get_parameter("confidence").as_double();
    is_cuda_cv_ = this->get_parameter("is_cuda_cv").as_bool();
    is_cuda_deep_ = this->get_parameter("is_cuda_deep").as_bool();
    
    // Load matrix parameters
    // Note: You'll need to implement proper matrix parameter loading based on your YAML format
    // This is a placeholder implementation
    std::vector<double> distortion_vec = this->declare_parameter("distortion", std::vector<double>{0.0});
    std::vector<double> intrinsics_vec = this->declare_parameter("intrinsics", std::vector<double>{0.0});
    std::vector<double> rotation_vec = this->declare_parameter("rotation", std::vector<double>{0.0});
    std::vector<double> translation_vec = this->declare_parameter("translation", std::vector<double>{0.0});
    
    // Convert vectors to OpenCV matrices
    // You'll need to implement proper reshaping based on your matrix dimensions
    distortion_ = cv::Mat(distortion_vec);
    intrinsics_ = cv::Mat(intrinsics_vec).reshape(1, 3);
    rotation_ = cv::Mat(rotation_vec).reshape(1, 3);
    translation_ = cv::Mat(translation_vec);
}

void PerceptionNode::initVariables() {
    new_cam_received_ = false;
    first_img_arrived_ = false;
    img_size_ = cv::Size(1024, 576);
    last_cam_capture_time_ = 0.0;
    
    // Initialize camera undistortion maps
    cv::initUndistortRectifyMap(
        intrinsics_,
        distortion_,
        cv::Mat(),
        intrinsics_,
        img_size_,
        CV_32FC1,
        mapx_,
        mapy_
    );
    
    if (is_cuda_cv_) {
        mapx_gpu_.upload(mapx_);
        mapy_gpu_.upload(mapy_);
    }
    
    // Find and initialize camera
    if (!findCamera()) {
        throw std::runtime_error("Unable to detect camera");
    }
}

bool PerceptionNode::findCamera() {
    for (int index = 0; index < 10; ++index) {
        cam_capture_.open(index, cv::CAP_V4L2);
        if (cam_capture_.isOpened()) {
            RCLCPP_INFO(this->get_logger(), "Connected to camera at index %d", index);
            cam_capture_.set(cv::CAP_PROP_FRAME_WIDTH, img_size_.width);
            cam_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, img_size_.height);
            cam_capture_.set(cv::CAP_PROP_FPS, 30);
            return true;
        }
        cam_capture_.release();
    }
    RCLCPP_ERROR(this->get_logger(), "No cameras found");
    return false;
}

void PerceptionNode::cameraCaptureTimerCallback() {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    cv::Mat frame;
    if (!cam_capture_.read(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error: Could not read frame.");
        return;
    }
    
    img_raw_ = frame;
    new_cam_received_ = true;
    
    if (debug_) {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        double current_time = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;
        
        RCLCPP_INFO(this->get_logger(), "Capture time: %f ms", duration.count());
        RCLCPP_INFO(this->get_logger(), "Capture Hz: %f", 1.0 / (current_time - last_cam_capture_time_));
        last_cam_capture_time_ = current_time;
    }
}

bool PerceptionNode::frameChanged(const cv::Mat& frame, const cv::Mat& prev_frame) {
    cv::Mat diff, gray, blur, thresh, dilated;
    std::vector<std::vector<cv::Point>> contours;
    
    cv::absdiff(frame, prev_frame, diff);
    cv::cvtColor(diff, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
    cv::threshold(blur, thresh, 20, 255, cv::THRESH_BINARY);
    cv::dilate(thresh, dilated, cv::Mat(), cv::Point(-1, -1), 3);
    cv::findContours(dilated, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    return !contours.empty();
}

bool PerceptionNode::hasVisualArtifacts(const cv::Mat& frame) {
    cv::Mat gray, blur, thresh;
    std::vector<std::vector<cv::Point>> contours;
    
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
    cv::threshold(blur, thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::findContours(thresh, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    return !contours.empty();
}

} // namespace perception

// main.cpp

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<perception::PerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}