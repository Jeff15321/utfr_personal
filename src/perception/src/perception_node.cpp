// perception_node.cpp
#include "../include/perception_node.hpp"

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
    // Remove stereo camera parameters
    this->declare_parameter("camera_topic", "/camera/image");  // Single camera topic
    this->declare_parameter("camera_capture_rate", 30.0);
    this->declare_parameter("debug", true);
    this->declare_parameter("save_pic", false);
    this->declare_parameter("confidence", 0.70);
    this->declare_parameter("is_cuda_cv", true);
    this->declare_parameter("is_cuda_deep", true);
    this->declare_parameter("hue_low", 0);
    this->declare_parameter("hue_high", 180);
    this->declare_parameter("sat_low", 0);
    this->declare_parameter("sat_high", 255);
    this->declare_parameter("val_low", 0);
    this->declare_parameter("val_high", 255);
    this->declare_parameter("heartbeat_topic", "/perception/heartbeat");
    this->declare_parameter("debug_topic", "/perception/debug");
    this->declare_parameter("heartbeat_rate", 1.0);
    this->declare_parameter("min_cone_area", 100.0);
    this->declare_parameter("max_cone_area", 10000.0);
    this->declare_parameter("min_cone_width", 10);
    this->declare_parameter("max_cone_width", 100);
    this->declare_parameter("min_cone_height", 20);
    this->declare_parameter("max_cone_height", 200);
    
    // Get parameter values
    camera_topic_ = this->get_parameter("camera_topic").as_string();
    camera_capture_rate_ = this->get_parameter("camera_capture_rate").as_double();
    debug_ = this->get_parameter("debug").as_bool();
    save_pic_ = this->get_parameter("save_pic").as_bool();
    confidence_ = this->get_parameter("confidence").as_double();
    is_cuda_cv_ = this->get_parameter("is_cuda_cv").as_bool();
    is_cuda_deep_ = this->get_parameter("is_cuda_deep").as_bool();
    hue_low_ = this->get_parameter("hue_low").as_int();
    hue_high_ = this->get_parameter("hue_high").as_int();
    sat_low_ = this->get_parameter("sat_low").as_int();
    sat_high_ = this->get_parameter("sat_high").as_int();
    val_low_ = this->get_parameter("val_low").as_int();
    val_high_ = this->get_parameter("val_high").as_int();
    heartbeat_topic_ = this->get_parameter("heartbeat_topic").as_string();
    debug_topic_ = this->get_parameter("debug_topic").as_string();
    heartbeat_rate_ = this->get_parameter("heartbeat_rate").as_double();
    min_cone_area_ = this->get_parameter("min_cone_area").as_double();
    max_cone_area_ = this->get_parameter("max_cone_area").as_double();
    min_cone_width_ = this->get_parameter("min_cone_width").as_int();
    max_cone_width_ = this->get_parameter("max_cone_width").as_int();
    min_cone_height_ = this->get_parameter("min_cone_height").as_int();
    max_cone_height_ = this->get_parameter("max_cone_height").as_int();
    
    // Get array parameters with proper sizes
    std::vector<double> distortion_vec = this->declare_parameter("distortion", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});
    std::vector<double> intrinsics_vec = this->declare_parameter("intrinsics", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    std::vector<double> rotation_vec = this->declare_parameter("rotation", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    std::vector<double> translation_vec = this->declare_parameter("translation", std::vector<double>{0.0, 0.0, 0.0});
    
    // Convert vectors to OpenCV matrices with proper sizes
    distortion_ = cv::Mat(distortion_vec).reshape(1, 5);  // 5x1 distortion coefficients
    intrinsics_ = cv::Mat(intrinsics_vec).reshape(1, 3);  // 3x3 camera matrix
    rotation_ = cv::Mat(rotation_vec).reshape(1, 3);      // 3x3 rotation matrix
    translation_ = cv::Mat(translation_vec).reshape(1, 3); // 3x1 translation vector
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
    
    // Check CUDA availability before using GPU features
    if (is_cuda_cv_) {
        if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
            mapx_gpu_.upload(mapx_);
            mapy_gpu_.upload(mapy_);
        } else {
            RCLCPP_WARN(this->get_logger(), "CUDA not available, falling back to CPU");
            is_cuda_cv_ = false;
        }
    }
    
    // Find and initialize camera
    if (!findCamera()) {
        throw std::runtime_error("Unable to detect camera");
    }
    
    // Initialize publishers and timers
    initPublishers();
    initTimers();
    
    // Initialize performance metrics
    metrics_.fps = 0.0;
    metrics_.processing_time = 0.0;
    metrics_.frame_count = 0;
    metrics_.last_frame_time = std::chrono::steady_clock::now();
}

//Look through ten cameras to find one that is open
bool PerceptionNode::findCamera() {
    for (int index = 0; index < 10; ++index) {
        cam_capture_.open(index, cv::CAP_V4L2); //Jeff: assigns camera to cam_capture_
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
    
    // Add this line to process the image
    processImage();
    
    // Display the captured frame
    cv::imshow("Camera Feed", frame);
    cv::waitKey(1); 
    
    if (debug_) {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        double current_time = static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count()) / 1000.0;
        
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

void PerceptionNode::processImage() {
    if (!new_cam_received_) {
        RCLCPP_DEBUG(this->get_logger(), "No new camera frame received, skipping processing");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Starting image processing pipeline...");
    auto start_time = std::chrono::high_resolution_clock::now();

    // Convert to HSV and apply color mask
    cv::cvtColor(img_raw_, hsv_, cv::COLOR_BGR2HSV);
    applyColorMask(hsv_);
    
    // Detect cones
    auto cone_centers = detectCones(mask_);
    
    // Track and classify cones
    trackAndClassifyCones(cone_centers);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    RCLCPP_INFO(this->get_logger(), "Processed %zu cones (%zu tracked) in %ld ms", 
                cone_centers.size(), tracked_cones_.size(), duration.count());
    
    if (debug_) {
        cv::Mat debug_img = img_raw_.clone();
        for (const auto& cone : tracked_cones_) {
            cv::Scalar color = (cone.color == "yellow") ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 0, 0);
            cv::circle(debug_img, cone.position, 5, color, -1);
            cv::putText(debug_img, 
                       "ID:" + std::to_string(cone.tracking_id) + " F:" + std::to_string(cone.frames_tracked),
                       cv::Point(cone.position.x + 10, cone.position.y),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }
        cv::imshow("Tracked Cones", debug_img);
        cv::imshow("HSV Mask", mask_);
        cv::waitKey(1);
    }
}

void PerceptionNode::applyColorMask(const cv::Mat& frame) {
    cv::split(frame, hsv_channels_);
    
    RCLCPP_DEBUG(this->get_logger(), "Applying color thresholds - H:[%d-%d], S:[%d-%d], V:[%d-%d]",
        hue_low_, hue_high_, sat_low_, sat_high_, val_low_, val_high_);
    
    cv::Mat hue_mask = hsv_channels_[0] >= hue_low_ & hsv_channels_[0] <= hue_high_;
    cv::Mat sat_mask = hsv_channels_[1] >= sat_low_ & hsv_channels_[1] <= sat_high_;
    cv::Mat val_mask = hsv_channels_[2] >= val_low_ & hsv_channels_[2] <= val_high_;
    
    mask_ = hue_mask & sat_mask & val_mask;
}

std::vector<cv::Point2f> PerceptionNode::detectCones(const cv::Mat& frame) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(frame, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    RCLCPP_DEBUG(this->get_logger(), "Found %zu contours before filtering", contours.size());
    
    std::vector<cv::Point2f> cone_centers;
    for (const auto& contour : contours) {
        if (isValidContour(contour)) {
            cone_centers.push_back(getContourCentroid(contour));
        }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Filtered down to %zu valid cones", cone_centers.size());
    return cone_centers;
}

bool PerceptionNode::isValidContour(const std::vector<cv::Point>& contour) {
    double area = cv::contourArea(contour);
    bool is_valid = area > 100.0;  // Minimum area threshold
    
    if (is_valid) {
        RCLCPP_DEBUG(this->get_logger(), "Valid contour found with area: %.2f", area);
    }
    
    return is_valid;
}

cv::Point2f PerceptionNode::getContourCentroid(const std::vector<cv::Point>& contour) {
    cv::Moments m = cv::moments(contour);
    return cv::Point2f(m.m10/m.m00, m.m01/m.m00);
}

void PerceptionNode::trackAndClassifyCones(const std::vector<cv::Point2f>& detected_centers) {
    std::vector<bool> detected_matched(detected_centers.size(), false);
    std::vector<TrackedCone> new_tracked_cones;
    
    // Update existing tracks
    for (auto& tracked_cone : tracked_cones_) {
        bool found_match = false;
        double min_dist = max_tracking_distance_;
        size_t best_match_idx = 0;
        
        // Find closest detection
        for (size_t i = 0; i < detected_centers.size(); ++i) {
            if (detected_matched[i]) continue;
            
            double dist = cv::norm(tracked_cone.position - detected_centers[i]);
            if (dist < min_dist) {
                min_dist = dist;
                best_match_idx = i;
                found_match = true;
            }
        }
        
        if (found_match) {
            RCLCPP_DEBUG(this->get_logger(), "Updated cone ID %d (frames: %d, dist: %.2f)", 
                        tracked_cone.tracking_id, tracked_cone.frames_tracked, min_dist);
            
            tracked_cone.position = detected_centers[best_match_idx];
            tracked_cone.frames_tracked++;
            detected_matched[best_match_idx] = true;
            new_tracked_cones.push_back(tracked_cone);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Lost track of cone ID %d after %d frames", 
                        tracked_cone.tracking_id, tracked_cone.frames_tracked);
        }
    }
    
    // Create new tracks for unmatched detections
    for (size_t i = 0; i < detected_centers.size(); ++i) {
        if (!detected_matched[i]) {
            TrackedCone new_cone;
            new_cone.position = detected_centers[i];
            new_cone.tracking_id = next_tracking_id_++;
            new_cone.color = classifyConeColor(detected_centers[i]);
            new_cone.frames_tracked = 1;
            
            RCLCPP_INFO(this->get_logger(), "New cone detected - ID: %d, Color: %s, Position: (%.2f, %.2f)",
                       new_cone.tracking_id, new_cone.color.c_str(), 
                       new_cone.position.x, new_cone.position.y);
            
            new_tracked_cones.push_back(new_cone);
        }
    }
    
    tracked_cones_ = new_tracked_cones;
}

std::string PerceptionNode::classifyConeColor(const cv::Point2f& position) {
    double avg_hue = getAverageHue(position);
    std::string color = (avg_hue < 30) ? "yellow" : "blue";  // Simple threshold for yellow/blue
    RCLCPP_DEBUG(this->get_logger(), "Classified cone at (%.2f, %.2f) as %s (hue: %.2f)",
                 position.x, position.y, color.c_str(), avg_hue);
    return color;
}

double PerceptionNode::getAverageHue(const cv::Point2f& position, int region_size) {
    cv::Rect roi(
        cv::Point(std::max(0, (int)position.x - region_size/2),
                 std::max(0, (int)position.y - region_size/2)),
        cv::Size(region_size, region_size)
    );
    
    // Make sure ROI is within image bounds
    roi &= cv::Rect(0, 0, hsv_.cols, hsv_.rows);
    
    if (roi.area() > 0) {
        cv::Mat roi_hue = hsv_(roi);
        cv::Scalar mean = cv::mean(roi_hue);
        return mean[0];  // Hue channel
    }
    return 0.0;
}

void PerceptionNode::initPublishers() {
    // Initialize all publishers
    cone_detections_pub_ = this->create_publisher<utfr_msgs::msg::ConeDetections>(
        "/perception/cone_detections", 10);
    heartbeat_pub_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
        heartbeat_topic_, 10);
    debug_pub_ = this->create_publisher<utfr_msgs::msg::PerceptionDebug>(
        debug_topic_, 10);
}

void PerceptionNode::initTimers() {
    // Initialize heartbeat timer
    using namespace std::chrono_literals;
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(heartbeat_rate_ * 1000.0)),
        std::bind(&PerceptionNode::publishHeartbeat, this)
    );
}

void PerceptionNode::publishHeartbeat() {
    // Implement heartbeat publishing
}

void PerceptionNode::publishDebugInfo() {
    // Implement debug info publishing
}

bool PerceptionNode::isValidCone(const std::vector<cv::Point>& contour) {
    // Implement cone validation logic
    return false;
}

void PerceptionNode::filterContours(std::vector<std::vector<cv::Point>>& contours) {
    // Implement contour filtering
}

cv::Rect PerceptionNode::getBoundingBox(const std::vector<cv::Point>& contour) {
    // Get bounding box for contour
    return cv::boundingRect(contour);
}

double PerceptionNode::getAspectRatio(const cv::Rect& bbox) {
    // Calculate aspect ratio
    return static_cast<double>(bbox.width) / bbox.height;
}

void PerceptionNode::updateMetrics() {
    // Update performance metrics
}

void PerceptionNode::drawDebugInfo(cv::Mat& debug_frame) {
    // Draw debug information on frame
}

void PerceptionNode::showDebugWindows() {
    // Show debug windows if enabled
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