#include "../include/perception_node.hpp"
//TODO: fill in the values for distortion, intrinsics, rotation, and translation and correctly distort
//TODO: fix can't connect to CUDA error

// # 1. Camera capture and initialization
// - Sets up camera parameters (resolution, FPS)
// - Loads calibration parameters
// - Initializes CUDA if available

// # 2. Image preprocessing
// - Undistorts image using camera calibration
// - Converts to proper format for model input
// - Resizes to model input size (640x640)
// - Normalizes pixel values (0-1)

// # 3. Model inference
// - Runs YOLO model on preprocessed image
// - Gets bounding boxes, classes, and confidence scores

// # 4. Post-processing
// - Filters detections by confidence threshold
// - Converts normalized coordinates to image coordinates
// - Maps class indices to class names

// # 5. Debug visualization and publishing
// - Draws bounding boxes on debug image
// - Publishes debug messages with detections
// - Publishes undistorted image if debug enabled
namespace perception {

// ==========================================
// Constructor
// ==========================================
PerceptionNode::PerceptionNode() 
    : Node("perception_node_cpp")
{
    // Initialize tf2 components first
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Rest of initialization
    loadParams();
    initVariables();
    initSubscribers();
    initPublishers();
    initTimers();
    initHeartbeat();
    
    // Set up camera capture timer
    using namespace std::chrono_literals;
    camera_capture_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(camera_capture_rate_)),
        std::bind(&PerceptionNode::cameraCaptureTimerCallback, this)
    );

    // Add these lines
    this->declare_parameter("model_path", "");
    std::string model_path = this->get_parameter("model_path").as_string();
    
    // Initialize detector with model path
    detector_ = std::make_unique<DeepDetector>(model_path, confidence_);

    // Add after model initialization
    if (!detector_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize YOLO detector");
        throw std::runtime_error("Failed to initialize YOLO detector");
    }
    RCLCPP_INFO(this->get_logger(), "Successfully initialized YOLO detector");
}

// ==========================================
// Initialization Functions
// ==========================================
void PerceptionNode::loadParams() {
    // Declare parameters - real values will come from config.yaml
    this->declare_parameter<std::string>("camera_topic", "");  // Empty default
    this->declare_parameter<double>("camera_capture_rate", 0.0);  // Zero default
    this->declare_parameter<double>("heartbeat_rate", 0.0);
    this->declare_parameter<bool>("debug", false);
    
    // Declare topic names
    this->declare_parameter<std::string>("heartbeat_topic", "");
    this->declare_parameter<std::string>("debug_topic", "");
    this->declare_parameter<std::string>("processed_lidar_topic", "");
    
    // Initialize vectors with empty/identity defaults
    this->declare_parameter<std::vector<double>>("distortion", std::vector<double>(5, 0.0));  // 5 zeros
    this->declare_parameter<std::vector<double>>("intrinsics", std::vector<double>(9, 0.0));  // 9 zeros
    this->declare_parameter<std::vector<double>>("rotation", std::vector<double>(9, 0.0));    // 9 zeros
    this->declare_parameter<std::vector<double>>("translation", std::vector<double>(3, 0.0)); // 3 zeros

    // Get values from parameters (these will come from config.yaml)
    camera_topic_ = this->get_parameter("camera_topic").as_string();
    camera_capture_rate_ = this->get_parameter("camera_capture_rate").as_double();
    heartbeat_rate_ = this->get_parameter("heartbeat_rate").as_double();
    debug_ = this->get_parameter("debug").as_bool();
    
    // Get topic names
    heartbeat_topic_ = this->get_parameter("heartbeat_topic").as_string();
    debug_topic_ = this->get_parameter("debug_topic").as_string();
    processed_lidar_topic_ = this->get_parameter("processed_lidar_topic").as_string();

    // Get arrays and convert to OpenCV matrices
    auto distortion_vec = this->get_parameter("distortion").as_double_array();
    auto intrinsics_vec = this->get_parameter("intrinsics").as_double_array();
    auto rotation_vec = this->get_parameter("rotation").as_double_array();
    auto translation_vec = this->get_parameter("translation").as_double_array();

    // Convert vectors to OpenCV matrices
    distortion_ = cv::Mat(distortion_vec).reshape(1, 5);
    intrinsics_ = cv::Mat(intrinsics_vec).reshape(1, 3);
    rotation_ = cv::Mat(rotation_vec).reshape(1, 3);
    translation_ = cv::Mat(translation_vec).reshape(1, 3);

    // Print loaded values for verification
    RCLCPP_INFO(this->get_logger(), "Loaded parameters from config.yaml");
    RCLCPP_INFO(this->get_logger(), "  camera_topic: %s", camera_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  camera_capture_rate: %.2f", camera_capture_rate_);
    RCLCPP_INFO(this->get_logger(), "  debug: %s", debug_ ? "true" : "false");
    
    // Print matrix dimensions for verification
    RCLCPP_INFO(this->get_logger(), "Matrix dimensions:");
    RCLCPP_INFO(this->get_logger(), "  distortion: %dx%d", distortion_.rows, distortion_.cols);
    RCLCPP_INFO(this->get_logger(), "  intrinsics: %dx%d", intrinsics_.rows, intrinsics_.cols);
    RCLCPP_INFO(this->get_logger(), "  rotation: %dx%d", rotation_.rows, rotation_.cols);
    RCLCPP_INFO(this->get_logger(), "  translation: %dx%d", translation_.rows, translation_.cols);
}

void PerceptionNode::initVariables() {
    new_cam_received_ = false;
    first_img_arrived_ = false;
    img_size_ = cv::Size(1024, 576);
    last_cam_capture_time_ = 0.0;
    
    // Initialize camera undistortion maps
    cv::initUndistortRectifyMap(
        intrinsics_,      // Input: 3x3 camera matrix
        distortion_,      // Input: 5x1 distortion coefficients
        cv::Mat(),        // Input: 3x3 rectification matrix (optional, empty here)
        intrinsics_,      // Input: 3x3 new camera matrix
        img_size_,        // Input: size of undistorted image
        CV_32FC1,        // Input: type of map (32-bit float)
        mapx_,           // Output: x-coordinate mapping
        mapy_            // Output: y-coordinate mapping
    );
    
    // Always use CPU
    is_cuda_cv_ = false;
    RCLCPP_INFO(this->get_logger(), "Using CPU for image processing");
    
    // Find and initialize camera
    if (!findCamera()) {
        throw std::runtime_error("Unable to detect camera");
    }
    
    // Initialize performance metrics
    metrics_.fps = 0.0;
    metrics_.processing_time = 0.0;
    metrics_.frame_count = 0;
    metrics_.last_frame_time = std::chrono::steady_clock::now();
}

void PerceptionNode::initSubscribers() {
    // LiDAR subscriber with callback group
    processed_lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        processed_lidar_topic_, 1,
        std::bind(&PerceptionNode::lidarCallback, this, std::placeholders::_1));
        
    // Ego state subscriber
    ego_state_subscriber_ = this->create_subscription<utfr_msgs::msg::EgoState>(
        "synced_ego_state", 1,
        std::bind(&PerceptionNode::egoStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Initialized subscribers");
}

void PerceptionNode::initPublishers() {
    // Initialize all publishers
    cone_detections_pub_ = this->create_publisher<utfr_msgs::msg::ConeDetections>(
        "/perception/cone_detections", 10);
    debug_pub_ = this->create_publisher<utfr_msgs::msg::PerceptionDebug>(
        debug_topic_, 10);
    
    heartbeat_pub_ = this->create_publisher<utfr_msgs::msg::Heartbeat>(
        heartbeat_topic_, 10);
        
    // Add these publishers
    undistorted_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/perception/undistorted", 1);
    lidar_projection_pub_ = this->create_publisher<utfr_msgs::msg::PerceptionDebug>(
        "/perception/lidar_projection", 1);
    lidar_projection_matched_pub_ = this->create_publisher<utfr_msgs::msg::PerceptionDebug>(
        "/perception/lidar_projection_matched", 1);
}

void PerceptionNode::initTimers() {
    // Initialize heartbeat timer
    using namespace std::chrono_literals;
    heartbeat_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(heartbeat_rate_ * 1000.0)),
        std::bind(&PerceptionNode::publishHeartbeat, this)
    );
}

void PerceptionNode::initHeartbeat() {
    // Initialize heartbeat message
    //Jeff: module.data identifies which module/node is sending the heartbeat
    heartbeat_msg_.module.data = "perception_node_cpp";
    heartbeat_msg_.header.stamp = this->now();
    heartbeat_msg_.status = STATUS_UNINITIALIZED;
    
    RCLCPP_INFO(this->get_logger(), "Initialized heartbeat publisher");
}


// ==========================================
// Publishing Functions
// ==========================================
void PerceptionNode::publishHeartbeat() {
    // Get camera status
    int status = cameraStatus();
    
    // Update heartbeat message
    heartbeat_msg_.header.stamp = this->now();
    heartbeat_msg_.status = status;
    
    // Add debug info
    if (debug_) {
        std::string status_str;
        switch (status) {
            case STATUS_UNINITIALIZED:
                status_str = "UNINITIALIZED";
                break;
            case STATUS_ACTIVE:
                status_str = "ACTIVE";
                break;
            case STATUS_FATAL:
                status_str = "FATAL";
                break;
            default:
                status_str = "UNKNOWN";
        }
        RCLCPP_DEBUG(this->get_logger(), "Publishing heartbeat - Status: %s", status_str.c_str());
    }
    
    // Publish heartbeat
    heartbeat_pub_->publish(heartbeat_msg_);
}

void PerceptionNode::publishDebugInfo() {
    // Implement debug info publishing
}

// ==========================================
// Camera Functions
// ==========================================
bool PerceptionNode::findCamera() {
    for (int index = 0; index < 10; ++index) {
        cam_capture_.open(index, cv::CAP_V4L2); //Jeff: assigns camera to cam_capture_
        if (cam_capture_.isOpened()) {
            // RCLCPP_INFO(this->get_logger(), "Connected to camera at index %d", index);
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
    
    // 1. Camera capture
    cv::Mat frame;
    if (!cam_capture_.read(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Error: Could not read frame.");
        return;
    }
    
    // Store raw image
    img_raw_ = frame;
    img_ = frame;
    new_cam_received_ = true;
    
    if (!first_img_arrived_) {
        first_img_arrived_ = true;
        RCLCPP_INFO(this->get_logger(), "First camera frame received");
    }

    // 2. Process image through detection pipeline
    auto [boxes, classes, scores] = process_image(frame);
    
    // 3. Visualize results if debug enabled
    if (debug_) {
        displayBoundingBox(boxes, classes, scores, this->now());
    }
    
    // Update metrics
    if (debug_) {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        metrics_.processing_time = duration.count() / 1000.0;
        metrics_.fps = 1000.0 / duration.count();
        metrics_.frame_count++;
    }
}

std::tuple<std::vector<cv::Rect>, std::vector<std::string>, std::vector<float>>
PerceptionNode::process_image(const cv::Mat& img) {
    // 1. Undistort image using calibration
    cv::Mat undistorted;
    cv::undistort(img, undistorted, intrinsics_, distortion_);
    
    // 2. Run YOLO detection
    auto [boxes, classes, scores] = detector_->detect(undistorted);
    
    // 3. Publish undistorted image if debug enabled
    if (debug_) {
        sensor_msgs::msg::Image::SharedPtr img_msg = 
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", undistorted)
            .toImageMsg();
        undistorted_pub_->publish(*img_msg);
    }
    
    return {boxes, classes, scores};
}

void PerceptionNode::displayBoundingBox(
    const std::vector<cv::Rect>& results_left,
    const std::vector<std::string>& classes_left,
    const std::vector<float>& scores_left,
    const rclcpp::Time& img_stamp) {
    
    // Create debug message
    utfr_msgs::msg::PerceptionDebug debug_msg;
    debug_msg.header.stamp = img_stamp;
    debug_msg.header.frame_id = camera_frame_;

    // Add detections to debug message
    for (size_t i = 0; i < results_left.size(); i++) {
        // Convert cv::Rect to BoundingBox message
        utfr_msgs::msg::BoundingBox bbox;
        bbox.x = results_left[i].x;
        bbox.y = results_left[i].y;
        bbox.width = results_left[i].width;
        bbox.height = results_left[i].height;
        debug_msg.left.push_back(bbox);
    }

    // Draw boxes on debug frame if enabled
    if (debug_) {
        cv::Mat debug_frame = img_.clone();
        for (size_t i = 0; i < results_left.size(); i++) {
            cv::rectangle(debug_frame, results_left[i], cv::Scalar(0, 255, 0), 2);
            std::string label = classes_left[i] + " " + 
                              std::to_string(scores_left[i]);
            cv::putText(debug_frame, label, 
                       cv::Point(results_left[i].x, results_left[i].y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }
        cv::imshow("Detections", debug_frame);
        cv::waitKey(1);
    }

    // Publish debug visualization
    debug_pub_->publish(debug_msg);
}

// ==========================================
// Callback Functions
// ==========================================
void PerceptionNode::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    lidar_msg_count_++;
    double current_time = this->now().seconds();
    
    if (last_lidar_time_ > 0) {
        double dt = current_time - last_lidar_time_;
        RCLCPP_INFO(this->get_logger(), 
            "LiDAR Message Stats:\n"
            "  Total messages: %d\n"
            "  Current rate: %.2f Hz\n"
            "  Points in cloud: %u\n"
            "  Frame ID: %s",
            lidar_msg_count_,
            1.0 / dt,
            msg->width * msg->height,
            msg->header.frame_id.c_str());
    }
    
    last_lidar_time_ = current_time;
    
    if (!img_.empty()) {
        // TODO: Implement deep_and_matching functionality
        // deep_and_matching(msg);
    }
}

void PerceptionNode::egoStateCallback(const utfr_msgs::msg::EgoState::SharedPtr msg) {
    ego_state_msg_count_++;
    RCLCPP_INFO(this->get_logger(), 
        "Ego State Message Received (#%d)", 
        ego_state_msg_count_);
    ego_state_ = *msg;
}

// ==========================================
// Image Processing Functions
// ==========================================
void PerceptionNode::processImage() {
    if (!new_cam_received_) {
        RCLCPP_DEBUG(this->get_logger(), "No new camera frame received, skipping processing");
        return;
    }

    //terminal message
    // RCLCPP_INFO(this->get_logger(), "Starting image processing pipeline...");
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
    
    //terminal message
    // RCLCPP_INFO(this->get_logger(), "Processed %zu cones (%zu tracked) in %ld ms", 
    //             cone_centers.size(), tracked_cones_.size(), duration.count());
    
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
    
    // Add parentheses around comparisons
    cv::Mat hue_mask = (hsv_channels_[0] >= hue_low_) & (hsv_channels_[0] <= hue_high_);
    cv::Mat sat_mask = (hsv_channels_[1] >= sat_low_) & (hsv_channels_[1] <= sat_high_);
    cv::Mat val_mask = (hsv_channels_[2] >= val_low_) & (hsv_channels_[2] <= val_high_);
    
    mask_ = hue_mask & sat_mask & val_mask;
}

// ==========================================
// Cone Detection and Tracking Functions
// ==========================================
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
            
            // RCLCPP_INFO(this->get_logger(), "New cone detected - ID: %d, Color: %s, Position: (%.2f, %.2f)",
            //            new_cone.tracking_id, new_cone.color.c_str(), 
            //            new_cone.position.x, new_cone.position.y);
            
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

// ==========================================
// Frame Analysis Functions
// ==========================================
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


// ==========================================
// Utility Functions
// ==========================================
int PerceptionNode::cameraStatus() {
    if (first_img_arrived_ == false) {
        return STATUS_UNINITIALIZED;
    }
    else if (img_.empty()) {
        return STATUS_UNINITIALIZED;
    }
    // Check if frame is too small
    else if (img_.rows < MIN_HEIGHT || img_.cols < MIN_WIDTH) {
        return STATUS_FATAL;
    }
    else if (hasVisualArtifacts(img_)) {
        return STATUS_FATAL;
    }
    
    // Check if frame has changed significantly
    bool frame_changed = frameChanged(img_, previous_img_);
    
    // Update previous frame
    previous_img_ = img_.clone();
    
    if (!frame_changed) {
        return STATUS_UNINITIALIZED;
    }
    
    return STATUS_ACTIVE;
}

void PerceptionNode::updateMetrics() {
    // Update performance metrics
}

void PerceptionNode::drawDebugInfo(cv::Mat& debug_frame) {
    // Silence warning until implemented
    (void)debug_frame;
    // TODO: Implement debug visualization
}

void PerceptionNode::showDebugWindows() {
    // Show debug windows if enabled
}

void PerceptionNode::printSubscriberStats() {
    RCLCPP_INFO(this->get_logger(),
        "\n=== Subscriber Statistics ===\n"
        "LiDAR Messages: %d\n"
        "Ego State Messages: %d\n"
        "========================",
        lidar_msg_count_,
        ego_state_msg_count_);
}

void PerceptionNode::deepAndMatching(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    // Silence warning until implemented
    (void)msg;
    // TODO: Implement deep learning and matching
}

std::vector<std::tuple<cv::Point2f, std::string, float>> 
PerceptionNode::process(const cv::Mat& img) {
    // Silence warning until implemented
    (void)img;
    // TODO: Implement deep learning inference
    return {};
}

void PerceptionNode::publish_cone_dets(
    const std::vector<std::tuple<cv::Point3f, std::string, float>>& cone_detections) {
    // TODO: Implement cone detection publishing
    // 1. Create ConeDetections message
    // 2. Sort cones by type (blue, yellow, orange)
    // 3. Fill message fields
    // 4. Publish message
}

void PerceptionNode::publish_cone_dets_lidar(
    const sensor_msgs::msg::PointCloud2::SharedPtr& lidar_msg) {
    // TODO: Implement lidar-only cone detection publishing
    // 1. Extract point cloud data
    // 2. Create ConeDetections message
    // 3. Convert points to cone detections
    // 4. Publish message
}

void PerceptionNode::publish_2d_projected_det(
    const std::vector<cv::Point2f>& projected_pts,
    const rclcpp::Time& stamp) {
    // Silence warnings until implemented
    (void)projected_pts;
    (void)stamp;
    // TODO: Implement projection visualization
}

void PerceptionNode::publish_2d_projected_det_matched(
    const std::vector<cv::Point2f>& projected_pts,
    const rclcpp::Time& stamp) {
    // Silence warnings until implemented
    (void)projected_pts;
    (void)stamp;
    // TODO: Implement matched projection visualization
}

} // namespace perception

// ==========================================
// Main Function
// ==========================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<perception::PerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}