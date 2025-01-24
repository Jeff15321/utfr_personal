#include <rclcpp/rclcpp.hpp>
#include <memory>
// #include <opencv2/opencv.hpp>
#include <vector>

class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode() : Node("perception_node") {
        // Initialize callback groups
        lidar_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cam_timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        preproc_timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        heartbeat_timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        RCLCPP_INFO(this->get_logger(), "JEFFFFF Perception Node...");
        
        loadParams();
        initVariables();
        initHeartbeat();
        initSubscribers();
        initPublishers();
        initServices();
        initConeTemplate();
        initTimers();
        test();
    }

private:
    void initVariables() {}
    
    void initHeartbeat() {}
    
    void initSubscribers() {}
    
    void initPublishers() {}
    
    void initServices() {}
    
    void initConeTemplate() {}
    
    void initTimers() {}
    
    void test() {
        RCLCPP_INFO(this->get_logger(), "Perception Node Initialized Successfully!");
    }
    void loadParams() {
        // Declare parameters with default values
        this->declare_parameter("baseline", 10.0);
        this->declare_parameter("left_camera_topic", "/left_camera/images");
        this->declare_parameter("right_camera_topic", "/right_camera/images");
        this->declare_parameter("cone_detections_topic", "/perception/cone_detections");
        this->declare_parameter("heartbeat_topic", "/perception/heartbeat");
        this->declare_parameter("processed_lidar_topic", "/lidar_pipeline/clustered");
        this->declare_parameter("camera_capture_rate", 0.0);
        this->declare_parameter("heartbeat_rate", 0.0);
        this->declare_parameter("debug", true);
        this->declare_parameter<std::vector<double>>("distortion", {0.0});
        this->declare_parameter<std::vector<double>>("intrinsics", {0.0});
        this->declare_parameter<std::vector<double>>("rotation", {0.0});
        this->declare_parameter<std::vector<double>>("translation", {0.0});
        this->declare_parameter("save_pic", "False");
        this->declare_parameter("confidence", 0.70);
        this->declare_parameter("perception_debug_topic", "/perception/debug");
        this->declare_parameter<std::vector<double>>("cone_heights", {0.0});
        this->declare_parameter("lidar_only_detection", false);
        this->declare_parameter("is_cuda_cv", true);
        this->declare_parameter("is_cuda_deep", true);

        // Get parameter values
        baseline_ = this->get_parameter("baseline").as_double();
        left_camera_topic_ = this->get_parameter("left_camera_topic").as_string();
        right_camera_topic_ = this->get_parameter("right_camera_topic").as_string();
        cone_detections_topic_ = this->get_parameter("cone_detections_topic").as_string();
        heartbeat_topic_ = this->get_parameter("heartbeat_topic").as_string();
        processed_lidar_topic_ = this->get_parameter("processed_lidar_topic").as_string();
        camera_capture_rate_ = this->get_parameter("camera_capture_rate").as_double();
        heartbeat_rate_ = this->get_parameter("heartbeat_rate").as_double();
        debug_ = this->get_parameter("debug").as_bool();
        
        // Get array parameters
        auto distortion_vec = this->get_parameter("distortion").as_double_array();
        auto intrinsics_vec = this->get_parameter("intrinsics").as_double_array();
        auto rotation_vec = this->get_parameter("rotation").as_double_array();
        auto translation_vec = this->get_parameter("translation").as_double_array();
        
        // Skip matrix conversions for now since OpenCV is commented out
        
        // Get remaining parameters
        save_pic_ = this->get_parameter("save_pic").as_string();
        confidence_ = this->get_parameter("confidence").as_double();
        perception_debug_topic_ = this->get_parameter("perception_debug_topic").as_string();
        cone_heights_ = this->get_parameter("cone_heights").as_double_array();
        lidar_only_detection_ = this->get_parameter("lidar_only_detection").as_bool();
        is_cuda_cv_ = this->get_parameter("is_cuda_cv").as_bool();
        is_cuda_deep_ = this->get_parameter("is_cuda_deep").as_bool();
    }

    // Member variables
    rclcpp::CallbackGroup::SharedPtr lidar_group_;
    rclcpp::CallbackGroup::SharedPtr cam_timer_group_;
    rclcpp::CallbackGroup::SharedPtr preproc_timer_group_;
    rclcpp::CallbackGroup::SharedPtr heartbeat_timer_group_;

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
    // cv::Mat distortion_;
    // cv::Mat intrinsics_;
    // cv::Mat rotation_;
    // cv::Mat translation_;
    std::string save_pic_;
    double confidence_;
    std::string perception_debug_topic_;
    std::vector<double> cone_heights_;
    bool lidar_only_detection_;
    bool is_cuda_cv_;
    bool is_cuda_deep_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

