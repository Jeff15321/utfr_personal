#include "../include/deep.hpp"

namespace perception {

DeepDetector::DeepDetector(const std::string& model_path, float conf_threshold) 
    : conf_threshold_(conf_threshold), use_cuda_(false) {
    
    // Check if model file exists
    if (model_path.empty()) {
        throw std::runtime_error("Model path is empty");
    }
    
    std::ifstream f(model_path.c_str());
    if (!f.good()) {
        throw std::runtime_error("Model file not found: " + model_path);
    }

    class_names_ = {
        "blue_cone", "large_orange_cone", "orange_cone", 
        "unknown_cone", "yellow_cone"
    };

    loadModel(model_path);
}

void DeepDetector::loadModel(const std::string& model_path) {
    try {
        // YOLOv8 specific settings
        net_ = cv::dnn::readNetFromONNX(model_path);
        
        if (net_.empty()) {
            throw std::runtime_error("Failed to load YOLOv8 model: " + model_path);
        }

        // YOLOv8 specific settings
        net_.setPreferableBackend(use_cuda_ ? cv::dnn::DNN_BACKEND_CUDA : cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(use_cuda_ ? cv::dnn::DNN_TARGET_CUDA : cv::dnn::DNN_TARGET_CPU);

        RCLCPP_INFO(rclcpp::get_logger("DeepDetector"), 
            "Loaded YOLOv8 model: %s (CUDA: %s)", 
            model_path.c_str(), use_cuda_ ? "enabled" : "disabled");

    } catch (const cv::Exception& e) {
        throw std::runtime_error("Failed to load YOLOv8 model: " + std::string(e.what()));
    }
}

std::tuple<std::vector<cv::Rect>, std::vector<std::string>, std::vector<float>>
DeepDetector::detect(const cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(detector_mutex_);
    std::vector<cv::Rect> bboxes;
    std::vector<std::string> classes;
    std::vector<float> scores;
    
    try {
        // YOLOv8 specific preprocessing
        cv::Mat blob = cv::dnn::blobFromImage(frame, scale_, input_size_, 
            cv::Scalar(0,0,0), true, false);
        
        net_.setInput(blob);
        
        // YOLOv8 has a single output
        cv::Mat output = net_.forward();
        
        // YOLOv8 output format: [xywh, conf, class_scores]
        float* data = (float*)output.data;
        const int dimensions = output.cols;  // Should be 85 for YOLOv8
        const int rows = output.rows;        // Number of detections
        
        // Process YOLOv8 detections
        for (int i = 0; i < rows; ++i) {
            float* row = data + i * dimensions;
            float conf = row[4];  // Object confidence
            
            if (conf >= conf_threshold_) {
                // Get class scores
                cv::Mat scores_mat(1, class_names_.size(), CV_32F, row + 5);
                cv::Point class_id;
                double max_class_score;
                cv::minMaxLoc(scores_mat, nullptr, &max_class_score, nullptr, &class_id);
                
                if (max_class_score > conf_threshold_) {
                    // YOLOv8 outputs normalized xywh
                    float x = row[0];
                    float y = row[1];
                    float w = row[2];
                    float h = row[3];
                    
                    // Convert to image coordinates
                    int left = static_cast<int>((x - w/2) * frame.cols);
                    int top = static_cast<int>((y - h/2) * frame.rows);
                    int width = static_cast<int>(w * frame.cols);
                    int height = static_cast<int>(h * frame.rows);
                    
                    bboxes.emplace_back(left, top, width, height);
                    classes.push_back(class_names_[class_id.x]);
                    scores.push_back(conf);
                }
            }
        }
        
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("DeepDetector"), 
            "Detection error: %s", e.what());
    }
    
    return {bboxes, classes, scores};
}

} // namespace perception 