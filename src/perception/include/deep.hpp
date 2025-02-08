#ifndef DEEP_HPP_
#define DEEP_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>
#include <vector>
#include <tuple>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <fstream>

namespace perception {

class DeepDetector {
public:
    DeepDetector(const std::string& model_path, float conf_threshold = 0.5);
    ~DeepDetector() = default;
    
    // Main detection function
    std::tuple<std::vector<cv::Rect>, std::vector<std::string>, std::vector<float>> 
    detect(const cv::Mat& frame);

private:
    void loadModel(const std::string& model_path);
    
    float conf_threshold_;
    bool use_cuda_;
    cv::dnn::Net net_;
    std::vector<std::string> class_names_;
    const cv::Size input_size_ = cv::Size(640, 640);
    const float scale_ = 1.0f/255.0f;
    std::mutex detector_mutex_;
};

} // namespace perception 

#endif // DEEP_HPP_ 