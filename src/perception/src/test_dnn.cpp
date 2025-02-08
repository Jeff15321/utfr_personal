#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>

int main() {
    // 1. Try to load model
    std::string model_path = "/home/jeff1216/utfr_dv/src/perception/perception/models/yolov7-e6e.onnx";
    try {
        std::cout << "Attempting to load model: " << model_path << std::endl;
        
        // Load YOLO model
        cv::dnn::Net net = cv::dnn::readNetFromONNX(model_path);
        
        if (net.empty()) {
            std::cerr << "Failed to load model" << std::endl;
            return -1;
        }
        
        std::cout << "Successfully loaded model!" << std::endl;
        
        // 2. Try to load a test image
        cv::Mat img = cv::imread("src/perception/test/test_image.jpg");
        if (img.empty()) {
            std::cerr << "Failed to load test image" << std::endl;
            return -1;
        }
        
        // 3. Preprocess image
        cv::Mat blob = cv::dnn::blobFromImage(img, 1.0/255.0, 
            cv::Size(640, 640), cv::Scalar(), true, false);
            
        // 4. Run inference
        net.setInput(blob);
        cv::Mat output = net.forward();
        
        std::cout << "Inference successful!" << std::endl;
        std::cout << "Output shape: " << output.size << std::endl;
        
        return 0;
        
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV error: " << e.what() << std::endl;
        return -1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
} 