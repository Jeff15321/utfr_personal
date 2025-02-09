#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
#include <fstream>

void test_onnx_model(const std::string& model_path) {
    std::cout << "Testing ONNX model: " << model_path << std::endl;
    
    try {
        // Check if file exists
        std::ifstream f(model_path.c_str());
        if (!f.good()) {
            std::cerr << "Model file not found: " << model_path << std::endl;
            return;
        }
        
        // Load ONNX model
        cv::dnn::Net net = cv::dnn::readNetFromONNX(model_path);
        
        if (net.empty()) {
            std::cerr << "Failed to load model" << std::endl;
            return;
        }
        
        std::cout << "Successfully loaded model!" << std::endl;
        
        // Print model info
        std::vector<std::string> layer_names = net.getLayerNames();
        std::cout << "Model has " << layer_names.size() << " layers" << std::endl;
        
        // Print output layer names
        std::vector<std::string> outNames = net.getUnconnectedOutLayersNames();
        std::cout << "Output layers:" << std::endl;
        for (const auto& name : outNames) {
            std::cout << "  " << name << std::endl;
        }
        
        // Set backend
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        
        // Load test image
        cv::Mat img = cv::imread("src/perception/test/test_image.jpg");
        if (img.empty()) {
            std::cerr << "Failed to load test image" << std::endl;
            return;
        }
        
        // YOLOv7 preprocessing
        cv::Mat blob = cv::dnn::blobFromImage(img, 1.0/255.0, 
            cv::Size(640, 640), cv::Scalar(0,0,0), true, false);
            
        // Run inference
        net.setInput(blob);
        std::vector<cv::Mat> outputs;
        net.forward(outputs, outNames);
        
        std::cout << "Inference successful!" << std::endl;
        std::cout << "Number of outputs: " << outputs.size() << std::endl;
        for (size_t i = 0; i < outputs.size(); ++i) {
            std::cout << "Output " << i << " shape: " << outputs[i].size << std::endl;
            // Print first few values to verify output
            float* data = (float*)outputs[i].data;
            std::cout << "First few values: ";
            for (int j = 0; j < std::min(5, outputs[i].cols); ++j) {
                std::cout << data[j] << " ";
            }
            std::cout << std::endl;
        }
        
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV error: " << e.what() << std::endl;
    }
}

int main() {
    // Test the existing YOLOv7 model
    test_onnx_model("/home/jeff1216/utfr_dv/src/perception/perception/models/yolov7-e6e.onnx");
    return 0;
} 