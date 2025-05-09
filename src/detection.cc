#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <torch/script.h>
#include <torch/torch.h>

#include <memory>
#include <string>
#include <vector>
#include <exception>

class DetectionNode : public rclcpp::Node {
public:
    DetectionNode() : Node("yolo_detection_node") {
        try {
            model = torch::jit::load(model_path);
            model.to(torch::kCPU);
            model.eval();
        } catch (const c10::Error &e) {
            RCLCPP_ERROR(this->get_logger(), "Error loading the YOLO model: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        subscription = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw",
            8,
            std::bind(&DetectionNode::image_callback, this, std::placeholders::_1)
        );

        publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("/inference_result", 8);
        RCLCPP_INFO(this->get_logger(), "Detection Node initialized.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat img;
        try {
            img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image;
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
            return;
        }

        cv::Mat resized_img;
        try {
            cv::resize(img, resized_img, cv::Size(640, 640));
            resized_img.convertTo(resized_img, CV_32F, 1.0 / 255.0); // Normalize to [0, 1]
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Image preprocessing failed: %s", e.what());
            return;
        }

        auto input_tensor = torch::from_blob(resized_img.data, {1, resized_img.rows, resized_img.cols, 3}, torch::kFloat32)
                                .permute({0, 3, 1, 2})
                                .to(torch::kCPU);

        torch::Tensor output;
        try {
            auto output_tuple = model.forward({input_tensor});
            output = output_tuple.toTensor();  // Adjust if model returns a tuple
        } catch (const c10::Error &e) {
            RCLCPP_ERROR(this->get_logger(), "Model inference failed: %s", e.what());
            return;
        }

        if (output.numel() == 0) {
            RCLCPP_INFO(this->get_logger(), "No detections found.");
            return;
        }

        auto detections = output.view({-1, 6}); // Adjust shape based on model output
        int num_detections = detections.size(0);

        std_msgs::msg::Float32MultiArray result_msg;
        result_msg.data.resize(num_detections * 6);

        for (int i = 0; i < num_detections; ++i) {
            for (int j = 0; j < 6; ++j) {
                result_msg.data[i * 6 + j] = detections[i][j].item<float>();
            }
        }

        publisher->publish(result_msg);
        RCLCPP_INFO(this->get_logger(), "Published %d detections.", num_detections);
    }

    static constexpr const char* model_path = "best_models/yolo11n_0dropout.pt";
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;
    torch::jit::script::Module model;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionNode>());
    rclcpp::shutdown();
    return 0;
}