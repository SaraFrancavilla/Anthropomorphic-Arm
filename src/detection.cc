#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <torch/torch.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <memory>
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
                std::bind(&DetectionNode::image_callback, this) //check if camera driver is published on this topic. double-check topic name via "ro2 topic list"
            );

            publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("/inference_result", 8);
            // publisher = this->create_publisher<vision_msgs::msg::Detection2DArray>("/inference_result", 8);
            RCLCPP_INFO(this->get_logger(), "Detection Node initialised.");
        }

    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
            cv::Mat img;
            try{
                img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8)->image; // "rgb8" also might have to change encoding to bgr 
            }catch (cv_bridge::Exception &e){
                RCLCPP_ERROR(this->get_logger(), "Error converting image using cv_bridge: %s", e.what());
                return;
            }
            cv::Mat resized_img;
            try{
                cv::resize(img, resized_img, cv::Size(640, 640));
                resize_img.convertTo(resized_img, CV_32F, 1.0 / 255.0); // Normalize to [0, 1]
            }catch (const std::exception &e){
                RCLCPP_ERROR(this->get_logger(), "Error resizing image: %s", e.what());
                return;
            }

            auto input_tensor = torch::from_blob(resized_img.data, {1, resized_img.rows, resized_img.cols, 3}, torch::kFloat32)
                .permute({0, 3, 1, 2})
                .to(torch::kCPU);
            auto output = model.forward({input_tensor}).toTensor(); //or to Tuple if the model outputs multiple tensors
            RCLCPP_INFO(this->get_logger(), "Output from model :%s", output.sizes().vec().data());
            auto num_detections = output.size(0);
            if (num_detections == 0) {
                RCLCPP_INFO(this->get_logger(), "No detections found.");
                return;
            }
            std_msgs::msg::Float32MultiArray result_msg;
            result_msg.data.resize(num_detections * 6);
            auto detections = output.view({num_detections, -1}); // Flatten the output tensor

            for (int i = 0; i < num_detections; ++i) {
                auto detection = detections[i];
                result_msg.data[i * 6 + 0] = detection[0].item<float>();
                result_msg.data[i * 6 + 1] = detection[1].item<float>();
                result_msg.data[i * 6 + 2] = detection[2].item<float>();
                result_msg.data[i * 6 + 3] = detection[3].item<float>();
                result_msg.data[i * 6 + 4] = detection[4].item<float>();
                result_msg.data[i * 6 + 5] = detection[5].item<float>();
            }

            publisher->publish(result_msg);
        }

        static constexpr const char* model_path = "best_models/yolo11n_0dropout.pt"; // change to path of model
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
        torch::jit::script::Module model;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;
        // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher; // to publish the image with detections
        // torch::Device device = torch::Device(torch::kCPU); // Use CPU for inference
};
