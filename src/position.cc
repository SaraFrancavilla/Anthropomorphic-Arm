#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <memory>
#include <cmath>
#include <string>
#include <vector>
#include <iterator>

namespace std {
    template <>
    struct iterator_traits<sensor_msgs::PointCloud2ConstIterator<float>> {
        using difference_type = std::ptrdiff_t;
        using value_type = float;
        using pointer = float*;
        using reference = float&;
        using iterator_category = std::forward_iterator_tag;  // Assuming it's a forward iterator
    };
}

class CameraPoseNode : public rclcpp::Node{
    //("check if pixel coordinates match between the color image and point cloud");
    public:
        CameraPoseNode(): Node("pose_from_camera_node"){
            subscription_pixel = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/inference_result", rclcpp::QoS(8), std::bind(&CameraPoseNode::image_callback, this, std::placeholders::_1));
            subscription_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/camera/depth/points", rclcpp::QoS(8), std::bind(&CameraPoseNode::cloud_callback, this, std::placeholders::_1));
            publisher = this->create_publisher<vision_msgs::msg::Detection3DArray>("/inference_3d", 8);
            current_cloud = nullptr;
        }

    private:
        void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
            current_cloud = msg;
        }
        void image_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
            if(current_cloud == nullptr){
                RCLCPP_WARN(this->get_logger(), "No point cloud data available as yet. Waiting for point cloud data :)");
                return;
            }
            vision_msgs::msg::Detection3DArray publish_positions;
            publish_positions.header = current_cloud->header;
            auto positions = msg->data;
            if(positions.size() == 0){
                RCLCPP_WARN(this->get_logger(), "No positions data available as yet. Waiting for positions data :)");
                return;
            }
            for(size_t i =0; i+5 < positions.size(); i+=6){
                float id = positions[i];
                float confidence = positions[i+1];
                float x = positions[i+2];
                float y = positions[i+3];

                int width = current_cloud->width;
                int height = current_cloud->height;

                int u = static_cast<int>(x * width);
                int v = static_cast<int>(y * height);

                if(u < 0 || u >= width || v < 0 || v >= height){
                    RCLCPP_WARN(this->get_logger(), "Invalid pixel coordinates: (%d, %d)", u, v);
                    continue;
                }
                float index_1d = v * width + u;

                sensor_msgs::PointCloud2ConstIterator<float> iter_x(*current_cloud, "x");
                sensor_msgs::PointCloud2ConstIterator<float> iter_y(*current_cloud, "y");
                sensor_msgs::PointCloud2ConstIterator<float> iter_z(*current_cloud, "z");

                std::advance(iter_x, index_1d);
                std::advance(iter_y, index_1d);
                std::advance(iter_z, index_1d);

                x = *iter_x;
                y = *iter_y;
                float z = *iter_z;

                if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)){
                    RCLCPP_WARN(this->get_logger(), "Invalid point cloud data at index_1d: %f", index_1d);
                    continue;
                }
                vision_msgs::msg::Detection3D detect;

                geometry_msgs::msg::Pose pose;
                pose.position.x = x;
                pose.position.y = y;
                pose.position.z = z;
                pose.orientation.w = 1.0;
    
                vision_msgs::msg::ObjectHypothesisWithPose object_hypothesis;
                object_hypothesis.hypothesis.class_id = std::to_string(static_cast<int>(id));
                object_hypothesis.hypothesis.score = confidence;
                object_hypothesis.pose.pose = pose;
    
                detect.results.push_back(object_hypothesis);
                detect.bbox.center = pose;
    
                publish_positions.detections.push_back(detect);
                RCLCPP_INFO(this->get_logger(), "Pose ID: %f, Confidence: %f, Position: (%f, %f, %f)", id, confidence, x, y, z);
            }
            publisher->publish(publish_positions);

        }
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_cloud;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_pixel;
        rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr publisher;
        sensor_msgs::msg::PointCloud2::SharedPtr current_cloud;
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPoseNode>());
    rclcpp::shutdown();
    return 0;
}
