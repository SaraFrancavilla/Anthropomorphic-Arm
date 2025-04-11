#ifndef INVERSE_KIN_HPP
#define INVERSE_KIN_HPP

// libraries providede by ROS2
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Class implementing the inverse kinematics node
class Inverse_kin : public rclcpp::Node
{
public:
    Inverse_kin(); //Constructor

private:
    // Callback for handling end effector position messages
    void end_effector_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    // Subscription to the end effector position topic
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;

    // Publisher for the joint angles
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_angles_publisher_;
};

#endif
