#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>


#include "ur5_globals.hpp"

//creating a server that passes the final positions to the inverse kin
Inverse_kin::Inverse_kin(): Node("inverse_kin")
{
    // Create a subscription to the end effector position topic
    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "end_effector_position", 10, std::bind(&Inverse_kin::end_effector_callback, this, std::placeholders::_1));

    // Create a publisher for the joint angles
    joint_angles_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_angles", 10);
}


void Inverse_kin::end_effector_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    // Log the received end effector position
    RCLCPP_INFO(this->get_logger(), "Received end effector position: x=%.2f, y=%.2f, z=%.2f",
                msg->position.x, msg->position.y, msg->position.z);

    // Define a set of predefined joint angles
    std_msgs::msg::Float64MultiArray joint_angles_msg;
    joint_angles_msg.data = {-1.41, -0.96, -1.8, -1.96, -1.60, 0.0}; // Example angles

    // Publish the joint angles
    joint_angles_publisher_->publish(joint_angles_msg);

    RCLCPP_INFO(this->get_logger(), "Published predefined joint angles.");
}


//convert quaternion into rotation matrix

//compute joint angles from rotation matrix

//check for unreachable positions

//send closes joint state