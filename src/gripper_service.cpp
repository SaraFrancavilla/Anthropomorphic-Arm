#include "rclcpp/rclcpp.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_tolerance.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/asio.hpp>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

using boost::asio::ip::tcp;

class GripperController : public rclcpp::Node
{
public:
    GripperController(const std::string &host)
        : Node("gripper_service"),
          host_(host), port_(30002),
          start_config_({0.0, 0.0}),
          time_between_points_(0.1),
          current_state_("neutral")
    {
        // Dynamically find script paths
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("ros2_ur5_interface");
        close_script_ = package_share_dir + "/gripper/close.script";
        open_script_ = package_share_dir + "/gripper/open.script";
        neutral_from_open_script_ = package_share_dir + "/gripper/neutral_from_open.script";
        neutral_from_closed_script_ = package_share_dir + "/gripper/neutral_from_closed.script";

        // Create services
        open_service_ = this->create_service<std_srvs::srv::Trigger>(
            "open_gripper", std::bind(&GripperController::open, this, std::placeholders::_1, std::placeholders::_2));
        close_service_ = this->create_service<std_srvs::srv::Trigger>(
            "close_gripper", std::bind(&GripperController::close, this, std::placeholders::_1, std::placeholders::_2));
        neutral_service_ = this->create_service<std_srvs::srv::Trigger>(
            "neutral_gripper", std::bind(&GripperController::neutral, this, std::placeholders::_1, std::placeholders::_2));

        // Create a client for the Trigger service
        client_ = this->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");

        // Subscribe to the joint states topic
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&GripperController::joint_state_callback, this, std::placeholders::_1));

        // Initialize action client
        action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this, "/gripper_controller/follow_joint_trajectory");

        RCLCPP_INFO(this->get_logger(), "Gripper controller node initialized.");

        // Send the gripper to the neutral position at startup
        send_script(neutral_from_open_script_);
        send_script(neutral_from_closed_script_);
        send_action({0.0, 0.0});
    }

private:
    std::string host_;
    int port_;
    std::string close_script_, open_script_, neutral_from_closed_script_, neutral_from_open_script_;
    std::vector<double> start_config_;
    double time_between_points_;
    std::string current_state_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr neutral_service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        //Without taking into account object dimension
        // if (msg->position.size() > 7) // Ensure valid indices
        // {
        //     start_config_ = {msg->position[5], msg->position[7]};
        // }

        //Stopping when resistance is detected
        if (msg->position.size() > 7) // Ensure valid indices
        {
            auto new_config = std::vector<double>{msg->position[5], msg->position[7]};
            if (new_config == start_config_) // No change in position
            {
                RCLCPP_INFO(this->get_logger(), "Gripper motion stopped due to resistance.");
                // Optionally cancel the action here
            }
            start_config_ = new_config;
        }
    }

    void send_action(const std::vector<double> &end_config)
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting. Ignore if using real robot.");
            return;
        }

        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.header.stamp = rclcpp::Time(0);
        traj_msg.joint_names = {"soft_robotics_gripper_left_finger_joint1", "soft_robotics_gripper_right_finger_joint1"};

        double T = 3 * time_between_points_;

        for (int i = 0; i <= 3; i++)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            double t = i * time_between_points_;
            for (size_t j = 0; j < start_config_.size(); j++)
            {
                double interpolated_position = start_config_[j] + (t / T) * (end_config[j] - start_config_[j]);
                point.positions.push_back(interpolated_position);
            }
            point.time_from_start = rclcpp::Duration::from_seconds(t);
            traj_msg.points.push_back(point);
        }

        auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
        goal_msg.trajectory = traj_msg;
        goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(0.5);

        RCLCPP_INFO(this->get_logger(), "Sending gripper goal.");

        auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void send_script(const std::string &script_path)
    {
        try
        {
            boost::asio::io_context io_context;
            tcp::socket socket(io_context);

            tcp::resolver resolver(io_context);
            boost::asio::connect(socket, resolver.resolve(host_, std::to_string(port_)));

            std::ifstream scriptFile(script_path, std::ios::binary);
            if (!scriptFile.is_open())
            {
                throw std::runtime_error("Failed to open script file: " + script_path);
            }

            char buffer[2024];
            while (scriptFile.read(buffer, sizeof(buffer)) || scriptFile.gcount() > 0)
            {
                boost::asio::write(socket, boost::asio::buffer(buffer, scriptFile.gcount()));
            }

            RCLCPP_INFO(this->get_logger(), "Script sent successfully!");
            socket.close();

            // Create a request object
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

            // Call the service asynchronously
            rclcpp::sleep_for(std::chrono::seconds(1));
            auto future = client_->async_send_request(request);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error sending script: %s", e.what());
        }
    }

    void open(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (current_state_ == "open")
        {
            RCLCPP_INFO(this->get_logger(), "Gripper is already open. Ignoring request.");
            response->success = false;
            response->message = "Gripper is already in the open position.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Request to open the gripper received.");
        send_script(open_script_);
        send_action({0.3, 0.3});
        current_state_ = "open";
        response->success = true;
    }

    void close(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (current_state_ == "close")
        {
            RCLCPP_INFO(this->get_logger(), "Gripper is already closed. Ignoring request.");
            response->success = false;
            response->message = "Gripper is already in the closed position.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Request to close the gripper received.");
        send_script(close_script_);
        send_action({-0.3, -0.3});
        current_state_ = "close";
        response->success = true;
    }

    void neutral(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (current_state_ == "neutral")
        {
            RCLCPP_INFO(this->get_logger(), "Gripper is already in the neutral position. Ignoring request.");
            response->success = false;
            response->message = "Gripper is already in the neutral position.";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Request to move the gripper to neutral position received.");
        if (current_state_ == "open")
            send_script(neutral_from_open_script_);
        else if (current_state_ == "close")
            send_script(neutral_from_closed_script_);
        else
        send_action({0.0, 0.0});
        current_state_ = "neutral";
        response->success = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        std::cerr << "Usage: ros2 run ros2_ur5_interface gripper_service <robot_ip>" << std::endl;
        return 1;
    }

    std::string host = argv[1];

    auto node = std::make_shared<GripperController>(host);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
