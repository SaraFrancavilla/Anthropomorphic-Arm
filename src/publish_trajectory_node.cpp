#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_tolerance.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class TrajectoryActionClient : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    TrajectoryActionClient() : Node("trajectory_action_client")
    {
        node_executed_ = false;

        // Define joint names
        joint_names_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");

        // Subscribe to the joint states topic
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&TrajectoryActionClient::joint_state_callback, this, std::placeholders::_1));

        open_gripper_client_ = this->create_client<std_srvs::srv::Trigger>("open_gripper");
        close_gripper_client_ = this->create_client<std_srvs::srv::Trigger>("close_gripper");

        if (!action_client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        if (!open_gripper_client_->wait_for_service(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "Gripper service not available after waiting");
            rclcpp::shutdown();
            return;
        }

        time_between_points_ = 0.5; // Time between points in seconds
    }

private:
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr open_gripper_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr close_gripper_client_;
    double time_between_points_;
    std::vector<trajectory_msgs::msg::JointTrajectory> trajectories_;
    size_t current_trajectory_index_;
    std::vector<std::string> joint_names_;
    std::vector<double> start_config_;
    bool node_executed_;


    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // check if nan
        for (size_t i = 0; i < msg->position.size(); i++)
        {
            if (std::isnan(msg->position[i]))
            {
                RCLCPP_WARN(this->get_logger(), "Joint state message contains NaN values");
                return;
            }
        }
        if (msg->position.size() > 5 && !node_executed_) // Ensure valid indices
        {
            RCLCPP_INFO(this->get_logger(), "Executing node");
            node_executed_ = true;

            // cycle through the positions in msg and find the corresponding joint names, then store the values in start_config_ with the same order as joint_names_
            for (size_t i = 0; i < joint_names_.size(); i++)
            {
                for (size_t j = 0; j < msg->name.size(); j++)
                {
                    if (msg->name[j] == joint_names_[i])
                    {
                        start_config_.push_back(msg->position[j]);
                        break;
                    }
                }
            }

            RCLCPP_INFO(this->get_logger(), "Start config: %f %f %f %f %f %f", start_config_[0], start_config_[1], start_config_[2], start_config_[3], start_config_[4], start_config_[5]);

            // Call the gripper service based on the trajectory index
            RCLCPP_INFO(this->get_logger(), "Opening the gripper");
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            open_gripper_client_->async_send_request(request);
            std::this_thread::sleep_for(2s);
            RCLCPP_INFO(this->get_logger(), "Gripper opened");

            // Prepare the sequence of trajectories
            RCLCPP_INFO(this->get_logger(), "Preparing trajectories");
            prepare_trajectories();
            RCLCPP_INFO(this->get_logger(), "Trajectories prepared");

            // Start executing the first trajectory
            current_trajectory_index_ = 0;
            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution");
            send_next_trajectory();
        }
    }

    trajectory_msgs::msg::JointTrajectory generate_trajectory_segment(
        const std::vector<double>& start_config,
        const std::vector<double>& end_config,
        int num_points)
    {
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = joint_names_;

        // Total interpolation time (N points * time_between_points_)
        double total_time = (num_points * time_between_points_) == 0 ? 1 : num_points * time_between_points_;

        for (int i = 0; i <= num_points; i++)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            double t = i * time_between_points_;

            for (size_t j = 0; j < start_config.size(); j++)
            {
                double interpolated_position = start_config[j] + (t / total_time) * (end_config[j] - start_config[j]);
                point.positions.push_back(interpolated_position);
            }
            RCLCPP_INFO(this->get_logger(), "Trajectory point %d: %f %f %f %f %f %f", i, point.positions[0], point.positions[1], point.positions[2], point.positions[3], point.positions[4], point.positions[5]);

            point.time_from_start = rclcpp::Duration::from_seconds(t);
            traj_msg.points.push_back(point);
        }

        return traj_msg;
    }


    void prepare_trajectories()
    {
       //Modify this function to publish a position on the topic end_effector_position
        //In the generate_trajectory_segment read and write position from topic joint_angles

        // Subscribe to the topic joint_angles
        auto joint_angles_subscription = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_angles", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Received joint angles");
                // Save the received joint angles in a variable
                if (msg->position.size() == joint_names_.size()) {
                    angle_config = msg->position;
                } else {
                    RCLCPP_WARN(this->get_logger(), "Received joint angles size mismatch");
                }
            });

        // Publish to the topic end_effector_position a random position
        auto end_effector_position_publisher = this->create_publisher<sensor_msgs::msg::JointState>("end_effector_position", 10);
        sensor_msgs::msg::JointState random_position_msg;
        random_position_msg.name = joint_names_;
        random_position_msg.position = {0.5, -0.5, 0.3, -0.3, 0.2, -0.2}; // Example random values
        end_effector_position_publisher->publish(random_position_msg);
        RCLCPP_INFO(this->get_logger(), "Published random end effector position");

        //Trajectory 1
        trajectory_msgs::msg::JointTrajectory traj1;
        traj1 = generate_trajectory_segment(
            start_config_,
            angle_config,
            10);
        trajectories_.push_back(traj1);



        // Define the trajectories
        // Trajectory 1
        // trajectory_msgs::msg::JointTrajectory traj1;
        // traj1 = generate_trajectory_segment(
        //     start_config_,
        //     {-1.41, -0.96, -1.8, -1.96, -1.60, 0.0},
        //     10);
        // trajectories_.push_back(traj1);

        // Trajectory 2
        trajectory_msgs::msg::JointTrajectory traj2;
        traj2 = generate_trajectory_segment(
            {-1.41, -0.96, -1.8, -1.96, -1.60, 0.0},
            start_config_,
            10);
        trajectories_.push_back(traj2);

        // Add additional trajectories as needed
    }

    void send_next_trajectory()
    {
        if (current_trajectory_index_ >= trajectories_.size())
        {
            RCLCPP_INFO(this->get_logger(), "All trajectories executed successfully");
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory = trajectories_[current_trajectory_index_];
        goal_msg.goal_time_tolerance.nanosec = 500000000;

        RCLCPP_INFO(this->get_logger(), "Sending trajectory goal %zu", current_trajectory_index_ + 1);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle) {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result");
                }
            };

        send_goal_options.result_callback =
            [this](const GoalHandleFollowJointTrajectory::WrappedResult &result) {
                switch (result.code)
                {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Goal %zu succeeded", current_trajectory_index_ + 1);
                    handle_trajectory_success();
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    break;
                }
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void handle_trajectory_success()
    {
        // Call the gripper service based on the trajectory index
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = close_gripper_client_->async_send_request(request);
        std::this_thread::sleep_for(2s);

        if (future.wait_for(5s) == std::future_status::ready)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Gripper service call succeeded: %s", response->message.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Gripper service call failed: %s", response->message.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Gripper service call timed out");
        }

        // Proceed to the next trajectory
        current_trajectory_index_++;
        send_next_trajectory();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryActionClient>());
    rclcpp::shutdown();
    return 0;
}
