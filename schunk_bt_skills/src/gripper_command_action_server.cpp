// Copyright 2023 Vishnuprasad Prachandabhanu
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "schunk_bt_skills/gripper_command_action_server.hpp"

namespace gripper_command_skill
{
GripperCommandActionServer::GripperCommandActionServer(const std::string action_name,
                                                       const rclcpp::NodeOptions &options)
    : ros2_skill_server_core::SkillActionServerLifecycleCore<control_msgs::action::GripperCommand>(
          action_name, action_name, options),
      action_name_(action_name)
{
    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(this->get_name()));
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

    RCLCPP_INFO(this->get_logger(), "Starting schunk_action_server_node");
    this->declare_parameter("gripper_open_io", 1);
    this->declare_parameter("gripper_close_io", 2);

    digital_output_client_ = this->create_client<phidgets_msgs::srv::SetDigitalOutput>("set_digital_output");

    while (!digital_output_client_->wait_for_service(std::chrono::seconds(10)) && rclcpp::ok())
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "set_digital_output service not available, waiting again...");
    }

    phidget_request_close_ = std::make_shared<phidgets_msgs::srv::SetDigitalOutput::Request>();
    phidget_request_close_->index = this->get_parameter("gripper_close_io").as_int();
    phidget_request_open_ = std::make_shared<phidgets_msgs::srv::SetDigitalOutput::Request>();
    phidget_request_open_->index = this->get_parameter("gripper_open_io").as_int();

    // open gripper as default
    digital_output_client_->async_send_request(phidget_request_open_);

    this->_pub_joints = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Joint States
    jstates_ = std::make_shared<sensor_msgs::msg::JointState>();
    jstates_->name.push_back("schunk_egp40_finger_left_joint");
    jstates_->name.push_back("schunk_egp40_finger_right_joint");
    // Default open
    jstates_->position.push_back(0.003);
    jstates_->position.push_back(-0.003);

    this->pub_timer = this->create_wall_timer(std::chrono::microseconds(500),
                                              std::bind(&GripperCommandActionServer::run_publisher, this));
}

nav2_util::CallbackReturn GripperCommandActionServer::on_configure(const rclcpp_lifecycle::State &state)
{
    auto node = shared_from_this();
    action_server_ =
        std::make_unique<ActionServer>(node, action_name_, std::bind(&GripperCommandActionServer::execution, this),
                                       nullptr, std::chrono::milliseconds(500), true);

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn GripperCommandActionServer::on_cleanup(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(*logger_, "Cleaning up...");
    action_server_.reset();
    // initial();
    return nav2_util::CallbackReturn::SUCCESS;
}

void GripperCommandActionServer::execution()
{
    std::scoped_lock<std::mutex> lock(this->lock_msgs_);
    const auto goal = this->getCurrentGoal();
    const auto target = goal->command.position;
    auto result = std::make_shared<ActionT::Result>();

    phidget_request_close_->state = false;
    phidget_request_open_->state = false;

    auto response_received_callback = [this](ServiceResponseFuture future)
    {
        future.wait();
        auto result = future.get();
        RCLCPP_DEBUG(this->get_logger(), "Result of set_digital_output: %d", result->success);
    };

    digital_output_client_->async_send_request(phidget_request_close_, response_received_callback);
    digital_output_client_->async_send_request(phidget_request_open_, response_received_callback);

    jstates_.reset(new sensor_msgs::msg::JointState);
    jstates_->name.push_back("schunk_egp40_finger_left_joint");
    jstates_->name.push_back("schunk_egp40_finger_right_joint");

    rclcpp::sleep_for(std::chrono::milliseconds(15)); // Requirement from the datasheet of Schunk EGP40

    if (target == 0)
    {

        phidget_request_close_->state = true;
        digital_output_client_->async_send_request(phidget_request_close_, response_received_callback);
        // Todo: Add feedback
        rclcpp::sleep_for(std::chrono::seconds(1)); // Wait for completly close the gripper
        result->position = 0.0;
        result->reached_goal = true;
        sendSucceededResult(result);
        RCLCPP_INFO(this->get_logger(), "Closing gripper succeeded");

        jstates_->position.push_back(-0.003);
        jstates_->position.push_back(0.003);
    }
    else if (target > 0)
    {
        phidget_request_open_->state = true;
        digital_output_client_->async_send_request(phidget_request_open_, response_received_callback);
        rclcpp::sleep_for(std::chrono::seconds(1)); // Wait for complete gripper close
        result->position = 1;
        result->reached_goal = true;
        sendSucceededResult(result);
        RCLCPP_INFO(this->get_logger(), "Opening gripper succeeded");

        jstates_->position.push_back(0.003);
        jstates_->position.push_back(-0.003);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Gripper position must be between 0 and 1");
        sendFaildResult(result);
    }
}

void GripperCommandActionServer::run_publisher()
{
    std::scoped_lock<std::mutex> lock(this->lock_msgs_);
    this->_pub_joints->publish(*jstates_);
}

} // namespace gripper_command_skill
