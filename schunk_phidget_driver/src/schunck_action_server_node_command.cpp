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

#include "schunk_phidget_driver/schunck_action_server_commnad.hpp"

namespace schunk_egp40
{
GripperActionServer::GripperActionServer(const rclcpp::NodeOptions &options)
    : Node("schunk_action_server_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting schunk_action_server_node");
    this->declare_parameter("gripper_open_io", 1);
    this->declare_parameter("gripper_close_io", 2);

    auto cbg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto timer_cbg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    gripper_action_server_ = rclcpp_action::create_server<GripperCommand>(
        this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "/gripper_command",
        std::bind(&GripperActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&GripperActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&GripperActionServer::handle_accepted, this, std::placeholders::_1));

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

    this->pub_timer = this->create_wall_timer(std::chrono::microseconds(500), std::bind(&GripperActionServer::run_publisher, this));
}

rclcpp_action::GoalResponse GripperActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                             std::shared_ptr<const GripperCommand::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal to the gripper to move positin: %f", goal->command.position);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
GripperActionServer::handle_cancel(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received gripper request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperActionServer::handle_accepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Gripper goal accepted");
    std::thread{std::bind(&GripperActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void GripperActionServer::execute(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
    std::scoped_lock<std::mutex> lock(this->lock_msgs_);
    const auto goal = goal_handle->get_goal();
    const auto target = goal->command.position;

    auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
    auto feedback = std::make_shared<control_msgs::action::GripperCommand::Feedback>();

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
        goal_handle->succeed(result);
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
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Opening gripper succeeded");

        jstates_->position.push_back(0.003);
        jstates_->position.push_back(-0.003);
    }
}

void GripperActionServer::run_publisher()
{
    std::scoped_lock<std::mutex> lock(this->lock_msgs_);
    this->_pub_joints->publish(*jstates_);
}
} // namespace schunk_egp40

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(schunk_egp40::GripperActionServer)
