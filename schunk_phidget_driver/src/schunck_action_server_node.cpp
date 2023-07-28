#include "schunk_phidget_driver/schunck_action_server.hpp"

namespace schunk_egp40
{
GripperActionServer::GripperActionServer(const rclcpp::NodeOptions &options)
    : Node("schunk_action_server_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Starting schunk_action_server_node");
    this->declare_parameter("gripper_open_io", 1);
    this->declare_parameter("gripper_close_io", 2);

    gripper_action_server_ = rclcpp_action::create_server<GripperCommand>(
        this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "/gripper_command",
        std::bind(&GripperActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&GripperActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&GripperActionServer::handle_accepted, this, std::placeholders::_1));

    digital_output_client_ = this->create_client<phidgets_msgs::srv::SetDigitalOutput>("set_digital_output");

    while (!digital_output_client_->wait_for_service(std::chrono::seconds(1)))
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
}

rclcpp_action::GoalResponse GripperActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                             std::shared_ptr<const GripperCommand::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal the gripper to open (true) close (false): %s",
                goal->is_open ? "true" : "false");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
GripperActionServer::handle_cancel(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperActionServer::handle_accepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    std::thread{std::bind(&GripperActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void GripperActionServer::execute(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    const auto target = goal->is_open;

    auto result = std::make_shared<schunk_command_interface::action::Egp40Command::Result>();
    auto feedback = std::make_shared<schunk_command_interface::action::Egp40Command::Feedback>();

    phidget_request_close_->state = false;
    phidget_request_open_->state = false;

    auto response_received_callback = [this](ServiceResponseFuture future)
    {
        auto result = future.get();
        RCLCPP_DEBUG(this->get_logger(), "Result of set_digital_output: %d", result->success);
    };

    digital_output_client_->async_send_request(phidget_request_close_, response_received_callback);
    digital_output_client_->async_send_request(phidget_request_open_, response_received_callback);

    rclcpp::sleep_for(std::chrono::milliseconds(15));

    if (!target)
    {
        phidget_request_close_->state = true;
        digital_output_client_->async_send_request(phidget_request_close_, response_received_callback);
        feedback->operation_state = "Closing gripper";
        goal_handle->publish_feedback(feedback);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Closing gripper succeeded");
    }
    else if (target)
    {
        phidget_request_open_->state = true;
        digital_output_client_->async_send_request(phidget_request_open_, response_received_callback);
        feedback->operation_state = "Opening gripper";
        goal_handle->publish_feedback(feedback);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Opening gripper succeeded");
    }
}
} // namespace schunk_egp40

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(schunk_egp40::GripperActionServer)
