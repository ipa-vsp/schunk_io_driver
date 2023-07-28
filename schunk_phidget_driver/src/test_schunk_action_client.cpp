#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// #include "control_msgs/action/gripper_command.hpp"
#include "schunk_command_interface/action/egp40_command.hpp"
#include "schunk_phidget_driver/visibility_control.h"

class GripperActionClient : public rclcpp::Node
{
  public:
    using GripperCommand = schunk_command_interface::action::Egp40Command;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

    explicit GripperActionClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("gripper_action_client", options)
    {
        using namespace std::placeholders;

        this->client_ptr_ = rclcpp_action::create_client<GripperCommand>(this, "/gripper_command");

        this->timer_ =
            this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&GripperActionClient::send_goal, this));
    }

    void send_goal()
    {
        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = schunk_command_interface::action::Egp40Command::Goal();
        goal_msg.is_open = false;

        RCLCPP_INFO(this->get_logger(), "Sending goal close");
        auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&GripperActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&GripperActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&GripperActionClient::result_callback, this, std::placeholders::_1);

        auto future_handle = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        // Wait for the result
        // while(rclcpp::ok())
        // {
        //     if(future_handle.wait_for(std::chrono::seconds(1)) != std::future_status::ready)
        //     {
        //         RCLCPP_INFO(this->get_logger(), "Waiting for result");
        //     }
        //     else
        //     {
        //         break;
        //     }
        // }

        // auto future_state = goal_handle_future.wait_for(std::chrono::seconds(1));
        // if(future_state != std::future_status::ready)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        // }

        // // sleep for 5 seconds
        rclcpp::sleep_for(std::chrono::seconds(5));

        goal_msg.is_open = true;

        RCLCPP_INFO(this->get_logger(), "Sending goal open");
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

        // // future_state = goal_handle_future.wait_for(std::chrono::seconds(1));
        // // if(future_state != std::future_status::ready)
        // // {
        // //     RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        // // }

        rclcpp::sleep_for(std::chrono::seconds(5));
    }

  private:
    rclcpp_action::Client<GripperCommand>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void feedback_callback(GoalHandleGripperCommand::SharedPtr,
                           const std::shared_ptr<const GripperCommand::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback: %s", feedback->operation_state.c_str());
    }

    void goal_response_callback(const GoalHandleGripperCommand::SharedPtr &goal_handle)
    {
        using namespace std::placeholders;
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void result_callback(const GoalHandleGripperCommand::WrappedResult &result)
    {
        using namespace std::placeholders;
        switch (result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
        RCLCPP_INFO(this->get_logger(), "Result received");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<GripperActionClient>();

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
