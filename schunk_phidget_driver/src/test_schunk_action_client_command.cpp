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


#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "control_msgs/action/gripper_command.hpp"
// #include "schunk_command_interface/action/egp40_command.hpp"
#include "schunk_phidget_driver/visibility_control.h"

class GripperActionClient : public rclcpp::Node
{
  public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

    explicit GripperActionClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("gripper_action_client", options)
    {
        using namespace std::placeholders;

        // auto client_cbg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // auto timer_cbg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        this->client_ptr_ = rclcpp_action::create_client<GripperCommand>(this, "/gripper_command");

        this->timer_ =
            this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&GripperActionClient::send_goal, this));
    }

    void send_goal()
    {
        this->timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Starting sending goal");
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = control_msgs::action::GripperCommand::Goal();
        goal_msg.command.position = 0.0;

        RCLCPP_INFO(this->get_logger(), "Sending goal close");
        auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&GripperActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&GripperActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&GripperActionClient::result_callback, this, std::placeholders::_1);

        auto future_handle = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

        // std::future<std::shared_ptr<GoalHandleGripperCommand>> result_fut = std::async(std::launch::async, [&](){
        //     auto res = future_handle.get();
        //     RCLCPP_INFO(this->get_logger(), "Result status: %d", res->get_status());
        //     return res;
        // });
        std::thread(
            [&]()
            {
                auto res = future_handle.get();
                RCLCPP_INFO(this->get_logger(), "Result status: %d", res->get_status());
            })
            .detach();
    }

  private:
    rclcpp_action::Client<GripperCommand>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void feedback_callback(GoalHandleGripperCommand::SharedPtr,
                           const std::shared_ptr<const GripperCommand::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback: %f", feedback->position);
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
