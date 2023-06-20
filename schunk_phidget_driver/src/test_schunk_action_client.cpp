#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "control_msgs/action/gripper_command.hpp"
#include "schunk_phidget_driver/visibility_control.h"

class GripperActionClient : public rclcpp::Node
{
    public:
        using GripperCommand = control_msgs::action::GripperCommand;
        using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

        explicit GripperActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("gripper_action_client", options)
        {
            using namespace std::placeholders;

            this->client_ptr_ = rclcpp_action::create_client<GripperCommand>(this, "/gripper_command");

            this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&GripperActionClient::send_goal, this));
        }

        void send_goal()
        {
            if(!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5)))
            {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = control_msgs::action::GripperCommand::Goal();
            goal_msg.command.position = 0.0;
            
            RCLCPP_INFO(this->get_logger(), "Sending goal close");
            auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&GripperActionClient::goal_response_callback, this, _1);
            send_goal_options.result_callback = std::bind(&GripperActionClient::result_callback, this, _1);

            auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
            
            auto future_state = goal_handle_future.wait_for(std::chrono::seconds(1));
            if(future_state != std::future_status::ready)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            }

            // sleep for 5 seconds
            rclcpp::sleep_for(std::chrono::seconds(5));

            goal_msg.command.position = 1.0;

            RCLCPP_INFO(this->get_logger(), "Sending goal open");
            goal_handle_future = this->client_ptr_->async_send_goal(goal_msg); // , send_goal_options);
            
            future_state = goal_handle_future.wait_for(std::chrono::seconds(1));
            if(future_state != std::future_status::ready)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            }

            rclcpp::sleep_for(std::chrono::seconds(5));

        }
    
    private:
        rclcpp_action::Client<GripperCommand>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;


};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperActionClient>());
    rclcpp::shutdown();
    return 0;
}