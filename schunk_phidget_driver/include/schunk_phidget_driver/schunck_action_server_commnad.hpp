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

#ifndef SCHUNK_ACTION_SERVER_COMMAND_HPP_
#define SCHUNK_ACTION_SERVER_COMMAND_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "control_msgs/action/gripper_command.hpp"
// #include "schunk_command_interface/action/egp40_command.hpp"
#include "phidgets_msgs/srv/set_digital_output.hpp"

#include "schunk_phidget_driver/visibility_control.h"

namespace schunk_egp40
{
class GripperActionServer : public rclcpp::Node
{
  public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;
    using ServiceResponseFuture = rclcpp::Client<phidgets_msgs::srv::SetDigitalOutput>::SharedFuture;

    SCHUNK_EGP40_CONTROL__VISIBILITY_PUBLIC
    explicit GripperActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    rclcpp_action::Server<GripperCommand>::SharedPtr gripper_action_server_;
    rclcpp::Client<phidgets_msgs::srv::SetDigitalOutput>::SharedPtr digital_output_client_;
    std::shared_ptr<phidgets_msgs::srv::SetDigitalOutput::Request> phidget_request_close_;
    std::shared_ptr<phidgets_msgs::srv::SetDigitalOutput::Request> phidget_request_open_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_joints;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const GripperCommand::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);

    void execute(const std::shared_ptr<GoalHandleGripperCommand> goal_handle);
};
} // namespace schunk_egp40

#endif // SCHUNK_ACTION_DRIVER_HPP_
