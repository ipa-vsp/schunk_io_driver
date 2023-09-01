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

#ifndef GRIPPER_COMMAND_ACTION_SERVER_HPP_
#define GRIPPER_COMMAND_ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "control_msgs/action/gripper_command.hpp"
#include "man2_skill_server_core/skill_action_server_lifecycle_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "phidgets_msgs/srv/set_digital_output.hpp"

namespace gripper_command_skill
{
class GripperCommandActionServer
    : public ros2_skill_server_core::SkillActionServerLifecycleCore<control_msgs::action::GripperCommand>
{
  public:
    using ActionT = control_msgs::action::GripperCommand;
    using ServiceResponseFuture = rclcpp::Client<phidgets_msgs::srv::SetDigitalOutput>::SharedFuture;
    explicit GripperCommandActionServer(const std::string action_name,
                                        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~GripperCommandActionServer() override = default;

    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

    void execution() override;

    void run_publisher();

  private:
    std::string action_name_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Client<phidgets_msgs::srv::SetDigitalOutput>::SharedPtr digital_output_client_;
    std::shared_ptr<phidgets_msgs::srv::SetDigitalOutput::Request> phidget_request_close_;
    std::shared_ptr<phidgets_msgs::srv::SetDigitalOutput::Request> phidget_request_open_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub_joints;

    std::shared_ptr<sensor_msgs::msg::JointState> jstates_;
    rclcpp::TimerBase::SharedPtr pub_timer;
    std::mutex lock_msgs_;
};
} // namespace gripper_command_skill

#endif // GRIPPER_COMMAND_ACTION_SERVER_HPP_
