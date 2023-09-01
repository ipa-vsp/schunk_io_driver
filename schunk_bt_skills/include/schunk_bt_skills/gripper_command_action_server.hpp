#ifndef GRIPPER_COMMAND_ACTION_SERVER_HPP_
#define GRIPPER_COMMAND_ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "man2_skill_server_core/skill_action_server_lifecycle_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"
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
