#ifndef GRIPPER_COMMAND_ACTION_SERVER_HPP_
#define GRIPPER_COMMAND_ACTION_SERVER_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "man2_skill_server_core/skill_action_server_lifecycle_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "schunk_command_interface/action/gripper_command.hpp"

namespace gripper_command_skill
{
class GripperCommandActionServer
    : public ros2_skill_server_core::SkillActionServerLifecycleCore<schunk_command_interface::action::GripperCommand>
{
  public:
    using ActionT = schunk_command_interface::action::GripperCommand;
    explicit GripperCommandActionServer(const std::string action_name,
                                        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~GripperCommandActionServer() override = default;

    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

    void execution() override;

  private:
    std::string action_name_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
};
} // namespace gripper_command_skill

#endif // GRIPPER_COMMAND_ACTION_SERVER_HPP_
