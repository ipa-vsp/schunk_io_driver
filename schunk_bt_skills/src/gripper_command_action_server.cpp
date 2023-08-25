#include "schunk_bt_skills/gripper_command_action_server.hpp"


namespace gripper_command_skill
{
    GripperCommandActionServer::GripperCommandActionServer(const std::string action_name, const rclcpp::NodeOptions & options) : ros2_skill_server_core::SkillActionServerLifecycleCore<
        schunk_command_interface::action::GripperCommand>(action_name, action_name, options), action_name_(action_name)
    {
        logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(this->get_name()));
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    }

    nav2_util::CallbackReturn GripperCommandActionServer::on_configure(const rclcpp_lifecycle::State & state)
    {
        auto node = shared_from_this();
        action_server_ = std::make_unique<ActionServer>(
            node, action_name_, std::bind(&GripperCommandActionServer::execution, this), nullptr,
            std::chrono::milliseconds(500), true);

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn GripperCommandActionServer::on_cleanup(const rclcpp_lifecycle::State & state)
    {
        RCLCPP_INFO(*logger_, "Cleaning up...");
        action_server_.reset();
        // initial();
        return nav2_util::CallbackReturn::SUCCESS;
    }

}