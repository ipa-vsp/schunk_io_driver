#include <memory>

#include "schunk_bt_skills/gripper_command_action_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_name = "gripper_command";
  auto node_option = rclcpp::NodeOptions();
  auto node =
    std::make_shared<gripper_command_skill::GripperCommandActionServer>(action_name, node_option);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
