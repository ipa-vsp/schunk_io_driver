# Copyright 2023 Vishnuprasad Prachandabhanu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "launch"))  # noqa

import launch
import launch.actions
import launch.events

import launch_ros
import launch_ros.events
import launch_ros.events.lifecycle
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.actions import DeclareLaunchArgument
import lifecycle_msgs.msg


def generate_launch_description():

    phidget_driver_package = DeclareLaunchArgument(
        "phidget_driver_package",
        default_value=TextSubstitution(text="phidgets_digital_outputs"),
        description="Path to the bus configuration to use.",
    )

    phidget_driver_executable = DeclareLaunchArgument(
        "phidget_driver_executable",
        default_value=TextSubstitution(text="phidgets::DigitalOutputsRosI"),
        description="Executable to launch.",
    )

    ld = launch.LaunchDescription()

    device_container_node = launch_ros.actions.LifecycleNode(
        name="schunk_phidget_driver_node",
        namespace="",
        package="schunk_phidget_driver",
        output="screen",
        executable="phidgets_container_node",
        parameters=[
            {"package": LaunchConfiguration("phidget_driver_package")},
            {"driver": LaunchConfiguration("phidget_driver_executable")},
        ],
    )

    device_action_server_node = launch_ros.actions.Node(
        package="schunk_phidget_driver",
        executable="gripper_action_server_command_node",
        output="screen",
        parameters=[{"gripper_close_io": 1}, {"gripper_open_io": 2}],
    )

    ld.add_action(phidget_driver_package)
    ld.add_action(phidget_driver_executable)
    ld.add_action(device_container_node)
    ld.add_action(device_action_server_node)

    return ld
