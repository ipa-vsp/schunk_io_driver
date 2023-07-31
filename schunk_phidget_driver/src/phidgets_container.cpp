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


#include "schunk_phidget_driver/phidgets_container.hpp"
#include "schunk_phidget_driver/phidgets_error.hpp"

#include <cstring>
#include <string>

void PhidgetsContainer::init()
{
    configure();

    if (!this->load_driver())
    {
        throw PhidgetsContainerException("Failed to load driver");
    }
}

void PhidgetsContainer::configure()
{
    if (!this->get_parameter("package", this->package_))
    {
        throw PhidgetsContainerException("Failed to load driver: package name is empty");
    }
    if (!this->get_parameter("driver", this->driver_))
    {
        throw PhidgetsContainerException("Failed to load driver: driver name is empty");
    }

    RCLCPP_INFO(this->get_logger(), "Starting phidget container");
    RCLCPP_INFO(this->get_logger(), "\t Package: %s", this->package_.c_str());
    RCLCPP_INFO(this->get_logger(), "\t Driver: %s", this->driver_.c_str());
}

bool PhidgetsContainer::load_driver()
{
    RCLCPP_INFO(this->get_logger(), "Loading driver");

    std::vector<rclcpp::Parameter> params;
    int serial, hub_port;
    bool is_hub_port_device;
    if (this->get_parameter("serial", serial))
        params.push_back(rclcpp::Parameter("serial", serial));
    if (this->get_parameter("hub_port", hub_port))
        params.push_back(rclcpp::Parameter("hub_port", hub_port));
    if (this->get_parameter("is_hub_port_device", is_hub_port_device))
        params.push_back(rclcpp::Parameter("is_hub_port_device", is_hub_port_device));

    if (!this->load_component(this->package_, this->driver_, params))
    {
        throw PhidgetsContainerException("Failed to load component");
        return false;
    }
    add_node_to_executor(this->digital_outputs_ros_i_->get_node_base_interface());
    return true;
}

bool PhidgetsContainer::load_component(std::string &package_name, std::string &driver_name,
                                       std::vector<rclcpp::Parameter> &params)
{
    RCLCPP_INFO(this->get_logger(), "Loading component: %s/%s", package_name.c_str(), driver_name.c_str());

    ComponentResource component_resource;
    std::string resource_index("rclcpp_components");
    std::string node_name = "phidgets_driver_node";
    std::vector<ComponentResource> components = this->get_component_resources(package_name, resource_index);

    for (auto it = components.begin(); it != components.end(); ++it)
    {
        if (it->first.compare(driver_name) == 0)
        {
            std::shared_ptr<rclcpp_components::NodeFactory> node_factory = this->create_component_factory(*it);
            rclcpp::NodeOptions options;

            options.use_global_arguments(false);
            options.use_intra_process_comms(true);
            std::vector<std::string> remap_rules;
            remap_rules.push_back("--ros-args");
            remap_rules.push_back("-r");
            remap_rules.push_back("__node:=" + node_name);
            options.arguments(remap_rules);
            options.parameter_overrides(params);

            try
            {
                auto wapper = node_factory->create_node_instance(options);
                this->digital_outputs_ros_i_ =
                    std::static_pointer_cast<phidgets::DigitalOutputsRosI>(wapper.get_node_instance());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to load component: %s", e.what());
                throw PhidgetsContainerException("Failed to load component: " + std::string(e.what()));
                return false;
            }
            return true;
        }
    }

    return false;
}
