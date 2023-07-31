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

#ifndef PHIIDGETS_CONTAINER_HPP_
#define PHIIDGETS_CONTAINER_HPP_

#include <chrono>
#include <memory>
#include <vector>

#include "phidgets_digital_outputs/digital_outputs_ros_i.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executor.hpp>
#include <rclcpp_components/component_manager.hpp>

class PhidgetsContainer : public rclcpp_components::ComponentManager
{
  public:
    PhidgetsContainer(
        std::weak_ptr<rclcpp::Executor> executor = std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
        std::string node_name = "phidgets_container_node", const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : rclcpp_components::ComponentManager(executor, node_name, options)
    {
        executor_ = executor;
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        this->declare_parameter<std::string>("package", "");
        this->declare_parameter<std::string>("driver", "");
        this->declare_parameter<int>("serial", -1);
        this->declare_parameter<int>("hub_port", 0);
        this->declare_parameter<bool>("is_hub_port_device", false);

        this->loadNode_srv_.reset();
        this->unloadNode_srv_.reset();
    }

    void init();

    // virtual void on_list_nodes(const std::shared_ptr<rmw_request_id_t> request_header,
    //     const std::shared_ptr<ListNodes::Request> request,
    //     std::shared_ptr<ListNodes::Response> response) override;

  private:
    std::weak_ptr<rclcpp::Executor> executor_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    std::shared_ptr<phidgets::DigitalOutputsRosI> digital_outputs_ros_i_;

    std::string package_;
    std::string driver_;

    void configure();
    bool load_driver();
    bool load_component(std::string &package_name, std::string &driver_name, std::vector<rclcpp::Parameter> &params);

    void add_node_to_executor(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_inteface)
    {
        if (auto exec = executor_.lock())
        {
            RCLCPP_INFO(this->get_logger(), "Added %s to executor", node_inteface->get_fully_qualified_name());
            exec->add_node(node_inteface, true);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Add to executor failed: %s", node_inteface->get_fully_qualified_name());
        }
    }

    // std::map<uint16_t, std::string> list_components();
};

#endif /* PHIIDGETS_CONTAINER_HPP_ */
