#include "schunk_phidget_driver/phidgets_container.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto container = std::make_shared<PhidgetsContainer>(exec);
    std::thread contInit([&container]() { container->init(); });
    exec->add_node(container);
    exec->spin();
    contInit.join();
    rclcpp::shutdown();
    return 0;
}
