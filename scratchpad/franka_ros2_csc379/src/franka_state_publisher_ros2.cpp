#include <memory>
#include "franka_ros2_csc379/franka_state_publisher_ros2.hpp"

namespace csc379
{

FrankaStatePublisherROS2::FrankaStatePublisherROS2(
    std::shared_ptr<rclcpp::Node> node_handle)
{
    node_handle_ = node_handle;
    // Task: Create necessary Franka objects, follow any libfranka example

    // Task: Create a publisher
}

void FrankaStatePublisherROS2::ReadStateAndPublish()
{
    // Task: Read Franka State Once, do not use franka::control method

    this->publishState();
}

void FrankaStatePublisherROS2::publishState()
{
    // Task: Create sensor_msgs JointState and Publish
}

} // namespace csc379

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node_handle =
        std::make_shared<rclcpp::Node>("franka_state_publisher");
    csc379::FrankaStatePublisherROS2 franka_state_publisher =
        csc379::FrankaStatePublisherROS2(node_handle);
    auto spin_thread = std::thread([&]() { rclcpp::spin(node_handle); });

    while (rclcpp::ok())
    {
        franka_state_publisher.ReadStateAndPublish();
        // Task: Add rate of publishing
    }

    rclcpp::shutdown();
    spin_thread.join(); // rclcpp::shutdown stops rclcpp::spin and thread
    return 0;
}
