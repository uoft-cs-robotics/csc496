#include <chrono>
#include <functional>
#include <cstdlib>

#include "franka_ros2_csc379/franka_impedance_control_ros2.hpp"

namespace csc379
{

FrankaImpedanceControlROS2::FrankaImpedanceControlROS2(
    std::shared_ptr<rclcpp::Node> node_handle)
{
    // This should be correct from your previous task
    fic_ = std::make_unique<FrankaImpedanceControl>();

    node_handle_ = node_handle;

    // Task: Create a publisher

    auto timer_callback = [&, this]() {
        // Task: Get current joint positions from fic and publish
        this->publishState(); // Modify accordingly
    };
    timer_ = node_handle_->create_wall_timer(
        std::chrono::milliseconds(10), timer_callback);

    // Task: Create a subscriber with setJointPosition as the callback
}

void FrankaImpedanceControlROS2::publishState()
{
    // Task: Copy from your franka_state_publisher
}

void FrankaImpedanceControlROS2::setJointPositions(
    const sensor_msgs::msg::JointState& js_msg)
{
    // Task: Create a subscriber and set the joint positions,
}

void FrankaImpedanceControlROS2::Join()
{
    fic_->Join();
    return;
}

} // namespace csc379
