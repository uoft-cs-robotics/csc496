#include <chrono>
#include <functional>
#include <cstdlib>

#include "franka_ros2_csc379/franka_impedance_control_ros2.hpp"

namespace csc379
{

FrankaImpedanceControlROS2::FrankaImpedanceControlROS2(
    std::shared_ptr<rclcpp::Node> node_handle)
{
    fic_ = std::make_unique<FrankaImpedanceControl>();

    node_handle_ = node_handle;

    // Task: Create a publisher

    auto timer_callback = [&, this]() {
        std::vector<double> current_joint_positions;
        {
            std::lock_guard<std::mutex> lock(this->current_state_mtx_);
            current_joint_positions = current_joint_positions_;
        }
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
    // Task: Create a subscriber to set the joint positions,
}

void FrankaImpedanceControlROS2::join()
{
    fic_->join();
}

} // namespace csc379

