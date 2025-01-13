
#include "franka_ros2_csc379/franka_impedance_control_ros2.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node_handle =
        std::make_shared<rclcpp::Node>("franka_impedence_control");
    auto spin_thread = std::thread([&]() { rclcpp::spin(node_handle); });
    auto franka_impedence_control_ros2 =
        csc379::FrankaImpedanceControlROS2(node_handle);
    spin_thread.join();
    rclcpp::shutdown();
    franka_impedence_control_ros2.Join();
    return 0;
}
