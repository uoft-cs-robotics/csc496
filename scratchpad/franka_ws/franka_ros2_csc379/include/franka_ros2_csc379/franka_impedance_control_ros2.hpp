#pragma once

#include <memory>
#include <thread>

#include "franka_impedance_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace csc379
{

class FrankaImpedanceControlROS2
{
  public:
    FrankaImpedanceControlROS2(std::shared_ptr<rclcpp::Node> node_handle);
    void Join();

  private:
    // Franka
    std::unique_ptr<FrankaImpedanceControl> fic_;

    // ROS
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
        js_publisher_;
    std::thread publisher_thread_;
    void publishState();

    std::shared_ptr<rclcpp::Node> node_handle_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>>
        js_subscription_;
    void setJointPositions(const sensor_msgs::msg::JointState& js_msg);
};

} // namespace csc379
