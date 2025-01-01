#pragma once

#include <array>
#include <memory>
#include <thread>

#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace csc379
{

class FrankaImpedanceControl
{
  public:
    FrankaImpedanceControl(std::shared_ptr<rclcpp::Node> node_handle);

  private:
    // Franka
    std::shared_ptr<franka::Model> model_;
    std::shared_ptr<franka::Robot> robot_;
    std::array<double, 7> k_gains_;
    std::array<double, 7> d_gains_;
    franka::Torques impedanceControlCallback(
        const franka::RobotState& state, franka::Duration /*period*/);

    // ROS
    std::mutex current_state_mtx_;
    std::vector<double> current_joint_positions_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
        js_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void publishState();

    std::mutex joint_positions_mtx_;
    std::vector<double> command_joint_positions_;
    std::shared_ptr<rclcpp::Node> node_handle_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>>
        js_subscription_;
    void setJointPositions(const sensor_msgs::msg::JointState& js_msg);

    // Add more objects when needed
};

} // namespace csc379
