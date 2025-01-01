#pragma once

#include <memory>

#include <franka/robot.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace csc379
{

class FrankaStatePublisher
{
  public:
    FrankaStatePublisher(std::shared_ptr<rclcpp::Node> node_handle);
    void ReadStateAndPublish();

  private:
    void publishState();

    std::shared_ptr<rclcpp::Node> node_handle_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
        js_publisher_;

    // Franka
    std::shared_ptr<franka::Robot> robot_;

    // Add more objects when needed
};

} // namespace csc379
