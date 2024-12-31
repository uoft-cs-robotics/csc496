#pragma once

#include <memory>

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

    std::shared_ptr<rclcpp::Node> nodeHandle;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>>
        JSPublisher_;

    // Add more objects when needed
};

} // namespace csc379
