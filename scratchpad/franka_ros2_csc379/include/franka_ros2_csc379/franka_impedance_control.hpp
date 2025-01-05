#pragma once

#include <array>
#include <memory>
#include <vector>

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
    FrankaImpedanceControl();

  private:
    // Franka
    std::shared_ptr<franka::Model> model_;
    std::shared_ptr<franka::Robot> robot_;
    std::array<double, 7> k_gains_;
    std::array<double, 7> d_gains_;
    franka::Torques impedanceControlCallback(
        const franka::RobotState& state, franka::Duration /*period*/);

    std::mutex current_state_mtx_;
    std::vector<double> current_joint_positions_;
    std::vector<double> GetCurrentJointPositions();

    std::mutex joint_positions_mtx_;
    std::vector<double> command_joint_positions_;
    void FrankaImpedanceControl::SetCommandJointPositions(
        const std::vector<double>& joint_positions);
    // Add more objects when needed
};

} // namespace csc379
