#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <thread>

#include <franka/model.h>
#include <franka/robot.h>

namespace csc379
{

class FrankaImpedanceControl
{
  public:
    FrankaImpedanceControl();
    void Join();
    std::array<double, 7> GetCurrentJointPositions();
    void SetCommandJointPositions(const std::array<double, 7>& joint_positions);
  private:
    // Franka
    std::shared_ptr<franka::Model> model_;
    std::shared_ptr<franka::Robot> robot_;
    std::array<double, 7> k_gains_;
    std::array<double, 7> d_gains_;
    franka::Torques impedanceControlCallback(
        const franka::RobotState& state, franka::Duration /*period*/);
    std::thread control_thread_;

    std::mutex current_state_mtx_;
    std::array<double, 7> current_joint_positions_;

    std::mutex joint_positions_mtx_;
    std::array<double, 7> command_joint_positions_;
    // Add more objects when needed
};

} // namespace csc379
