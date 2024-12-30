#pragma once

#include <array>
#include <memory>
#include <vector>

#include <sensor_msgs/JointState.h>
#include <franka/model.h>


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

namespace csc379
{
    
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}



// class JointImpedenceController
// {
// public:
//   struct Config
//   {
//     std::array<double, 7> k_gains = {
//         400.0, 400.0, 400.0, 400.0, 200.0, 100.0, 50.0};
//     std::array<double, 7> d_gains = {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0};
//     size_t filter_size = 5;
//   };

//   JointImpedenceController(const Config& config);
//   franka::Torques impedenceControl(
//       const franka::RobotState& robot_state, franka::Duration period);
//   BoolReturn
//   setCommandJointPosition(const std::array<double, 7>& joint_positions);
//   BoolReturn
//   setCommandJointPosition(const std::vector<double>& joint_positions);

// private:
//   void updateDQFilter(const franka::RobotState& state);
//   double getDQFiltered(size_t index) const;
//   size_t dq_current_filter_position_;
//   size_t dq_filter_size_;
//   std::unique_ptr<double[]> dq_buffer_;

//   template <typename T>
//   BoolReturn setCommandJointPositionImpl(const T& joint_positions)
//   {
//     static double tolerance = 0.05; // radian
//     std::lock_guard<std::mutex> lock(mtx);
//     if (joint_positions.size() > command_joint_pos.size())
//     {
//       return BoolReturn(false, error_code_t::JOINT_POSITION_SIZE_NOT_THE_SAME);
//     }
//     for (unsigned int i = 0; i < joint_positions.size(); ++i)
//     {
//       if (std::abs(joint_positions[i] - command_joint_pos[i]) > tolerance)
//       {
//         ROS_ERROR(
//             "Commanded js for joint %i, larger than tolerance: %f",
//             i,
//             tolerance);
//         ROS_ERROR("Desired position: %f", joint_positions[i]);
//         ROS_ERROR("Current position: %f", command_joint_pos[i]);
//         return BoolReturn(
//             false,
//             error_code_t::JOINT_POSITION_COMMAND_TOO_FAR_FROM_CURRENT_POSITION);
//       }
//     }
//     for (unsigned int i = 0; i < joint_positions.size(); ++i)
//     {
//       command_joint_pos[i] = joint_positions[i];
//     }
//     return BoolReturn(true);
//   }
//   std::mutex mtx;
//   std::vector<double> command_joint_pos;
//   std::shared_ptr<franka::Model> f_model;
//   const std::array<double, 7> k_gains_;
//   const std::array<double, 7> d_gains_;
// };

// class JointPositionController : public Controller
// {
// public:
//   JointPositionController(
//       ros::NodeHandle& nh,
//       const std::string& topic_name,
//       const std::string& topic_namespace = "medrct_franka",
//       const JointImpedenceController::Config& joint_config =
//           JointImpedenceController::Config());
//   virtual ~JointPositionController() {}
//   virtual franka::Torques control(
//       const franka::RobotState& robot_state, franka::Duration period) override;

// private:
//   ros::Subscriber js_sub;
//   void jointCommandCallback(const sensor_msgs::JointState& msg);
//   std::unique_ptr<JointImpedenceController> jic;
// };
} // namespace csc379
