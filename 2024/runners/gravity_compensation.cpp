// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "ik.h"
/**
 * @example echo_robot_state.cpp
 * An example showing how to continuously read the robot state.
 */


namespace robotContext {
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;

}

int main(int argc, char** argv) {
      std::vector<std::array< double, 16 >> ee_poses; 
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  InverseKinematics ik_controller(1, IKType::M_P_PSEUDO_INVERSE);


  try {
    franka::Robot robot(argv[1]);
    robotContext::robot = &robot;
    franka::Model model = robot.loadModel();
    robotContext::model = &model;


    std::cout << "Done." << std::endl;



    double time = 0.0;
    robot.control([](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::Torques {

      franka::Torques output_torques = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 

      return output_torques;
    });
  

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}


        // franka::Robot robot_(robot_ip);
        // franka::Gripper gripper_(robot_ip);

        // robotContext::robot = &robot_; 
        // robotContext::gripper = &gripper_; 
        // franka::Model model_ = robotContext::robot->loadModel();
        // robotContext::model = &model_;  