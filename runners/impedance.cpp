// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <math.h> 
#include <eigen3/Eigen/Dense>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>
#include "ik.h"
#include "common.h"
/**
 * @example echo_robot_state.cpp
 * An example showing how to continuously read the robot state.
 */


namespace robotContext {
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;

}

    
namespace global_variable{
  std::array< double, 16 > current_ee_pose; 
  bool flag_done_collecting; 
  std::vector<std::array< double, 16 >> collected_ee_poses;
}

std::vector<std::array< double, 16 >> record_pose_thread(int n_poses=3){
  int collected_poses = 0; 
  std::string my_string = "";
  
  while(collected_poses < n_poses){
    std::cout << "Press ENTER to collect current pose, anything else to quit data collection" << std::endl;
    std::getline(std::cin, my_string);
    if(my_string.length()==0){
      global_variable::collected_ee_poses.push_back(global_variable::current_ee_pose);
      collected_poses++;
    }
    else{
      std::cout << "Exiting data collection"<<std::endl; 
      global_variable::flag_done_collecting = true; 
      break;
    }
  }
  global_variable::flag_done_collecting = true; 


}

double distance(const Eigen::Vector3d &v1,const Eigen::Vector3d &v2){
  Eigen::Vector3d diff = v1 - v2;
  return diff.norm();
}


int main(int argc, char** argv) {
  global_variable::flag_done_collecting = false; 

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

    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    robotContext::robot->control(motion_generator);

    int choice{}; 

    std::thread t1(record_pose_thread, 3);
    try{
      robot.control([](const franka::RobotState& robot_state,
                      franka::Duration period) -> franka::Torques {
        
        std::array<double, 7> coriolis = robotContext::model->coriolis(robot_state);
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        // Stiffness
        const std::array<double, 7> k_gains = {{15, 15, 15, 15, 5, 5, 5}};
        // Damping    
        std::array<double, 7> tau_d_calculated;        
        for (size_t i = 0; i < 7; i++) {
          double d_gain = 2 * sqrt(k_gains[i]);
          tau_d_calculated[i] =
              k_gains[i] * (q_goal[i] - robot_state.q[i]) - d_gain * robot_state.dq[i] + coriolis[i];
        }
        franka::Torques tau_d_rate_limited = franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, robot_state.tau_J_d);

        global_variable::current_ee_pose =  robot_state.O_T_EE;
        if (global_variable::flag_done_collecting)
          return franka::MotionFinished(tau_d_rate_limited);  
        else
          return tau_d_rate_limited;
      });
    }catch (franka::Exception const& e) {
      std::cout << e.what() << std::endl;
      return -1;
    }
    t1.join();

  std::cout << "Done collecting" << std::endl;

  // for (auto &ee_pose: ee_poses){
  for (auto &ee_pose: global_variable::collected_ee_poses){
    Eigen::Matrix4d pose = Eigen::Matrix4d::Map(ee_pose.data());
    std::cout<<pose<<"\n"; 
    double time = 0.0;
    robot.control([&time, &pose, &ik_controller](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();
      franka::JointVelocities output_velocities = ik_controller(robot_state, period, pose);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> output_eigen_velocities(robot_state.dq.data());
      Eigen::Vector3d current_position(robot_state.O_T_EE[12],robot_state.O_T_EE[13],robot_state.O_T_EE[14]); 
      Eigen::Vector3d desired_position(pose(0,3), pose(1,3), pose(2,3)) ;
      double dist = distance(current_position, desired_position);

      if (time >= 8.0 || (output_eigen_velocities.norm() < 0.002 && dist < 0.001) ) {
        output_velocities = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
        return franka::MotionFinished(output_velocities);
      }
      
      return output_velocities;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

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


/*

    // std::cout<<choice<<"\n";
    while(true){
    std::cout<<"enter your choice";
    std::cin >> choice; 

    if(choice==1)
    {
      std::cout<<"got choice\n";
      size_t count = 0;
      franka::RobotState robot_state = robot.readOnce();
      ee_poses.push_back(robot_state.O_T_EE);
    }
    else
    break; 

    }
*/
    // std::cout<<pose(0,3)<<"\n";
    // std::cout<<pose(1,3)<<"\n";
    // std::cout<<pose(2,3)<<"\n";    
    // std::cout<<pose.block<3,4>(0,0).col(3)<<"\n"; 
    // Eigen::Quaterniond q(pose.block<3,3>(0,0));
    // std::cout<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w()<<"\n";
    // std::array <double, 7> ee_desired_pose = {pose(0,3),pose(1,3),pose(2,3),q.x(),q.y(),q.z(),q.w()};//.head(3) = pose.block<3,3>(0,0);
    // ee_desired_pose.head(3) = pose.block<3,4>(0,0).col(3);