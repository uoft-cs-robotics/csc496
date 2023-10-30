/** 
 * Muhammad Arshad 
 * 26-July-2021
 * See license file
**/

#include <iostream>
#include "teleop/follower.h"
#include "ik.h"
#include "context.h"

namespace robotContext
{
    franka::Model *model;
    franka::Robot *robot;  
    franka::RobotState initial_state;
}


// franka::JointVelocities ik_control_loop( franka::RobotState _fstate, franka::RobotState _lstate, franka::Duration _period, bool is_state)
// {
//     // you can control state in this loop (if is_state is true) 
//     std::array<double, 7> current_position = {0, 0, 0, 0, 0, 0, 0};
//     std::array<double, 7> calculated_torque = {0, 0, 0, 0, 0, 0, 0};
//     if ( !is_state )
//     {
//         current_position = _fstate.q_d;
//     }
//     else
//     {
//         current_position = _lstate.q;
//     }


// }


franka::Torques control_loop( franka::RobotState _fstate, franka::RobotState _lstate, franka::Duration _period, bool is_state)
{
    
    // you can control state in this loop (if is_state is true) 
    std::array<double, 7> current_position = {0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> calculated_torque = {0, 0, 0, 0, 0, 0, 0};
    if ( !is_state )
    {
        current_position = _fstate.q_d;
    }
    else
    {
        current_position = _lstate.q;
    }

    std::vector<double> Kp = {30, 30, 30, 30, 15, 15, 15};
    std::vector<double> Kd = {0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.25};
    
    for (int i = 0; i < _fstate.q.size(); i++)
    {
        calculated_torque[i] = Kp[i] * ( current_position[i] - _fstate.q[i] )  + Kd[i] * _fstate.dq[i];
    }

    franka::Torques output  = {{calculated_torque[0], calculated_torque[1],
                                calculated_torque[2], calculated_torque[3] ,
                                calculated_torque[4] , calculated_torque[5],
                                calculated_torque[6] }};

    return output;
}


bool read_loop( franka::RobotState _fstate, franka::RobotState _lstate, franka::Duration _period, bool is_state)
{

    if (is_state)
    {
        std::cout << "Follower and leader states are available" << std::endl;
    }
    // you can read state in this loop (if is_state is true) 
    return true;
}


int main(int argc, char** argv) 
{
    if (argc != 3)
    {
        std::cerr << "Usage: "  << argv[0] << "  <server-port>  <follower-robot-hostname> \n";
        return 1;
    }

    franka::Robot robot_(argv[2]);
    robotContext::robot = &robot_; 
    franka::Model model_ = robotContext::robot->loadModel();
    robotContext::model = &model_; 


    teleop::Follower follower(argv[1], argv[2]) ;
    std::cout << "WARNING: The robot will go to ready pose! "
        << "Please make sure to have the user stop button at hand!" << std::endl
        << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robotContext::initial_state = follower.GoHome();

    std::cout << "WARNING: The robot will try to imitate leader robot! "
        << "Please make sure to have the user stop button at hand!" << std::endl
        << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    // follower.Read(read_loop);
    follower.Control(control_loop);

    std::cout << "Done" << std::endl;
    return 0;
}
