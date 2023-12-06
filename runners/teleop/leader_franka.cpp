/** 
 * Muhammad Arshad 
 * 26-July-2021
 * See license file
**/

#include <iostream>
#include "teleop/leader.h"
#include "context.h"

namespace robotContext
{
    franka::Model *model;
    franka::Robot *robot;  
    bool is_started_receiving = false;     
}



franka::Torques control_loop_no_feedback( franka::RobotState _fstate, franka::RobotState _lstate, franka::Duration _period, bool is_state)
{
    
    // you can control the leader in this loop (if is_state is true) 

    franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    return zero_torques;
}


franka::Torques control_loop_joint_position_coupling( franka::RobotState _fstate, franka::RobotState _lstate, franka::Duration _period, bool is_state)
{
    std::array<double, 7> follower_joint_positions = {0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> calculated_torque = {0, 0, 0, 0, 0, 0, 0};  

    std::vector<double> Kp = {120.0, 120.0, 120.0, 120.0, 20.0, 20.0, 4.0};
    std::vector<double> Kd = {7.5, 7.5, 7.5, 7.5, 2.25, 2.25, 0.75}; 

    // std::vector<double> Kp = {30, 30, 30, 30, 15, 15, 15};
    // std::vector<double> Kd = {0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.25}; 

    if (!is_state){
        follower_joint_positions = _lstate.q_d;
    }
    else{
        follower_joint_positions = _fstate.q;
        robotContext::is_started_receiving = true;
    }

    for (int i = 0; i < _lstate.q.size(); i++)
    {
        calculated_torque[i] = Kp[i] * (follower_joint_positions[i] - _lstate.q[i] )  + Kd[i] * _lstate.dq[i];
    }

    // you can control the leader in this loop (if is_state is true) 

    
    franka::Torques output_torque{{calculated_torque[0], calculated_torque[1],
                                calculated_torque[2], calculated_torque[3] ,
                                calculated_torque[4] , calculated_torque[5],
                                calculated_torque[6] }};
    if(robotContext::is_started_receiving && !is_state)
        return MotionFinished(output_torque);
    else
        return output_torque;
}

franka::Torques control_loop_joint_torque_coupling( franka::RobotState _fstate, franka::RobotState _lstate, franka::Duration _period, bool is_state)
{


    std::array<double, 7> calculated_torque = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  
    std::vector<double> K_tau_scaling = {0.9, 0.9, 0.45, 0.9, 0.45, 0.9, 0.9};    
    if (!is_state){
        ;   
    }
    else{

        robotContext::is_started_receiving = true;
        // std::array<double, 7> follower_gravity_tau = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
        std::array<double, 7> follower_gravity_tau = (*robotContext::model).gravity(_fstate);

        // try{
        // }
        // catch (franka::Exception const& e) 
        // {
        //     std::cout << e.what() << std::endl;
        // }

        std::array<double, 7> follower_measured_tau = _fstate.tau_J;
        for (int i = 0; i < 7; i++)
        {
            calculated_torque[i] = -1.0 * K_tau_scaling[i] * (follower_measured_tau[i] - follower_gravity_tau[i]);
            // std::cout<<calculated_torque[i]<<", "<<K_tau_scaling[i]<<", "<<follower_measured_tau[i]<<", "<<follower_gravity_tau[i]<<"\n";
        }        
    }
    franka::Torques output_torque{{calculated_torque[0], calculated_torque[1],
                                calculated_torque[2], calculated_torque[3] ,
                                calculated_torque[4] , calculated_torque[5],
                                calculated_torque[6] }};
    
    if(robotContext::is_started_receiving && !is_state)
        return MotionFinished(output_torque);
    else
        return output_torque;   
}

bool read_loop( franka::RobotState _fstate, franka::RobotState _lstate, franka::Duration _period, bool is_state)
{
    
    if (is_state)
    {
        std::cout << "Follower states are available" << std::endl;
    }
    // you can read state in this loop (if is_state is true) 
    return true;
}


int main(int argc, char** argv) 
{
    if (argc != 4)
    {
        std::cerr << "Usage: "  << argv[0] << "  <server-port>  <server-pc-host> <leader-robot-hostname> \n";
        return 1;
    }

    franka::Robot robot_(argv[3]);
    robotContext::robot = &robot_; 
    franka::Model model_ = robotContext::robot->loadModel();
    robotContext::model = &model_; 

    teleop::Leader leader(argv[1], argv[2], argv[3]) ;
    std::cout << "WARNING: The robot will go to ready pose! "
        << "Please make sure to have the user stop button at hand!" << std::endl
        << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    leader.GoHome();

    std::cout << "WARNING: The robot will go in zero torque mode (zero gravity)! "
        << "Please make sure to have the user stop button at hand!" << std::endl
        << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    // leader.Read(read_loop);

    // leader.Control( control_loop_no_feedback);
    // leader.Control(control_loop_joint_position_coupling);
    leader.Control(control_loop_joint_torque_coupling);

    std::cout << "Done" << std::endl;
    return 0;
}
