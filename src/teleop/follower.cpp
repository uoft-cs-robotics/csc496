/** 
 * Muhammad Arshad 
 * 26-July-2021
 * See license file
**/

#include "teleop/follower.h" 
#include "teleop/logger.h"


namespace teleop
{

Follower::Follower(char* server_port, char* follower_ip)
    : server(io_service, std::atoi(server_port))
    //   robot(follower_ip)
{
    InitializeRobot();
}

Follower::~Follower()
{

}


void Follower::InitializeRobot()
{

    try 
    {
        setDefaultBehavior(*robotContext::robot);
        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        robotContext::robot->setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    } 
    catch (franka::Exception const& e) 
    {
        std::cout << e.what() << std::endl;
    }
}


franka::RobotState Follower::GoHome()
{
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    robotContext::robot->control(motion_generator);
    franka::RobotState current_robot_state = robotContext::robot->readOnce();
    return current_robot_state;
}

void Follower::Control( std::function<franka::Torques( 
    const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
    franka::Duration period,  bool _is_leader_state_received )> control_loop)
{

    const double print_rate = 100.0;
    std::atomic_bool running{true};
    std::string file_name = "/home/log_follower.txt";
    std::thread print_thread = std::thread(logger::log_data, std::cref(print_rate), std::ref(logger::print_data), std::ref(running), std::ref(file_name));

    robotContext::robot->control(
        [this, &control_loop]( const franka::RobotState& robot_state, franka::Duration period )
        -> franka::Torques
        {
                    
            // send state the other end 
            server.DoSend(robot_state);
            _slave_state = robot_state;
            _master_state = server._master_state;
            is_state_received = server.is_state_received;

            if (logger::print_data.mutex.try_lock() && is_state_received) {
                logger::print_data.has_data = true;
                logger::print_data.robot_state = robot_state;
                logger::print_data.mutex.unlock();
            }            
            // call user callback
            franka::Torques _torques = control_loop(_slave_state, _master_state, period, is_state_received);

            franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            return _torques;
        }
    );

    if (print_thread.joinable()) {
    print_thread.join();
    }  

}

void Follower::ScaledMotionControl(  std::function<franka::JointVelocities( 
            const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
            franka::Duration period,  bool _is_leader_state_received )> ik_control_loop)
{

    const double print_rate = 100.0;
    std::atomic_bool running{true};
    std::string file_name = "/home/log_follower.txt";
    std::thread print_thread = std::thread(logger::log_data, std::cref(print_rate), std::ref(logger::print_data), std::ref(running), std::ref(file_name));
    robotContext::robot->control(
        [this, &ik_control_loop]( const franka::RobotState& robot_state, franka::Duration period )
        -> franka::JointVelocities
        {
            // send state the other end 
            server.DoSend(robot_state);
            _slave_state = robot_state;
            _master_state = server._master_state;
            is_state_received = server.is_state_received;

            if (logger::print_data.mutex.try_lock() && is_state_received) {
                logger::print_data.has_data = true;
                logger::print_data.robot_state = robot_state;
                logger::print_data.mutex.unlock();
            }            
            // call user callback
            franka::JointVelocities _vels = ik_control_loop(_slave_state, _master_state, period, is_state_received);
            return _vels;
        }

    );
    if (print_thread.joinable()) {
    print_thread.join();
    }  
    
}            




void Follower::Read( std::function<bool( 
    const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
    franka::Duration period,  bool _is_leader_state_received )> read_loop)
{
    robotContext::robot->read(
        [this, &read_loop]( const franka::RobotState& robot_state)
        {
            // send state the other end 
            server.DoSend(robot_state);
            _slave_state = robot_state;
            _master_state = server._master_state;
            franka::Duration _duration(0);
            is_state_received = server.is_state_received;
            // call user callback
            bool read_state = read_loop(_slave_state, _master_state, _duration, is_state_received);
            return read_state;
        }
    );
}

}