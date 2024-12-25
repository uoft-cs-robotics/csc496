/** 
 * Muhammad Arshad 
 * 26-July-2021
 * See license file
**/


#include "teleop/leader.h" 
#include "teleop/logger.h"
// #include <franka/model.h>

namespace teleop
{

bool is_started_receiving = false; 
int lost_packets = 0; 
Leader::Leader(char* server_port, char* server_ip, char* leader_ip)
    : client(io_service, server_port, server_ip)
    //   teleop::robot(leader_ip)
{
    // franka::Model model_ = teleop::robot->loadModel();
    // model = &model_; // note we are using leaders kinematics model method to compute follower's gravity torque
    InitializeRobot();
}

Leader::~Leader()
{
    client.~NetworkClient(); 
}


void Leader::InitializeRobot()
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

void Leader::GoHome()
{
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    robotContext::robot->control(motion_generator);
}

void Leader::Control( std::function<franka::Torques( 
    const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
    franka::Duration period,  bool _is_leader_state_received )> custom_control_loop)
{
    const double print_rate = 100.0;
    std::atomic_bool running{true};
    std::string file_name = "/home/lueder/Desktop/log_leader.txt";
    // std::thread print_thread = std::thread(logger::log_data, std::cref(print_rate), std::ref(logger::print_data), std::ref(running), std::ref(file_name));
    robotContext::robot->control(
        [this, &custom_control_loop]( const franka::RobotState& robot_state, franka::Duration period )
        -> franka::Torques
        {
              
            // send state the other end 
            client.DoSend(robot_state);
            _master_state = robot_state;
            _slave_state = client._slave_state;
            is_state_received = client.is_state_received;
            // if (logger::print_data.mutex.try_lock()) {
            //     logger::print_data.has_data = is_state_received;
            //     logger::print_data.robot_state = robot_state;
            //     logger::print_data.mutex.unlock();
            // }         
            if (!is_started_receiving) {
                if(is_state_received)
                    is_started_receiving = true; 
            } 
            else {
                if (!is_state_received)
                    ++lost_packets;
                else    
                    lost_packets = 0;  
            }      
            franka::Torques _torques = custom_control_loop(_slave_state, _master_state, period, is_state_received);
            client.is_state_received = false;
            if(is_started_receiving && lost_packets > 20) 
                return MotionFinished( _torques);
            return _torques;
        }
    );
    return ;    
}

void Leader::Read( std::function<bool( 
    const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
    franka::Duration period,  bool _is_leader_state_received )> read_loop)
{
    robotContext::robot->read(
        [this, &read_loop]( const franka::RobotState& robot_state)
        {
            // send state the other end 
            client.DoSend(robot_state);
            _master_state = robot_state;
            _slave_state = client._slave_state;
            is_state_received = client.is_state_received;
            franka::Duration _duration(0);

            // call user callback
            bool read_state = read_loop(_slave_state, _master_state, _duration, is_state_received);
            return read_state;
        }
    );
}

}