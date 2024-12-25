/** 
 * UDP Server -> Slave Robot
 * UDP Client -> Master Robot
 * 
 * 
 * Muhammad Arshad 
 * 07-July-2021
**/

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdlib.h>   


// #include <Eigen/Dense>
#include <eigen3/Eigen/Dense>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/duration.h>
#include <franka/model.h>
#include "teleop/examples_common.h"

#include "teleop/messages.h"


#include <boost/asio.hpp>
#include <boost/thread.hpp>

using boost::asio::ip::udp;

enum { max_length = 8192 };

class server 
{
public:

    server(boost::asio::io_service& io_service, short port, char* slave_ip)
       : socket_( io_service, udp::endpoint(udp::v4(), port))
    {
        memset( receive_data_, 0, sizeof(receive_data_) );
        std::cout << "Tis is a UDP server" << std::endl;
        do_receive();

        // single thread
        // boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
        
        // multiple threads
        for (unsigned i = 0; i < boost::thread::hardware_concurrency(); ++i)
            tg.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
        
        // if running without an actual robot
        // test_loop();

        // if running with a robt
        initialize_robot(slave_ip);
    }

    ~server()
    {
        tg.join_all();
    }

    void test_loop()
    {
        while (true)
        {
            // franka::RobotState fake_state;

            // fake_state.q = {fake_joint_values, fake_joint_values, fake_joint_values, fake_joint_values, fake_joint_values, fake_joint_values, fake_joint_values};
            // fake_joint_values += 0.1;
            // if (fake_joint_values > 1) fake_joint_values = 0.1;
            
            // _slave_state = fake_state;

            // std::cout << "Joints: " ;
            // for ( int i = 0; i < _slave_state.q.size(); i++)
            // {
            //     std::cout << _master_state.q.at(i) - _slave_state.q.at(i) << "  " ;
            // }
            // std::cout << std::endl;

            // do_send( fake_state );

            // boost::this_thread::sleep( boost::posix_time::milliseconds(10) );
        }
    }

    bool initialize_robot(char* slave_ip)
    {
        try 
        {
            franka::Robot robot(slave_ip);
            // setup_state_read_loop(robot);
            setDefaultBehavior(robot);
            setup_initial_pose(robot);
            setup_position_control(robot);
        } 
        catch (franka::Exception const& e) 
        {
            std::cout << e.what() << std::endl;
            return false;
        }
    }

    void setup_state_read_loop(franka::Robot& robot)
    {
        robot.read(  [this] (const franka::RobotState& robot_state) 
        {   
            _slave_state = robot_state;
            do_send(_slave_state);
            
            if (is_master_state_received)
            {
                double Kp = 0.1;
                std::array<double, 7> desired ;
                for ( int i = 0; i < _master_state.q.size(); i++)
                {
                    desired[i] = Kp * ( _master_state.q[i] - _slave_state.q[i] ) ;
                }

                print_array(desired, "After Kp: " );
            }

            return true;                
        });
    }

    void setup_initial_pose(franka::Robot& robot)
    {
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        std::cout << "WARNING: The robot will move to a pre-configured pose! "
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        robot.control(motion_generator);
        std::cout << "Finished moving to initial joint configuration." << std::endl;   
    }

    void setup_position_control(franka::Robot& robot)
    {
        // Set additional parameters always before the control loop, NEVER in the control loop!
        // Set collision behavior.
        robot.setCollisionBehavior(
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
            {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        std::array<double, 7> initial_position = {0, 0, 0, 0, 0, 0, 0};
        std::array<double, 7> current_position = {0, 0, 0, 0, 0, 0, 0};
        double time = 0.0;

        std::cout << "WARNING: The robot will try to imitate master! "
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();

        robot.control([this, &current_position, &time](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques 
        {
            _slave_state = robot_state;
            do_send(_slave_state);
            
            if ( !is_master_state_received )
            {
                current_position = robot_state.q_d;
            }
            else
            {
                current_position = _master_state.q;
                // is_master_state_received = false;
            }

            std::vector<double> Kp = {30, 30, 30, 30, 15, 15, 15};
            std::vector<double> Kd = {0.5, 0.5, 0.5, 0.5, 0.25, 0.25, 0.25};
            
            for (int i = 0; i < _master_state.q.size(); i++)
            {
                current_position[i] = Kp[i] * ( current_position[i] - robot_state.q[i] )  + Kd[i] * robot_state.dq[i];
            }

            franka::Torques output  = {{current_position[0], current_position[1],
                                        current_position[2], current_position[3] ,
                                        current_position[4] , current_position[5],
                                        current_position[6] }};


            if (time >= 50.0) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(output);
            }
            return output;
        });

    }   

    franka::Torques control_callback(const franka::RobotState& robot_state, franka::Duration)
    {

    }

    franka::JointPositions position_motion_generator_callback(const franka::RobotState& robot_state, franka::Duration)
    {
        // return position profiles
    }

    franka::JointVelocities velocity_motion_generator_callback(const franka::RobotState& robot_state, franka::Duration)
    {
        // return velocity profiles

    }

    void do_receive() 
    { 
        msgIn.body.clear();
        msgIn.header.size = msgIn.size();
        msgIn.body.resize(received_bytes);

        socket_.async_receive_from(
        boost::asio::buffer( msgIn.body.data(),  msgIn.body.size()), slave_endpoint,
        [this](boost::system::error_code ec, std::size_t bytes_recvd)
        {
            if (!ec && bytes_recvd > 0 )
            {
                // master state received here (from master)
                msgIn.body.resize( bytes_recvd );
                msgIn >> _master_state;
                is_master_state_received = true;
                if (debug) std::cout << "[Slave][Received][Bytes][" << slave_endpoint << "]\t" << bytes_recvd  << std::endl;
                // print_array( _master_state.q , "Position");
                // print_array( _master_state.dq , "Speeds");
                // print_array( _master_state.tau_J , "Torques");
            }
            do_receive();
        });
    }

    void do_send(const franka::RobotState& robot_state) 
    { 
        teleop::message<CustomType> msg;
        msg.header.id = CustomType::RobotStateFull;
        msg << robot_state;

        socket_.async_send_to( boost::asio::buffer( msg.body.data(), msg.body.size()), slave_endpoint, 
        [this](boost::system::error_code ec, std::size_t bytes_sent)
            {   
                if ( !ec && bytes_sent == received_bytes )
                {
                    // slave state message sent here (to master)
                    if (debug) std::cout << "[Slave][Sent][Bytes][" << slave_endpoint << "]" << bytes_sent << std::endl;
                }             
            });
    }

    void print_array(std::array<double, 7> &arr, std::string name)
    {   
        // std::string name = "Joints: ";
        std::cout << name.c_str() ;
        for ( int i = 0; i < arr.size(); i++ )
        {
            std::cout << "  " << arr[i] << " , " ;
        }
        std::cout << std::endl;
    }

    void print_array(std::vector<std::uint8_t> &arr)
    {   
        std::string  name = "Raw";
        std::cout << name.c_str() << ":  " ;
        for ( int i = 0; i < arr.size(); i++ )
        {
            std::cout << (int) arr[i] << " , " ;
        }
        std::cout << std::endl;
    }

private:
    // networking stuff
    boost::thread_group tg;
    udp::socket socket_;
    udp::endpoint slave_endpoint;
    int max_length = 8192;
    char send_data_[8192];
    char receive_data_[8192];
    franka::RobotState _master_state; 
    franka::RobotState _slave_state;

    double fake_joint_values = 0.1;
    bool is_master_state_received = false;

    std::size_t received_bytes = 4096;
    teleop::message<CustomType> msgIn;
    teleop::message<CustomType> msgOut;
    

    bool debug = false;

}; // end of client


int main(int argc , char* argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: "  << argv[0] << "  <server-port>  <slave-robot-hostname> \n";
        return 1;
    }

    short port = std::atoi(argv[1]);
    boost::asio::io_service io_service;
    server s(io_service, port, argv[2] );
    std::cout << "bye bye server! " << std::endl;
    return 0;
} 