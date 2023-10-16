 
#pragma once

#include "commons.h"
#include "messages.h"
#include "network_server.h"
#include "common.h"


namespace teleop
{
    class Follower {
        public:
            Follower(char* server_port, char* follower_ip);
            ~Follower();

            franka::RobotState _master_state;
            franka::RobotState _slave_state;
            bool is_state_received = false;
            
            void GoHome();

            void Control(  std::function<franka::Torques( 
                        const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
                        franka::Duration period,  bool _is_leader_state_received )> control_loop
                    );

            void Read( std::function<bool(
                        const franka::RobotState& _fstate,  const franka::RobotState& _lstate, 
                        franka::Duration period,  bool _is_leader_state_received )> read_loop
                    );

        private: 

            char* follower_ip;
            char* server_port; 
            char* server_ip;
            boost::asio::io_service io_service;
            teleop::NetworkServer server;
            
            franka::Robot robot;
            // franka::Model model; 
            void InitializeRobot();
        
    };
}
