 
#pragma once

#include "teleop/commons.h"
#include "teleop/messages.h"
#include "teleop/network_client.h"
#include "common.h"
#include "context.h"

namespace teleop
{
  
    class Leader {
        public:
            Leader(char* server_port, char* server_ip, char* leader_ip);
            ~Leader();

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
            teleop::NetworkClient client;
            


            void InitializeRobot();
        
    };
}
