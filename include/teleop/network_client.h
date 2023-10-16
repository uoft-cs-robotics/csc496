 
#pragma once

#include "commons.h"
#include "messages.h"


namespace teleop
{
    class NetworkClient {
        public:
            NetworkClient(boost::asio::io_service& io_service, char* server_port, char* server_ip);
            ~NetworkClient();

            franka::RobotState _master_state;
            franka::RobotState _slave_state;
            bool is_state_received = false;
            void DoSend(const franka::RobotState& robot_state);

        private: 

            void DoReceive();
            bool debug = false;

            teleop::message<CustomType> msgIn;

            // networking stuff
            boost::thread_group tg;
            udp::socket socket_;
            udp::endpoint master_endpoint;
            udp::endpoint slave_endpoint;

            std::size_t received_bytes = 4096;
    };
}
