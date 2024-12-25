/** 
 * Muhammad Arshad 
 * 26-July-2021
 * See license file
**/

#include "teleop/network_server.h"

namespace teleop
{

NetworkServer::NetworkServer(boost::asio::io_service& io_service, short port)
: socket_( io_service, udp::endpoint(udp::v4(), port))
{

    DoReceive();
    // multiple threads
    for (unsigned i = 0; i < boost::thread::hardware_concurrency() * 2; ++i)
    {
        tg.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
    }
}
 
NetworkServer::~NetworkServer()
{
    tg.join_all();
}

void NetworkServer::DoSend(const franka::RobotState& robot_state)
{
    teleop::message<CustomType> msg;
    msg.header.id = CustomType::RobotStateFull;
    msg << robot_state;

    socket_.async_send_to( boost::asio::buffer( msg.body.data(), msg.body.size()), slave_endpoint, 
    [this](boost::system::error_code ec, std::size_t bytes_sent)
    {   
        if ( !ec && bytes_sent == received_bytes )
        {
            if (debug) std::cout << "[Server][Sent][Bytes][" << slave_endpoint << "]" << bytes_sent << std::endl;
        }             
    });
}

void NetworkServer::DoReceive()
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
            is_state_received = true;
            if (debug) std::cout << "[Server][Received][Bytes][" << slave_endpoint << "]\t" << bytes_recvd  << std::endl;
        }
        DoReceive();
    });
}

}