/** 
 * Muhammad Arshad 
 * 26-July-2021
 * See license file
**/


#include "teleop/network_client.h"

namespace teleop
{

NetworkClient::NetworkClient(boost::asio::io_service& io_service, char* server_port, char* server_ip)
: socket_( io_service, udp::endpoint(udp::v4(), 0))
{
    udp::resolver resolver(io_service);
    udp::resolver::query query( server_ip , server_port );
    slave_endpoint = *resolver.resolve( query );   
    DoReceive();

    // multiple threads 
    for (unsigned i = 0; i < boost::thread::hardware_concurrency() * 2; ++i)
    {
        tg.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
    }

}

NetworkClient::~NetworkClient() 
{
    tg.join_all();
}

void NetworkClient::DoSend(const franka::RobotState& robot_state)
{
    teleop::message<CustomType> msg;
    msg.header.id = CustomType::RobotStateFull;
    msg << robot_state;

    socket_.async_send_to( boost::asio::buffer( msg.body.data(), msg.body.size() ), slave_endpoint, 
    [this](boost::system::error_code ec, std::size_t bytes_sent)
    {   
        if ( !ec && bytes_sent > 0 )  
        {
            if (debug) std::cout << "[Master][Sent][Bytes][" << slave_endpoint << "]\t" << bytes_sent << std::endl;
        }             
    });
}

void NetworkClient::DoReceive()
{
    msgIn.body.clear();
    msgIn.header.size = msgIn.size();
    msgIn.body.resize(received_bytes);

    socket_.async_receive_from(
    boost::asio::buffer(msgIn.body.data(), msgIn.body.size()), master_endpoint,
    [this](boost::system::error_code ec, std::size_t bytes_recvd)
    {

        if (!ec)
        {
            msgIn.body.resize( bytes_recvd );
            msgIn >> _slave_state;
            is_state_received = true;
            if (debug) std::cout << "[Slave][Received][Bytes][" << slave_endpoint << "]\t" << bytes_recvd  << std::endl;
        }
        DoReceive();
    });
}

}