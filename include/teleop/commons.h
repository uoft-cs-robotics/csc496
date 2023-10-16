#pragma once

#include <memory>
#include <thread>
#include <mutex>
#include <deque>
#include <vector>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <cstdint>

#include <boost/bind/bind.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>

#include <franka/robot_state.h>
#include <franka/errors.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
using boost::asio::ip::udp;


enum class CustomType : uint32_t 
{
    RobotStateFull,
    RobotPositionOnly,
    RobotSpeedOnly,
    RobotTorqueOnly,
    RobotTorqueWithoutGravity
};


