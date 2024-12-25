#pragma once

#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>

namespace robotContext {
    extern franka::Robot *robot;
    extern franka::Gripper *gripper;
    extern franka::Model *model;
}


