#pragma once

#include "Robotik/Robotik.hpp"

namespace robotik::debug
{

// ****************************************************************************
//! \brief Print the robot to the console.
// ****************************************************************************
void printRobot(const Robot& p_robot);

// ****************************************************************************
//! \brief Print the joint to the console.
// ****************************************************************************
void printJoint(const Joint& p_joint);

// ****************************************************************************
//! \brief Print the link to the console.
// ****************************************************************************
void printLink(const Link& p_link);
} // namespace robotik::debug