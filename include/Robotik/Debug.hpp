#pragma once

#include "Robotik/Robot.hpp"

namespace robotik::debug
{

// ****************************************************************************
//! \brief Print the robot as scene graph to the console.
//! \param p_robot The robot to print.
//! \param p_detailed If true, show detailed information (transforms, geometry).
//!                   If false, show only the tree structure (default).
// ****************************************************************************
void printRobot(const Robot& p_robot, bool p_detailed = false);

} // namespace robotik::debug