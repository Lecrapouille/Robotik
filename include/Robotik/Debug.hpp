#pragma once

#include "Robotik/Robot.hpp"
#include <string>

namespace robotik::debug
{

// ****************************************************************************
//! \brief Print the robot as scene graph to a string.
//! \param p_robot The robot to print.
//! \param p_detailed If false, show only the tree structure (default).
//!  If true, show detailed information (local and world transforms, geometry).
//! \return A string containing the formatted robot representation.
// ****************************************************************************
std::string printRobot(Robot const& p_robot, bool p_detailed = false);

} // namespace robotik::debug