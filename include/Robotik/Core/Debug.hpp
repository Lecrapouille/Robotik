/**
 * @file Debug.hpp
 * @brief Debug printing functions for debugging the robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <string>

namespace robotik
{

// Forward declarations
class Robot;

namespace debug
{

// ----------------------------------------------------------------------------
//! \brief Print a robot's hierarchy in a human-readable format.
//!
//! \param p_robot The robot to print.
//! \param p_detailed Whether to include detailed information (transforms,
//! etc.).
//! \return String representation of the robot hierarchy.
// ----------------------------------------------------------------------------
std::string printRobot(Robot const& p_robot, bool const p_detailed = false);

} // namespace debug

} // namespace robotik
