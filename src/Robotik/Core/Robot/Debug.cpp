/**
 * @file Debug.cpp
 * @brief Debug printing functions for debugging the robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Common/Conversions.hpp"
#include "Robotik/Core/Robot/Robot.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

// ANSI color codes
namespace
{
const std::string RESET = "\033[0m";
const std::string BOLD = "\033[1m";

// Colors
const std::string BLUE = "\033[34m";
const std::string GREEN = "\033[32m";
const std::string YELLOW = "\033[33m";
const std::string MAGENTA = "\033[35m";
const std::string WHITE = "\033[37m";

// Emojis for different elements
const std::string ROBOT_EMOJI = "🤖";
} // namespace

namespace robotik::debug
{

// ----------------------------------------------------------------------------
std::string printRobot(Robot const& p_robot, bool const p_detailed)
{
    std::ostringstream oss;

    oss << ROBOT_EMOJI << " " << BOLD << MAGENTA << "Blueprint" << RESET << ": "
        << BOLD << WHITE << p_robot.name() << RESET << std::endl;

    auto const& blueprint = p_robot.blueprint();
    
    // Print joint information
    oss << "  " << BOLD << BLUE << "Joints" << RESET << " (" << blueprint.numJoints() << "):" << std::endl;
    blueprint.forEachJointData([&](JointData const& joint, size_t i) {
        oss << "    [" << i << "] " << BOLD << joint.name << RESET;
        if (p_detailed)
        {
            oss << " (parent_link: ";
            if (joint.parent_link_index == SIZE_MAX)
                oss << "root";
            else
                oss << joint.parent_link_index;
            oss << ", limits: [" << joint.position_min << ", " << joint.position_max << "])";
        }
        oss << std::endl;
    });

    // Print link information
    oss << "  " << BOLD << GREEN << "Links" << RESET << " (" << blueprint.numLinks() << "):" << std::endl;
    blueprint.forEachLinkData([&](LinkData const& link, size_t i) {
        oss << "    [" << i << "] " << BOLD << link.name << RESET;
        if (p_detailed)
        {
            oss << " (parent joint: ";
            if (link.parent_joint_index == SIZE_MAX)
                oss << "root";
            else
                oss << link.parent_joint_index;
            oss << ")";
        }
        oss << std::endl;
    });

    // Print geometry information
    oss << "  " << BOLD << YELLOW << "Geometries" << RESET << " (" << blueprint.numGeometries() << "):" << std::endl;
    blueprint.forEachGeometryData([&](GeometryData const& geom, size_t i) {
        oss << "    [" << i << "] " << BOLD << geom.name << RESET;
        if (p_detailed && geom.parent_link_index < blueprint.numLinks())
        {
            oss << " (parent link: " << blueprint.linkData(geom.parent_link_index).name << ")";
        }
        oss << std::endl;
    });

    return oss.str();
}

} // namespace robotik::debug
