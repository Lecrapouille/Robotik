// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file ToolMount.hpp
/// Which tool is mounted on a robot, and by which links: a link of the robot
/// (the flange) receives a link of the tool (its mount).
///
/// The kinematics, the drawn world and the physics all mount the tool, so
/// they read the same names here, and the world file can declare them
/// (@c attach: @c { parent: robot.flange, frame: tool_mount }).
///
/// @par Contract
/// - Expects: the robot URDF has the link @c flange, the tool URDF has the
///   links @c mount and @c tcp (the tip, the work frame of the robot).
/// - Errors: a missing link throws @c std::runtime_error when the robot is
///   built (@c StandaloneRobot::open turns it into a @c Result).
#pragma once

#include <filesystem>
#include <string>

namespace robotik
{

// ****************************************************************************
/// A tool on a robot (see the file comment). Empty @ref urdf: no tool (a
/// hexapod, a mobile base).
///
/// @code
/// robotik::ToolMount gripper{ .urdf = "gripper.urdf" }; // flange, tool_mount
/// robotik::ToolMount drill{ .urdf = "drill.urdf", .flange = "link6" };
/// @endcode
// ****************************************************************************
struct ToolMount
{
    /// URDF of the tool; empty for no tool.
    std::filesystem::path urdf;
    /// Link of the robot that carries the tool.
    std::string flange = "flange";
    /// Link of the tool that lands on @ref flange.
    std::string mount = "tool_mount";
};

// ----------------------------------------------------------------------------
/// @return True when a tool is mounted.
// ----------------------------------------------------------------------------
[[nodiscard]] inline bool hasTool(ToolMount const& p_tool)
{
    return !p_tool.urdf.empty();
}

} // namespace robotik
