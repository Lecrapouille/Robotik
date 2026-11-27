// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RobotModel.hpp
/// What does not change in a robot after it is loaded: its name, its URDF,
/// its work frame and its moving joints, in the order of their @ref JointId.
///
/// It is the "model" half of the model / state split of Pinocchio: the
/// @ref Robot holds the state (measured joints, base, clock) and points to
/// its model. So code that keeps data for later (a taught waypoint, a saved
/// posture) can say for which robot it was made, and check it.
///
/// @par Contract
/// - Promises: a model never changes once the robot is built (it is shared
///   as @c std::shared_ptr<RobotModel const>); @c joints, @c q_indices and
///   @c v_indices have one entry per joint of the robot.
/// - Limits: a tool is given when the robot is built; there is no tool
///   change while it runs, so the model stays the same for its whole life.
#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// The constant part of a robot (see the file comment).
///
/// @code
/// robotik::RobotModel const& model = robot.getModel();
/// std::cout << model.name << " has " << model.joints.size() << " joints\n";
/// if (!sameJoints(model, other.getModel()))
///     return; // a waypoint of one does not fit the other
/// @endcode
// ****************************************************************************
struct RobotModel
{
    /// Name of the robot in its URDF (@c <robot name="...">).
    std::string name;
    /// URDF file of the robot.
    std::filesystem::path urdf;
    /// URDF file of the tool mounted on the flange, or empty.
    std::filesystem::path tool_urdf;
    /// Work frame: @c tcp when a tool is mounted, otherwise @c tool0,
    /// @c end_effector, or the last moving link.
    std::string tool;
    /// Names of the moving joints, in the order of their @ref JointId.
    std::vector<std::string> joints;
    /// Pinocchio q index of each joint (-1 when not modeled).
    std::vector<int> q_indices;
    /// Pinocchio v index of each joint (-1 when not modeled).
    std::vector<int> v_indices;
};

// ----------------------------------------------------------------------------
/// @return True when both models have the same joints in the same order:
///   joint positions written for one have the same meaning on the other.
// ----------------------------------------------------------------------------
[[nodiscard]] inline bool sameJoints(RobotModel const& p_a,
                                     RobotModel const& p_b)
{
    return p_a.joints == p_b.joints;
}

} // namespace robotik
