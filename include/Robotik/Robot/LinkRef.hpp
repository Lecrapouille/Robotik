// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file LinkRef.hpp
/// A robot link found once by its name, then reused at every step.
///
/// @par Contract
/// - Promises: a @ref robotik::LinkRef stays good for the life of the robot
///   that made it, across resets (the kinematic model is built once).
/// - Expects: code uses it only on the robot that made it.
/// - Errors: a handle from another robot, or never found, is a mistake in
///   the code: @ref robotik::Robot::worldPose throws @c std::logic_error.
#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>

namespace robotik
{

class Robot;

// ****************************************************************************
/// A link of the robot (the arm segment, the wrist...), found once.
///
/// Why: finding a link by its name walks the tree of the robot and compares
/// strings. That is fine once, too slow for a sensor that reads its link
/// 1000 times a second. @ref Robot::requireLink does the search once;
/// @ref Robot::worldPose then reads the pose with no search.
///
/// Guarantees: the handle is opaque. It names a frame of the kinematic model
/// and the robot that made it, nothing of the simulator (no ECS entity). The
/// robot checks the owner at each @ref Robot::worldPose: a handle of another
/// robot (two arms in a cell), or a default one, throws @c std::logic_error
/// instead of reading the wrong frame. The check is one integer compare.
///
/// Limits: the handle stays good across resets, because the model of a robot
/// is built once. A new robot (even from the same URDF) needs new handles.
///
/// @code
/// robotik::LinkRef const wrist = robot.requireLink("link6"); // once
/// robotik::Pose const pose = robot.worldPose(wrist);    // every step
/// @endcode
// ****************************************************************************
class LinkRef
{
public:

    /// A handle that names no link; @ref Robot::worldPose refuses it.
    LinkRef() = default;

private:

    friend class Robot;

    /// Value of @ref m_frame for the robot base.
    static constexpr std::size_t BASE = std::numeric_limits<std::size_t>::max();

    /// @param p_robot Serial number of the robot that made it.
    /// @param p_frame Index of the link in the kinematic model, or @ref BASE.
    LinkRef(std::uint64_t p_robot, std::size_t p_frame)
        : m_robot(p_robot), m_frame(p_frame)
    {
    }

    /// Serial number of the robot that made it; 0 for none.
    std::uint64_t m_robot = 0;
    /// Index of the link in the kinematic model, or @ref BASE.
    std::size_t m_frame = BASE;
};

} // namespace robotik
