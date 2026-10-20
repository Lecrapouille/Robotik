// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file LinkRef.hpp
/// A robot link found once by its name, then reused at every step.
#pragma once

#include "Compages/World/Entity.hpp"

#include <cstddef>
#include <limits>

namespace robotik
{

// ****************************************************************************
/// The handles needed to read where a link of the robot is, found once.
///
/// A link is a rigid part of the robot (the arm segment, the wrist...).
/// Finding it by its name walks the tree of the robot and compares strings:
/// fine once, too slow for a sensor that reads its link 1000 times a second.
/// @ref Robot::resolve does the search once; @ref Robot::worldPose(LinkRef
/// const&) then reads the pose with no search.
///
/// @code
/// robotik::LinkRef const wrist = robot.resolve("link6"); // once
/// robotik::Pose const pose = robot.worldPose(wrist);    // every step
/// @endcode
// ****************************************************************************
struct LinkRef
{
    /// Value of @ref frame for the robot base.
    static constexpr std::size_t BASE = std::numeric_limits<std::size_t>::max();

    /// Entity of the link in the world (the robot root for the base); empty
    /// when the robot draws no entity for this link.
    compages::world::Entity entity;
    /// Index of the link in the kinematic model, or @ref BASE.
    std::size_t frame = BASE;
};

} // namespace robotik
