// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file RobotComponents.hpp
// @brief ECS markers of a loaded robot root entity.
#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

namespace robotik::ecs
{

// ****************************************************************************
//! @brief Marks the URDF root entity as the robot instance.
// ****************************************************************************
struct RobotTag
{
    //!< Reserved lifecycle flag (always true for loaded robots).
    bool alive = true;
};

// ****************************************************************************
//! @brief Human-readable robot id and source model path.
// ****************************************************************************
struct RobotIdentity
{
    //!< Name from the URDF @c robot element.
    std::string name;
    //!< URDF file.
    std::filesystem::path model_path;
};

// ****************************************************************************
//! @brief Marker of a teach-pendant waypoint, parented to the robot root.
//!
//! The entity pose is the recorded tool pose in the robot base frame. The
//! index matches @ref robotik::TeachPendant waypoints.
// ****************************************************************************
struct TeachMarker
{
    std::size_t index = 0;
};

} // namespace robotik::ecs
