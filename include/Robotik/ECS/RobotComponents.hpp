// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RobotComponents.hpp
/// ECS markers of a loaded robot root entity.
#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

namespace robotik::ecs
{

// ****************************************************************************
/// Marks the URDF root entity as the robot instance.
///
/// Use it to find every robot of a world:
///
/// @code
/// world.each<robotik::ecs::RobotTag>(
///     [](compages::world::Entity p_root, robotik::ecs::RobotTag&) { ... });
/// @endcode
// ****************************************************************************
struct RobotTag
{
    /// Reserved lifecycle flag (always true for loaded robots).
    bool alive = true;
};

// ****************************************************************************
/// Human-readable robot id and source model path, stored next to
/// @ref RobotTag on the root entity.
///
/// @code
/// auto const& identity = root.get<robotik::ecs::RobotIdentity>();
/// std::cout << identity.name << " from " << identity.model_path << '\n';
/// @endcode
// ****************************************************************************
struct RobotIdentity
{
    /// Name from the URDF @c robot element.
    std::string name;
    /// URDF file.
    std::filesystem::path model_path;
};

// ****************************************************************************
/// Marker of a teach-pendant waypoint, parented to the robot root.
///
/// The entity pose is the recorded tool pose in the robot base frame. The
/// index matches @ref robotik::TeachPendant waypoints, so the view can draw
/// a small frame at each recorded point.
///
/// @code
/// marker.add<robotik::ecs::TeachMarker>(robotik::ecs::TeachMarker{ 3 });
/// @endcode
// ****************************************************************************
struct TeachMarker
{
    /// Index of the waypoint in the teach pendant list.
    std::size_t index = 0;
};

} // namespace robotik::ecs
