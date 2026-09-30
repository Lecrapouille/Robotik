// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file ObjectComponents.hpp
 * @brief Scenario props and virtual suction gripper on the tool link.
 */

#pragma once

#include "Compages/World/EntityId.hpp"

#include <array>
#include <string>

namespace robotik::ecs
{

/**
 * @brief Pick-and-place prop spawned from a scenario file.
 *
 * Pose is the Compages transform on an entity parented to the robot root
 * (coordinates in the robot base frame).
 */
struct SceneObject
{
    /** @brief Primitive shape used for rendering. */
    enum class Type
    {
        Cube, ///< Single box mesh scaled by @ref size.
        Box   ///< Open container: floor plus four walls in the scene loader.
    };

    /** @brief Unique name referenced by skills and assertions. */
    std::string name;

    /** @brief Shape kind for @ref Simulation spawn. */
    Type type = Type::Cube;

    /** @brief Full extents in meters (X, Y, Z). */
    std::array<float, 3> size{ 0.04f, 0.04f, 0.04f };

    /** @brief RGB color in @c [0, 1] for rendering and color detection. */
    std::array<float, 3> color{ 0.8f, 0.8f, 0.8f };
};

/**
 * @brief Virtual vacuum tool: at most one @ref SceneObject may be attached.
 */
struct VacuumGripper
{
    /** @brief Entity id of the grasped object, or invalid when empty. */
    compages::world::EntityId held{};

    /**
     * @brief Distance from flange frame to cup tip along flange +Z, in meters.
     */
    double tool_length = 0.06;
};

} // namespace robotik::ecs
