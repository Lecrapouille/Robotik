// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file ObjectComponents.hpp
// @brief Pick-and-place objects in the world and vacuum grasp on the tool.
// @ref SceneObject — manipulable item from scenario YAML (name, shape, color,
// size). @ref VacuumGripper — on the end-effector link: which @ref SceneObject
// is currently stuck to the cup and how long the tool is (no parallel-jaw
// URDF).
#pragma once

#include "Compages/Core/Units.hpp"
#include "Compages/World/EntityId.hpp"

#include <array>
#include <string>

namespace robotik::ecs
{

// ****************************************************************************
// @brief Pick-and-place prop spawned from a scenario file.
//
// Pose is the Compages transform on an entity parented to the robot root
// (coordinates in the robot base frame).
// ****************************************************************************
struct SceneObject
{
    // ------------------------------------------------------------------------
    // @brief Primitive shape used for rendering.
    // ------------------------------------------------------------------------
    enum class Type
    {
        CUBE, //!< Single box mesh scaled by @ref size.
        BOX   //!< Open container: floor plus four walls in the scene loader.
    };

    //!< Unique name referenced by skills and assertions.
    std::string name;
    //!< Shape kind for @ref Simulation spawn.
    Type type = Type::CUBE;
    //!< Full extents (SI: m) along X, Y, Z.
    std::array<Length, 3> size{ Length(0.04), Length(0.04), Length(0.04) };
    //!< RGB color in @c [0, 1] for rendering and color detection.
    std::array<float, 3> color{ 0.8f, 0.8f, 0.8f };
};

// ****************************************************************************
// @brief Virtual vacuum tool: at most one @ref SceneObject may be attached.
// ****************************************************************************
struct VacuumGripper
{
    //!< Entity id of the grasped object, or invalid when empty.
    compages::world::EntityId held{};
    //!< Distance from flange frame to cup tip along flange +Z (SI: m).
    Length tool_length{ 0.06 };
};

} // namespace robotik::ecs
