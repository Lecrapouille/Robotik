// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file ObjectComponents.hpp
// @brief Objects of the world spawned from a scenario (ground truth).
#pragma once

#include "Compages/Core/Units.hpp"

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

} // namespace robotik::ecs
