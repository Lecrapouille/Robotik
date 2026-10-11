// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file ObjectComponents.hpp
// @brief Objects of the world spawned from a scenario (ground truth).
#pragma once

#include "Robotik/Math/Si.hpp"

#include <array>
#include <string>

namespace robotik::ecs
{

//! Wall thickness in metres for @ref SceneObject::Type::CONTAINER (rendering, @ref
//! Robotik/Scene/ContainerBounds.hpp).
inline constexpr double CONTAINER_WALL_M = 0.005;

// ****************************************************************************
// @brief Primitive object of the world (prop or primitive body).
// Pose is the Compages transform on an entity parented to the Z-up world
// frame (coordinates in the world frame).
// ****************************************************************************
struct SceneObject
{
    // ------------------------------------------------------------------------
    // @brief Primitive shape used for rendering.
    // ------------------------------------------------------------------------
    enum class Type
    {
        BOX,      //!< Solid box scaled by @ref size.
        CONTAINER //!< Open-top bin; cavity from @ref scene::innerBounds.
    };

    //!< World entity id, referenced by skills and assertions.
    std::string name;
    //!< Shape kind for rendering and the grasp rules.
    Type type = Type::BOX;
    //!< Full extents (SI: m) along X, Y, Z.
    std::array<Length, 3> size{ Length(0.04), Length(0.04), Length(0.04) };
    //!< RGB color in @c [0, 1] for rendering and color detection.
    std::array<float, 3> color{ 0.8f, 0.8f, 0.8f };
};

// ****************************************************************************
//! @brief Marks a @ref SceneObject moved by the physics backend: the grasp
//! system and the container rules leave it alone.
// ****************************************************************************
struct PhysicsBody
{
    //!< Body name in the backend.
    std::string body;
};

} // namespace robotik::ecs
