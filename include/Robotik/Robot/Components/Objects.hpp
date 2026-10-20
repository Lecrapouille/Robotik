// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file ObjectComponents.hpp
/// Objects of the world spawned from a scenario (ground truth).
///
/// These components are the truth of the simulator: metrics, assertions and
/// RL rewards may read them, skills never do (they read the
/// @ref robotik::WorldModel).
#pragma once

#include "Robotik/Common/Math.hpp"

#include <array>
#include <string>

namespace robotik::ecs
{

/// Wall thickness in metres for @ref Shape::Type::CONTAINER (rendering, @ref
/// Robotik/Scene/ContainerBounds.hpp).
inline constexpr double CONTAINER_WALL_M = 0.005;

// ****************************************************************************
/// Primitive object of the world (prop or primitive body).
///
/// Pose is the Compages transform on an entity parented to the Z-up world
/// frame (coordinates in the world frame).
///
/// @code
/// robotik::ecs::Shape cube;
/// cube.name = "red_cube";
/// cube.size = { Length(0.05), Length(0.05), Length(0.05) };
/// cube.color = { 1.0f, 0.0f, 0.0f };
/// entity.add<robotik::ecs::Shape>(cube);
/// @endcode
// ****************************************************************************
struct Shape
{
    // ------------------------------------------------------------------------
    /// Primitive shape used for rendering.
    // ------------------------------------------------------------------------
    enum class Type
    {
        BOX,      ///< Solid box scaled by @ref size.
        CONTAINER ///< Open-top bin; cavity from @ref scene::innerBounds.
    };

    /// World entity id, referenced by skills and assertions.
    std::string name;
    /// Shape kind for rendering and the grasp rules.
    Type type = Type::BOX;
    /// Full extents (SI: m) along X, Y, Z.
    std::array<Length, 3> size{ Length(0.04), Length(0.04), Length(0.04) };
    /// RGB color in @c [0, 1] for rendering and color detection.
    std::array<float, 3> color{ 0.8F, 0.8F, 0.8F };
};

// ****************************************************************************
/// Marks a @ref Shape moved by the physics backend: the grasp system and the
/// container rules leave it alone.
///
/// @code
/// entity.add<robotik::ecs::PhysicsBody>(robotik::ecs::PhysicsBody{ "box" });
/// @endcode
// ****************************************************************************
struct PhysicsBody
{
    /// Body name in the backend.
    std::string body;
};

// ****************************************************************************
/// Marks a @ref Shape drawn for the eye only (a sensor beam, a trail): not a
/// world entity. Grasp, range sensing and the ground truth ignore it.
///
/// Not empty: EnTT stores no instance of an empty type, and the Compages
/// entity API returns references to its components.
///
/// @code
/// beam.add<robotik::ecs::Decoration>();
/// @endcode
// ****************************************************************************
struct Decoration
{
    /// Always true; only here so the type is not empty.
    bool decorative = true;
};

} // namespace robotik::ecs
