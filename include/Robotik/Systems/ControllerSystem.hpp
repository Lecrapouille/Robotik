// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file ControllerSystem.hpp
// @brief Maps @ref ecs::JointCommand to @ref ecs::ActuatorCommand via
// joint-space PD loops (rate-limited setpoint). Runs each physics sub-step in
// @ref RobotRuntime::pipeline before MuJoCo receives efforts.
#pragma once

#include "Compages/Core/Units.hpp"

namespace compages::world
{
class World;
}

namespace robotik
{

// ****************************************************************************
// @brief Writes actuator efforts from joint commands and controller components.
//
// Position mode uses rate-limited reference tracking on @ref
// ecs::PositionController.
// ****************************************************************************
class ControllerSystem
{
public:

    // -------------------------------------------------------------------------
    //! @brief Updates all entities with joint command and controller
    //! components.
    //! @param p_world ECS world.
    //! @param p_dt Timestep used to slew position references.
    // -------------------------------------------------------------------------
    void update(compages::world::World& p_world, Seconds p_dt) const;
};

} // namespace robotik
