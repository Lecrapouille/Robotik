// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file ControllerSystem.hpp
 * @brief Maps @ref ecs::JointCommand to @ref ecs::ActuatorCommand via PD loops.
 */

#pragma once

namespace compages::world
{
class World;
}

namespace robotik
{

/**
 * @brief Writes actuator efforts from joint commands and controller components.
 *
 * Position mode uses rate-limited reference tracking on @ref ecs::PositionController.
 */
class ControllerSystem
{
public:

    /**
     * @brief Updates all entities with joint command and controller components.
     * @param p_world ECS world.
     * @param p_dt Timestep used to slew position references.
     */
    void update(compages::world::World& p_world, double p_dt);
};

} // namespace robotik
