// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file MujocoSyncSystem.hpp
// @brief ECS ↔ MuJoCo bridge: @ref writeCommands pushes @ref
// ecs::ActuatorCommand before @ref MujocoBackend::step; @ref readState fills
// @ref ecs::JointState from @c qpos/@c qvel via @ref ecs::MujocoJointBinding.
#pragma once

namespace compages::world
{
class World;
}

namespace robotik
{

class MujocoBackend;

// ****************************************************************************
// @brief Reads MuJoCo @c qpos/@c qvel into ECS and writes actuator efforts to
// MuJoCo.
// ****************************************************************************
class MujocoSyncSystem
{
public:

    // -------------------------------------------------------------------------
    //! @brief Pulls generalized coordinates from MuJoCo into @ref
    //! ecs::JointState.
    //! @param p_world ECS world.
    //! @param p_mujoco Dynamics backend after @ref MujocoBackend::step.
    // -------------------------------------------------------------------------
    void readState(compages::world::World& p_world,
                   MujocoBackend& p_mujoco) const;

    // -------------------------------------------------------------------------
    //! @brief Applies @ref ecs::ActuatorCommand to controls and @c
    //! qfrc_applied.
    //! @param p_world ECS world.
    //! @param p_mujoco Dynamics backend before @ref MujocoBackend::step.
    // -------------------------------------------------------------------------
    void writeCommands(compages::world::World& p_world,
                       MujocoBackend& p_mujoco) const;
};

} // namespace robotik
