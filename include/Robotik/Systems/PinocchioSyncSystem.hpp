//=============================================================================
// Source: https://github.com/Robotik-Core/Robotik
// Copyright (c) 2020-2026 Quentin Quadrat.
// Distributed under MIT License.
//=============================================================================
// @file PinocchioSyncSystem.hpp
// @brief Builds Pinocchio @c q from @ref ecs::JointState (@ref
// ecs::PinocchioJointBinding). Call @ref PinocchioBackend::updateKinematics
// afterward for FK used by skills, @ref GraspSystem, and @ref toolTip.
//=============================================================================

#pragma once

namespace compages::world
{
class World;
}

namespace robotik
{

class PinocchioBackend;

// ****************************************************************************
// @brief Fills Pinocchio @c q (and optionally @c v) from @ref ecs::JointState.
// ****************************************************************************
class PinocchioSyncSystem
{
public:

    // -------------------------------------------------------------------------
    //! @brief Writes bound joint states into the Pinocchio model.
    //! @param p_world ECS world.
    //! @param p_pinocchio Backend to update; call @ref
    //! PinocchioBackend::updateKinematics after.
    // -------------------------------------------------------------------------
    void update(compages::world::World& p_world,
                PinocchioBackend& p_pinocchio) const;
};

} // namespace robotik
