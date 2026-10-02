//=============================================================================
// Source: https://github.com/Robotik-Core/Robotik
// Copyright (c) 2020-2026 Quentin Quadrat.
// Distributed under MIT License.
//=============================================================================
// @file JointProjectionSystem.hpp
// @brief Writes @ref ecs::JointState into Compages @c RevoluteJoint /
// @c PrismaticJoint so @c KinematicSystem can rebuild link transforms for
// rendering and @c worldMatrix after simulation.
//=============================================================================

#pragma once

namespace compages::world
{
class World;
}

namespace robotik
{

// ****************************************************************************
// @brief Copies simulated joint positions onto @c RevoluteJoint / @c
// PrismaticJoint.
// ****************************************************************************
class JointProjectionSystem
{
public:

    // -------------------------------------------------------------------------
    //! @brief Updates Compages joint angles or offsets from @ref
    //! ecs::JointState.
    //! @param p_world ECS world.
    // -------------------------------------------------------------------------
    void update(compages::world::World& p_world) const;
};

} // namespace robotik
