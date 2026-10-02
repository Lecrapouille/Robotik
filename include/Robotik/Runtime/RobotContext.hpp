// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file RobotContext.hpp
//! @brief Per-tick view passed into skills: world, backends, and timing.
#pragma once

#include "Compages/Core/Units.hpp"
#include "Robotik/ECS/JointComponents.hpp"

#include <string>
#include <unordered_map>
#include <variant>

namespace compages::world
{
class World;
}

namespace robotik
{

class PinocchioBackend;
class MujocoBackend;

//! Revolute → @ref Radians ; prismatic → @ref Length (same as @ref ecs::JointCommand).
using JointGoal = std::variant<Radians, Length>;

//! Named joint targets for @ref RobotRuntime::hold and skills.
using JointPosture = std::unordered_map<std::string, JointGoal>;

//! @ref JointGoal as SI scalar for @ref ecs::JointCommand::position (rad or m).
inline double jointGoalSi(JointGoal const& p_goal)
{
    return std::visit([](auto const& value) { return value.value(); }, p_goal);
}

//! Compare @ref ecs::JointState to @ref JointGoal using typed units.
inline bool exceedsJointGoalTolerance(ecs::JointMechanism p_mechanism,
                                      ecs::JointState const& p_state,
                                      JointGoal const& p_goal,
                                      Radians p_angle_tolerance,
                                      Length p_linear_tolerance)
{
    if (p_mechanism == ecs::JointMechanism::Revolute)
    {
        return ecs::exceedsTolerance(
            std::get<ecs::RevoluteJointState>(p_state).position,
            std::get<Radians>(p_goal),
            p_angle_tolerance);
    }
    return ecs::exceedsTolerance(
        std::get<ecs::PrismaticJointState>(p_state).position,
        std::get<Length>(p_goal),
        p_linear_tolerance);
}

// ****************************************************************************
//! @brief Everything a skill may read or write during one tick.
//!
//! Skills never talk to backends directly except through this bundle and ECS
//! components on @c world.
// ****************************************************************************
struct RobotContext
{
    //!< Canonical scene graph and ECS registry.
    compages::world::World& world;
    //!< Analytical kinematics (FK, Jacobians, IK).
    PinocchioBackend& kinematics;
    //!< MuJoCo dynamics, or null when the same skill runs on hardware.
    MujocoBackend* simulation = nullptr;
    //!< Simulation time at the start of the tick.
    Seconds time{};
    //!< Duration of the tick.
    Seconds dt{};
};

} // namespace robotik
