// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RobotContext.hpp
/// Everything a skill may use during one tick: its robot, the beliefs of the
/// cell, the time.
///
/// It is the only door of a skill to the world. There is no simulator in it,
/// so the same skill can run on a real robot.
///
/// @par Contract
/// - Promises: the robot, the beliefs, the time; no ground truth.
/// - Expects: a skill keeps nothing of it after the tick.
#pragma once

#include "RobotIK/Common/Math.hpp"

#include <span>

namespace robotik
{

class Robot;
class WorldModel;

// ****************************************************************************
/// Per-tick view handed to skills: the robot API and the beliefs.
///
/// A skill that could read the simulator would work in simulation and fail
/// on the real robot, where nobody knows the true pose of the cube. So the
/// scheduler gives a skill this view and nothing else: what the robot
/// measures and what it believes. A skill cannot tell MuJoCo from hardware.
///
/// It is a plain struct of references, built again at each tick: it is
/// valid during the tick only, and a skill keeps none of it. With several
/// robots, @ref robot is the one the skill drives (@c SkillDescription::robot)
/// and @ref team lets it read the others, never command them.
///
/// @code
/// Status Reach::tick(RobotContext& p_context, Seconds)
/// {
///     WorldObject const* box = p_context.world_model.find("box");
///     if (box == nullptr)
///         return Status::FAILURE;
///     auto const q = p_context.robot.inverseKinematics("tool", target(*box));
///     // ... command p_context.robot.getJoints() toward q
///     return Status::RUNNING;
/// }
/// @endcode
// ****************************************************************************
struct RobotContext
{
    /// Joints, sensors, actuators, resources and kinematics.
    Robot& robot;
    /// Beliefs about the objects around the robot.
    WorldModel& world_model;
    /// Robot clock at the start of the tick.
    Seconds time{};
    /// Duration of the tick.
    Seconds dt{};
    /// Every robot of the world, @ref robot included, in world file order:
    /// what they measure of themselves (joints, base, sensors), as a real
    /// cell shares it on its network. Empty for a robot alone. Read only:
    /// a skill commands its own @ref robot, never another one (the
    /// scheduler leases would not see it).
    std::span<Robot const* const> team{};
};

} // namespace robotik
