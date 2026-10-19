// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RobotContext.hpp
/// Everything a skill may use during one tick.
#pragma once

#include "Robotik/Common/Math.hpp"

#include <span>

namespace robotik
{

class Robot;
class WorldModel;

// ****************************************************************************
/// Per-tick view handed to skills: the robot API and the beliefs.
///
/// There is no simulator in here: a skill cannot tell MuJoCo from hardware.
/// It reads what the robot measures and what it believes, never the ground
/// truth.
///
/// @code
/// Status Reach::tick(RobotContext& p_context, Seconds)
/// {
///     WorldObject const* box = p_context.world_model.find("box");
///     if (box == nullptr)
///         return Status::FAILURE;
///     auto const q = p_context.robot.inverseKinematics("tool", target(*box));
///     // ... command p_context.robot.joints() toward q
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
    /// cell shares it on its network. Empty for a robot alone.
    std::span<Robot* const> team{};
};

} // namespace robotik
