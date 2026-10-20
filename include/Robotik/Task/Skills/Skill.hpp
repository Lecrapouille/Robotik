// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Skill.hpp
/// One behaviour of a robot (move the arm, grasp, dip a brush), and the card
/// that tells the scheduler what it needs.
///
/// A skill is ticked again and again until it succeeds or fails. Its
/// @ref robotik::SkillDescription lists its resources and its priority, so the
/// scheduler can run several skills at once without conflict.
///
/// @par Contract
/// - Promises: @c reset before the first tick; @c cancel when the scheduler
///   stops the skill; @c why tells a person why it failed.
/// - Expects: a skill reads only its @ref robotik::RobotContext; it declares
///   the resources it needs in its @ref robotik::SkillDescription.
#pragma once

#include "Robotik/Robot/Resources.hpp"
#include "Robotik/Task/Status.hpp"

#include "Robotik/Common/Math.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace robotik
{

struct RobotContext;

/// Higher runs first and may preempt lower (e.g. stop = 1000).
using Priority = std::int32_t;

// ****************************************************************************
/// Condition that must hold before a skill starts.
///
/// @code
/// robotik::Precondition const seen{
///     "red_cube seen",
///     [](robotik::RobotContext const& p_context) {
///         auto const* cube = p_context.world_model.find("red_cube");
///         return cube != nullptr && cube->observed();
///     } };
/// @endcode
// ****************************************************************************
struct Precondition
{
    /// Human-readable text shown when the skill is blocked.
    std::string text;
    /// Returns true when the skill may start. Reads beliefs, not the truth.
    std::function<bool(RobotContext const&)> holds;
};

// ****************************************************************************
/// What a skill needs and how the scheduler should treat it.
///
/// @code
/// robotik::SkillDescription pick{
///     .name = "Pick(red_cube)",
///     .resources = { resources.require("arm"),
///                    resources.require("gripper"),
///                    resources.require("camera", robotik::Access::Shared) },
///     .priority = 100 };
/// @endcode
// ****************************************************************************
struct SkillDescription
{
    /// Unique name, also the behavior tree action name.
    std::string name{};
    /// Resources reserved for the whole run.
    std::vector<ResourceRequirement> resources{};
    /// Higher runs first and may preempt lower.
    Priority priority = 0;
    /// Waits while a resource is busy or a precondition is false, instead of
    /// failing at once.
    bool wait = true;
    /// False forbids preemption by a higher priority skill.
    bool cancellable = true;
    /// Conditions checked before the start.
    std::vector<Precondition> preconditions{};
    /// Id of the robot the skill drives (world entity); empty: the first
    /// robot. The skill gets the @ref RobotContext of that robot.
    std::string robot{};
};

// ****************************************************************************
/// One-shot or continuous robot action.
///
/// Skills read and command the robot through @ref RobotContext only, so the
/// same code drives MuJoCo, hardware or an RL environment. They are executed
/// by the @ref SkillScheduler, which reserves their resources first.
///
/// The life of a run: @ref reset, then @ref tick at every step until it
/// returns @c SUCCESS or @c FAILURE, or @ref cancel if the scheduler stops it.
///
/// @code
/// class WaveSkill final : public robotik::Skill {
/// public:
///     robotik::Status tick(robotik::RobotContext& p_context, Seconds) override
///     {
///         // command joints through p_context.robot.joints() ...
///         return robotik::Status::RUNNING;
///     }
///     void cancel(robotik::RobotContext& p_context) override
///     {
///         p_context.robot.joints().hold(); // stop where we are
///     }
/// };
/// @endcode
// ****************************************************************************
class Skill
{
public:

    virtual ~Skill() = default;

    // -------------------------------------------------------------------------
    /// Clears the run state; called each time the skill starts.
    // -------------------------------------------------------------------------
    virtual void reset()
    {
        /* no-op */
    }

    // -------------------------------------------------------------------------
    /// Advances the skill by one step.
    /// @param p_context Robot API and beliefs of this tick.
    /// @param p_dt Duration of the tick.
    /// @return @ref Status::RUNNING until the goal is met or failed.
    // -------------------------------------------------------------------------
    virtual Status tick(RobotContext& p_context, Seconds p_dt) = 0;

    // -------------------------------------------------------------------------
    /// Cooperative stop (cancel, preemption, lost resource): bring the
    /// commanded hardware to a safe state. Composite skills forward it to
    /// their children. The skill is not ticked again for this run.
    /// @param p_context Robot API and beliefs of this tick.
    // -------------------------------------------------------------------------
    virtual void cancel(RobotContext& /*p_context*/)
    {
        /* no-op */
    }

    // -------------------------------------------------------------------------
    /// Explains the last failure. Shown by the simulator and the planner.
    /// @return Why the last run returned @ref Status::FAILURE, in words for
    ///   the user ("red_cube not seen within 2 s"), or empty when the skill
    ///   does not say.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual std::string why() const
    {
        return {};
    }
};

} // namespace robotik
