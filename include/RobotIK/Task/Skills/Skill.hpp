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
///   stops the skill; @c failure tells a tree (its code) and a person (its
///   words) why the run failed.
/// - Expects: a skill reads only its @ref robotik::RobotContext; it declares
///   the resources it needs in its @ref robotik::SkillDescription.
#pragma once

#include "RobotIK/Robot/Resources.hpp"
#include "RobotIK/Task/Status.hpp"

#include "RobotIK/Common/Math.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace robotik
{

struct RobotContext;

/// Higher runs first and may preempt lower (e.g. stop = 1000).
using Priority = std::int32_t;

// ****************************************************************************
/// Why a run of a skill failed, for a tree and for a person.
///
/// A behavior tree reacts to the @c code: it reads @c "<skill>.failure" on
/// its blackboard, so a grasp not confirmed can lead to a new approach, and
/// an object not seen to a new detection. The @c why is for the trace, the
/// simulator and the planner history. The plugin chooses its codes: the
/// core does not know the tools (a vacuum cup, a brush). The core motion
/// skills use @c no_joints, @c unknown_joint and @c no_ik_solution.
///
/// @code
/// return fail("grasp_not_confirmed", "suction holds nothing after 2 s");
/// @endcode
// ****************************************************************************
struct SkillFailure
{
    /// Short, stable name of the failure, in lower case with underscores
    /// (@c grasp_not_confirmed); empty when the skill gave none.
    std::string code;
    /// The same failure in words for a person ("red_cube not seen within
    /// 2 s").
    std::string why;
};

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
/// The scheduler reads this card, never the code of the skill: the
/// resources it holds for the whole run, its priority, whether it waits or
/// fails when it cannot start, whether a higher priority may cancel it, and
/// the robot it drives. It is plain data, so a scenario logic can fill it from a
/// scenario parameter.
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
    /// failing at once. A preempted run also waits for its resources again
    /// and starts again from @ref Skill::reset; false: it ends
    /// @c Cancelled (reason @c Preempted).
    bool wait = true;
    /// False forbids preemption by a higher priority skill.
    bool cancellable = true;
    /// True for an emergency skill (Stop): the skills it preempts end at
    /// once, without their @ref Skill::stopping phase, so it never waits.
    bool urgent = false;
    /// Longest @ref Skill::stopping phase; after it the run ends anyway and
    /// the trace says the stop was forced.
    Seconds stop_timeout{ 1.0 };
    /// Conditions checked before the start.
    std::vector<Precondition> preconditions{};
    /// Id of the robot the skill drives (world entity); empty: the first
    /// robot. The skill gets the @ref RobotContext of that robot.
    std::string robot{};
};

// ****************************************************************************
/// One-shot or continuous robot action.
///
/// A skill is the smallest action that a tree, a plan or a test can ask for
/// by name: "move to this pose", "grasp", "follow the line". It knows how to
/// do one thing and nothing about the job around it.
///
/// Skills read and command the robot through @ref RobotContext only, so the
/// same code drives MuJoCo, hardware or an RL environment. They are executed
/// by the @ref SkillScheduler, which reserves their resources first: a skill
/// never checks if the arm is free.
///
/// The life of a run: @ref reset, then @ref tick at every step until it
/// returns @c SUCCESS or @c FAILURE, or @ref cancel if the scheduler stops it
/// (preemption, lost resource, a tree that moves on), then @ref stopping
/// until the hardware is safe. @ref cancel must leave the hardware safe:
/// hold the joints, open the gripper. A tick commands a little and returns;
/// it never loops until the goal, because the other skills and the physics
/// run between two ticks.
///
/// A preempted run starts again from @ref reset when its resources come
/// back (@ref SkillDescription::wait). So @ref reset must clear everything a
/// run keeps, and @ref tick must read where the robot and the objects are
/// now (joints, beliefs), not where they were when the run started.
///
/// A tick that fails returns @ref fail with a code and words: the tree can
/// then choose its reaction from the code (@ref SkillFailure). A composite
/// skill passes on the failure of its child with @c fail(child.failure()).
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
    /// Called after @ref cancel, at once then at every update, while the
    /// run is @c Stopping: the skill keeps its resources until the hardware
    /// is safe (an arm slowing down, a gripper that opens). Not called after
    /// a lost resource, a reset or an urgent preemption: the run ends at
    /// once there.
    /// @param p_context Robot API and beliefs of this tick.
    /// @param p_dt Duration of the tick.
    /// @return @ref Status::RUNNING while stopping; anything else ends the
    ///   run. The default stops at once.
    // -------------------------------------------------------------------------
    virtual Status stopping(RobotContext& /*p_context*/, Seconds /*p_dt*/)
    {
        return Status::SUCCESS;
    }

    // -------------------------------------------------------------------------
    /// @return Why the current run returned @ref Status::FAILURE; empty
    ///   while it has not failed, or when the skill gave no reason. The
    ///   scheduler clears it when a run starts.
    // -------------------------------------------------------------------------
    [[nodiscard]] SkillFailure const& failure() const
    {
        return m_failure;
    }

protected:

    // -------------------------------------------------------------------------
    /// Records why the run fails; return its result from @ref tick.
    /// @param p_code Short name of the failure (@ref SkillFailure::code).
    /// @param p_why The failure in words for a person.
    /// @return @ref Status::FAILURE.
    // -------------------------------------------------------------------------
    Status fail(std::string p_code, std::string p_why)
    {
        m_failure = { std::move(p_code), std::move(p_why) };
        return Status::FAILURE;
    }

    // -------------------------------------------------------------------------
    /// Records the failure of a child skill as the failure of this run.
    /// @param p_failure The failure of the child.
    /// @return @ref Status::FAILURE.
    // -------------------------------------------------------------------------
    Status fail(SkillFailure p_failure)
    {
        m_failure = std::move(p_failure);
        return Status::FAILURE;
    }

private:

    /// The scheduler clears the failure when a run starts.
    friend class SkillScheduler;

    /// Why the current run failed.
    SkillFailure m_failure;
};

} // namespace robotik
