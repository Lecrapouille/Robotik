// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Scheduler.hpp
/// Decides which requested skills can run now and runs them.
///
/// A planner or a behavior tree answers "what should be done"; the scheduler
/// answers "what can be done now": it checks resources, priorities and
/// preconditions, waits, preempts, cancels and stops skills whose resources
/// fail. State is stored per skill in parallel arrays indexed by @ref SkillId.
#pragma once

#include "Robotik/Robot/Resources.hpp"
#include "Robotik/Common/Signal.hpp"
#include "Robotik/Task/Skills/Skill.hpp"

#include <concepts>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

namespace robotik
{

/// Index of a skill in its @ref SkillScheduler.
using SkillId = std::uint32_t;
/// No skill (e.g. not found).
inline constexpr SkillId NO_SKILL = 0xFFFFFFFFu;

// ****************************************************************************
/// Life cycle of one skill run.
// ****************************************************************************
enum class SkillState : std::uint8_t
{
    Idle,      ///< Never requested since the last reset.
    Waiting,   ///< Requested, not started (busy resource or precondition).
    Running,   ///< Holding its resources, ticked every update.
    Succeeded, ///< Last run reached its goal.
    Failed,    ///< Last run failed (see @ref SkillReason).
    Cancelled, ///< Last run was cancelled or preempted.
};

// ****************************************************************************
/// Why a skill waits, failed or was cancelled.
// ****************************************************************************
enum class SkillReason : std::uint8_t
{
    None,         ///< Nothing to explain.
    Busy,         ///< A resource is held by another skill.
    Unavailable,  ///< A resource failed before the start.
    Precondition, ///< A precondition is false.
    ResourceLost, ///< A resource failed while running.
    Preempted,    ///< A higher priority skill took a resource.
    Cancelled,    ///< Explicit @ref SkillScheduler::cancel.
    Failed,       ///< The skill itself returned @ref Status::FAILURE.
};

// ----------------------------------------------------------------------------
/// Names a state, for logs and the user interface.
/// @param p_state State to name.
/// @return For example "RUNNING".
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(SkillState p_state);

// ----------------------------------------------------------------------------
/// Names a reason, for logs and the user interface.
/// @param p_reason Reason to name.
/// @return For example "preempted".
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(SkillReason p_reason);

// ****************************************************************************
/// One phase of a skill run, for timelines and logs.
///
/// A skill that waits, then runs, keeps the waiting phase: the next phase is a
/// new entry. A running phase is updated in place until it ends.
///
/// @code
/// for (robotik::SkillRun const& run : scheduler.trace())
///     std::cout << scheduler.name(run.skill) << ' ' <<
///     robotik::toString(run.state)
///               << " [" << run.start.value() << ", " << run.end.value() <<
///               "]\n";
/// @endcode
// ****************************************************************************
struct SkillRun
{
    /// Skill of the phase.
    SkillId skill = NO_SKILL;
    /// State during the phase.
    SkillState state = SkillState::Waiting;
    /// Why it waits, failed or was cancelled.
    SkillReason reason = SkillReason::None;
    /// Resource behind @ref reason, or @ref NO_RESOURCE.
    ResourceId blocker = NO_RESOURCE;
    /// Time of the request (first update after it).
    Seconds start{};
    /// Time of the last update of this run.
    Seconds end{};
};

// ****************************************************************************
/// Owns the skills of a robot and arbitrates their execution.
///
/// @code
/// robotik::SkillScheduler skills(robot.resources());
/// auto const home = skills.add<robotik::HomeSkill>(
///     { .name = "Home", .resources = { robot.resources().require("arm") } });
/// skills.request(home);
/// while (running) {
///     skills.update(context);   // admit, preempt, tick
///     robot.step(dt);
/// }
/// @endcode
// ****************************************************************************
class SkillScheduler
{
public:

    /// @param p_resources Resources the skills reserve. It must outlive the
    ///   scheduler.
    explicit SkillScheduler(ResourceManager& p_resources);
    ~SkillScheduler();

    // -------------------------------------------------------------------------
    /// Registers a skill.
    /// @param p_description Name, resources, priority and preconditions.
    /// @param p_skill The skill, now owned by the scheduler.
    /// @return The id of the skill.
    /// @throws std::invalid_argument on a duplicated name.
    // -------------------------------------------------------------------------
    SkillId add(SkillDescription p_description, std::unique_ptr<Skill> p_skill);

    // -------------------------------------------------------------------------
    /// Registers a skill built in place.
    /// @tparam T Concrete skill type.
    /// @param p_description Name, resources, priority and preconditions.
    /// @param p_args Arguments of the constructor of @p T.
    /// @return The id of the skill.
    /// @throws std::invalid_argument on a duplicated name.
    // -------------------------------------------------------------------------
    template <std::derived_from<Skill> T, typename... Args>
    SkillId add(SkillDescription p_description, Args&&... p_args)
    {
        return add(std::move(p_description),
                   std::make_unique<T>(std::forward<Args>(p_args)...));
    }

    // -------------------------------------------------------------------------
    /// Finds a skill by name.
    /// @param p_name Skill name.
    /// @return Its id, or @ref NO_SKILL.
    // -------------------------------------------------------------------------
    [[nodiscard]] SkillId find(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    /// @return The number of skills.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t size() const
    {
        return m_skills.size();
    }

    // -------------------------------------------------------------------------
    /// Asks for a new run. No effect while waiting or running.
    /// @param p_id Skill to run.
    // -------------------------------------------------------------------------
    void request(SkillId p_id);

    // -------------------------------------------------------------------------
    /// Cancels the run at the next @ref update.
    /// @param p_id Skill to cancel.
    // -------------------------------------------------------------------------
    void cancel(SkillId p_id);

    /// Cancels every run at the next @ref update.
    void cancelAll();

    // -------------------------------------------------------------------------
    /// Stops lost runs, admits requests by priority, ticks the runs.
    /// @param p_context Robot API and beliefs of this tick.
    // -------------------------------------------------------------------------
    void update(RobotContext& p_context);

    // -------------------------------------------------------------------------
    /// Same, for a world of several robots: each skill gets the context of
    /// the robot it drives (@ref SkillDescription::robot, the first context
    /// when empty).
    /// @param p_contexts One context per robot.
    /// @throws std::runtime_error for a skill driving an unknown robot.
    // -------------------------------------------------------------------------
    void update(std::span<RobotContext> p_contexts);

    // -------------------------------------------------------------------------
    /// Forgets every run and releases every resource (episode reset).
    // -------------------------------------------------------------------------
    void reset();

    // -------------------------------------------------------------------------
    /// Emitted when a skill is requested (@c Waiting), starts (@c Running) or
    /// ends (@c Succeeded, @c Failed, @c Cancelled), with the skill and its
    /// new state. Not emitted by @ref reset. A slot must not call the
    /// scheduler back.
    ///
    /// @code
    /// skills.changed.connect([&](robotik::SkillId p_id, robotik::SkillState p_state) {
    ///     if (p_state == robotik::SkillState::Failed)
    ///         std::cerr << skills.name(p_id) << " failed\n";
    /// });
    /// @endcode
    // -------------------------------------------------------------------------
    Signal<SkillId, SkillState> changed;

    // --- Introspection ------------------------------------------------------

    // -------------------------------------------------------------------------
    /// @param p_id Skill id.
    /// @return Where the last run of the skill is.
    // -------------------------------------------------------------------------
    [[nodiscard]] SkillState state(SkillId p_id) const
    {
        return m_states[p_id];
    }

    // -------------------------------------------------------------------------
    /// @param p_id Skill id.
    /// @return Why the skill waits, failed or was cancelled.
    // -------------------------------------------------------------------------
    [[nodiscard]] SkillReason reason(SkillId p_id) const
    {
        return m_reasons[p_id];
    }

    // -------------------------------------------------------------------------
    /// @param p_id Skill id.
    /// @return Resource behind @ref reason, or @ref NO_RESOURCE.
    // -------------------------------------------------------------------------
    [[nodiscard]] ResourceId blocker(SkillId p_id) const
    {
        return m_blockers[p_id];
    }

    // -------------------------------------------------------------------------
    /// @param p_id Skill id.
    /// @return False precondition behind @ref SkillReason::Precondition, or
    ///   null.
    // -------------------------------------------------------------------------
    [[nodiscard]] Precondition const* precondition(SkillId p_id) const;

    // -------------------------------------------------------------------------
    /// @param p_id Skill id.
    /// @return The description given to @ref add.
    // -------------------------------------------------------------------------
    [[nodiscard]] SkillDescription const& description(SkillId p_id) const
    {
        return m_descriptions[p_id];
    }

    // -------------------------------------------------------------------------
    /// @param p_id Skill id.
    /// @return The skill name.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& name(SkillId p_id) const
    {
        return m_descriptions[p_id].name;
    }

    // -------------------------------------------------------------------------
    /// @param p_id Skill id.
    /// @return The skill object.
    // -------------------------------------------------------------------------
    [[nodiscard]] Skill& skill(SkillId p_id) const
    {
        return *m_skills[p_id];
    }

    // -------------------------------------------------------------------------
    /// @return Chronological list of runs since the last @ref reset.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::span<SkillRun const> trace() const
    {
        return m_trace;
    }

private:

    /// Context of the robot skill @p_id drives (during @ref update).
    [[nodiscard]] RobotContext& contextOf(SkillId p_id) const;
    /// Starts a waiting skill whose resources and preconditions are met.
    void admit(SkillId p_id);
    /// Cancels the lower priority holders of @p_conflict for @p_by; false
    /// when one of them cannot be preempted.
    bool preempt(SkillId p_by, Conflict const& p_conflict);
    /// Ends a run, releases its resources and records the phase.
    void finish(SkillId p_id,
                SkillState p_state,
                SkillReason p_reason,
                Seconds p_now);
    /// Cancels a running skill (calls @ref Skill::cancel) and finishes it.
    void stop(SkillId p_id, SkillState p_state, SkillReason p_reason);
    /// Adds or updates the phase of @p_id in @ref m_trace.
    void record(SkillId p_id, Seconds p_now);

private:

    /// Resources the skills reserve.
    ResourceManager& m_resources;

    /// Skills, indexed by @ref SkillId.
    std::vector<std::unique_ptr<Skill>> m_skills;
    /// Description of each skill.
    std::vector<SkillDescription> m_descriptions;
    /// State of each skill.
    std::vector<SkillState> m_states;
    /// Reason of each state.
    std::vector<SkillReason> m_reasons;
    /// Resource behind each reason.
    std::vector<ResourceId> m_blockers;
    /// Index of the false precondition of each waiting skill.
    std::vector<std::uint8_t> m_preconditions;
    /// Cancel asked for each skill (1) or not (0).
    std::vector<std::uint8_t> m_cancels;
    /// Request order of each skill, to admit equal priorities fairly.
    std::vector<std::uint64_t> m_orders;
    /// Index of the current phase of each skill in @ref m_trace, or -1.
    std::vector<std::int32_t> m_runs;
    /// Resources held by each running skill.
    std::vector<ResourceLease> m_leases;

    /// Contexts of the @ref update in progress.
    std::span<RobotContext> m_contexts;
    /// Phases since the last reset.
    std::vector<SkillRun> m_trace;
    /// Skills to admit, sorted by priority then order.
    std::vector<SkillId> m_queue;
    /// Next request order.
    std::uint64_t m_order = 0;
};

} // namespace robotik
