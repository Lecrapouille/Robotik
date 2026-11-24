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
/// fail. State is stored per skill in parallel arrays indexed by @ref
/// robotik::SkillId.
///
/// @par Contract
/// - Promises: a resource is held by one running skill at most; a skill whose
///   resource fails stops at once with @c ResourceLost. Each request gets its
///   own ticket (@ref robotik::RunId); a skill has one run at a time, the
///   other requests wait in a queue. A preemption stops all the holders it
///   needs or none. A cancelled or preempted run keeps its resources while
///   it is @c Stopping (up to @c stop_timeout); @c reset, @c cancelAll() and
///   an urgent skill do not wait. A preempted run that may wait starts again
///   later. While a skill is called, it is the writer of the resources
///   (@ref robotik::WriterScope). @c changed is emitted at each change of
///   state (waiting, running, stopping, finished).
/// - Expects: one @c update per step; skill names are unique.
/// - Errors: a duplicated name throws @c std::invalid_argument; a skill driving
///   an unknown robot throws @c std::runtime_error.
#pragma once

#include "RobotIK/Robot/Resources.hpp"
#include "RobotIK/Common/Signal.hpp"
#include "RobotIK/Task/Skills/Skill.hpp"

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
inline constexpr SkillId NO_SKILL = 0xFFFFFFFFU;

// ****************************************************************************
/// Ticket of one request: which skill, and which of its runs.
///
/// Two callers that ask for the same skill get two tickets. Each one reads
/// the state of its own run and cancels only its own run, so a behavior
/// tree, a plan and a button of the interface do not cancel each other.
/// The generation goes on counting after a reset: an old ticket never names
/// a new run.
///
/// @code
/// robotik::RunId const run = skills.request(home);
/// skills.update(context);
/// if (skills.state(run) == robotik::SkillState::Succeeded) { ... }
/// @endcode
// ****************************************************************************
struct RunId
{
    /// Skill asked for, or @ref NO_SKILL.
    SkillId skill = NO_SKILL;
    /// Number of the request for this skill (1 for the first one).
    std::uint32_t generation = 0U;

    /// Two tickets are equal when they name the same run.
    friend bool operator==(RunId const&, RunId const&) = default;
};

/// No run (a ticket not yet asked for).
inline constexpr RunId NO_RUN{};

// ****************************************************************************
/// Life cycle of one skill run.
// ****************************************************************************
enum class SkillState : std::uint8_t
{
    Idle,      ///< Never requested since the last reset.
    Waiting,   ///< Requested, not started (busy resource or precondition).
    Running,   ///< Holding its resources, ticked every update.
    Stopping,  ///< Cancelled or preempted, still holding its resources until
               ///< @ref Skill::stopping says the hardware is safe.
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
    Queued,       ///< Another run of the same skill is in progress.
    Preempting,   ///< Waits for the skills it preempted to stop.
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
    /// True when the stop of the run reached @ref SkillDescription::stop_timeout
    /// and was forced.
    bool forced = false;
};

// ****************************************************************************
/// How a run ended, kept to answer @ref SkillScheduler::state(RunId) const.
// ****************************************************************************
struct RunEnd
{
    /// Run of the skill.
    std::uint32_t generation = 0U;
    /// End state.
    SkillState state = SkillState::Idle;
    /// Why it ended so.
    SkillReason reason = SkillReason::None;
};

// ****************************************************************************
/// Owns the skills of a robot and arbitrates their execution.
///
/// A behavior tree, a plan or a test asks for a skill; it does not know if
/// the arm is free or if a more urgent skill must run first. The scheduler
/// answers that, the same way for every caller: a request waits while a
/// resource it needs is busy or a precondition is false, a higher priority
/// skill cancels a lower one that is @c cancellable, and a skill whose
/// resource fails stops with @c ResourceLost. So two callers never drive the
/// same joints at once, and a fault never leaves a skill running blind.
///
/// Call @ref update once per step: it stops the lost runs, admits the
/// requests by priority, then ticks the running skills. A state change
/// emits @ref changed; @ref trace keeps the phases for timelines and
/// reports. The scheduler does not choose what to do next (the tree or the
/// plan does), and it does not retry a failed skill. Its state is stored in
/// arrays indexed by @ref SkillId, found once by name with @ref find.
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
    /// @throws std::invalid_argument on a duplicated name, an unknown
    ///   resource, or more than @ref ResourceLease::CAPACITY resources
    ///   (duplicates are merged first, see @ref normalize).
    // -------------------------------------------------------------------------
    SkillId add(SkillDescription p_description, std::unique_ptr<Skill> p_skill);

    // -------------------------------------------------------------------------
    /// Registers a skill built in place.
    /// @tparam T Concrete skill type.
    /// @param p_description Name, resources, priority and preconditions.
    /// @param p_args Arguments of the constructor of @p T.
    /// @return The id of the skill.
    /// @throws std::invalid_argument on a duplicated name, an unknown
    ///   resource, or more than @ref ResourceLease::CAPACITY resources
    ///   (duplicates are merged first, see @ref normalize).
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
    /// Asks for a new run. A skill has one run at a time: while one waits or
    /// runs, the new one is queued (state @c Waiting, reason @c Queued) and
    /// becomes the run of the skill when the previous one ends, in the
    /// order of the requests.
    /// @param p_id Skill to run.
    /// @return The ticket of this run.
    // -------------------------------------------------------------------------
    RunId request(SkillId p_id);

    // -------------------------------------------------------------------------
    /// Cancels one run: a queued run is removed at once (it ends
    /// @c Cancelled), the current run is cancelled at the next @ref update.
    /// No effect on a run that ended, nor on the other runs of the skill.
    /// @param p_run Ticket given by @ref request.
    // -------------------------------------------------------------------------
    void cancel(RunId p_run);

    // -------------------------------------------------------------------------
    /// Cancels every run of a skill, whoever asked for it (emergency button
    /// of the interface).
    /// @param p_id Skill to cancel.
    // -------------------------------------------------------------------------
    void cancelAll(SkillId p_id);

    /// Cancels every run of every skill at the next @ref update, at once:
    /// the running skills get @ref Skill::cancel but no @c Stopping phase
    /// (a new plan, an emergency).
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
    /// @return Where the current or last run of the skill is (for the
    ///   interface; a caller reads its own run with @ref state(RunId) const).
    // -------------------------------------------------------------------------
    [[nodiscard]] SkillState state(SkillId p_id) const
    {
        return m_states[p_id];
    }

    // -------------------------------------------------------------------------
    /// @param p_run Ticket given by @ref request.
    /// @return Where this run is: @c Waiting while queued, its end state
    ///   once ended, @c Idle for a ticket this scheduler does not know
    ///   (given before the last @ref reset).
    // -------------------------------------------------------------------------
    [[nodiscard]] SkillState state(RunId p_run) const;

    // -------------------------------------------------------------------------
    /// @param p_id Skill id.
    /// @return Why the skill waits, failed or was cancelled.
    // -------------------------------------------------------------------------
    [[nodiscard]] SkillReason reason(SkillId p_id) const
    {
        return m_reasons[p_id];
    }

    // -------------------------------------------------------------------------
    /// @param p_run Ticket given by @ref request.
    /// @return Why this run waits (@c Queued while queued), failed or was
    ///   cancelled.
    // -------------------------------------------------------------------------
    [[nodiscard]] SkillReason reason(RunId p_run) const;

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
    /// Finds in @ref m_victims every holder that blocks @p_by, without
    /// stopping any. False, with @p_blocker set, when one of them cannot be
    /// preempted (higher priority, not cancellable, or outside the
    /// scheduler).
    bool victims(SkillId p_by, ResourceId& p_blocker);
    /// Ends a run, releases its resources and records the phase.
    void finish(SkillId p_id,
                SkillState p_state,
                SkillReason p_reason,
                Seconds p_now);
    /// Cancels a running skill (calls @ref Skill::cancel). With
    /// @p p_at_once, or when @ref Skill::stopping succeeds at once, the run
    /// ends now; else it is @c Stopping and keeps its resources.
    void stop(SkillId p_id,
              SkillState p_state,
              SkillReason p_reason,
              bool p_at_once);
    /// Asks a stopping skill if it is safe; ends the run when it is, or when
    /// its stop timeout is over.
    void stopping(SkillId p_id);
    /// Adds or updates the phase of @p_id in @ref m_trace.
    void record(SkillId p_id, Seconds p_now);
    /// Makes the first queued run of @p_id its current run (state Waiting).
    void dequeue(SkillId p_id);
    /// @return The end of run @p_run, or null when it did not end.
    [[nodiscard]] RunEnd const* ended(RunId p_run) const;

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
    /// Cancel asked for each skill: 0 none, 1 with a Stopping phase, 2 at
    /// once.
    std::vector<std::uint8_t> m_cancels;
    /// End state of each stopping run.
    std::vector<SkillState> m_stop_states;
    /// End reason of each stopping run.
    std::vector<SkillReason> m_stop_reasons;
    /// Time each stopping run started to stop.
    std::vector<Seconds> m_stop_since;
    /// Request order of each skill, to admit equal priorities fairly.
    std::vector<std::uint64_t> m_orders;
    /// Index of the current phase of each skill in @ref m_trace, or -1.
    std::vector<std::int32_t> m_runs;
    /// Resources held by each running skill.
    std::vector<ResourceLease> m_leases;
    /// Generation of the current or last run of each skill.
    std::vector<std::uint32_t> m_generations;
    /// Generation of the last request of each skill (not reset).
    std::vector<std::uint32_t> m_requested;
    /// Queued runs of each skill, oldest first.
    std::vector<std::vector<std::uint32_t>> m_queued;
    /// Ended runs of each skill since the last reset.
    std::vector<std::vector<RunEnd>> m_ended;

    /// Contexts of the @ref update in progress.
    std::span<RobotContext> m_contexts;
    /// Phases since the last reset.
    std::vector<SkillRun> m_trace;
    /// Skills to admit, sorted by priority then order.
    std::vector<SkillId> m_queue;
    /// A skill to preempt, and the resource it is preempted for.
    struct Victim
    {
        SkillId skill;       ///< Running skill to stop.
        ResourceId resource; ///< Resource wanted by the new skill.
    };
    /// Skills to preempt for the admission in progress.
    std::vector<Victim> m_victims;
    /// Next request order.
    std::uint64_t m_order = 0;
};

} // namespace robotik
