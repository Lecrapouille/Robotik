// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Scheduler.hpp
//! @brief Decides which requested skills can run now and runs them.
//!
//! A planner or a behavior tree answers "what should be done"; the scheduler
//! answers "what can be done now": it checks resources, priorities and
//! preconditions, waits, preempts, cancels and stops skills whose resources
//! fail. State is stored per skill in parallel arrays indexed by @ref SkillId.
#pragma once

#include "Robotik/Runtime/Resources.hpp"
#include "Robotik/Skills/Skill.hpp"

#include <concepts>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

namespace robotik
{

using SkillId = std::uint32_t;
inline constexpr SkillId NO_SKILL = 0xFFFFFFFFu;

// ****************************************************************************
//! @brief Life cycle of one skill run.
// ****************************************************************************
enum class SkillState : std::uint8_t
{
    Idle,      //!< Never requested since the last reset.
    Waiting,   //!< Requested, not started (busy resource or precondition).
    Running,   //!< Holding its resources, ticked every update.
    Succeeded, //!< Last run reached its goal.
    Failed,    //!< Last run failed (see @ref SkillReason).
    Cancelled, //!< Last run was cancelled or preempted.
};

// ****************************************************************************
//! @brief Why a skill waits, failed or was cancelled.
// ****************************************************************************
enum class SkillReason : std::uint8_t
{
    None,
    Busy,         //!< A resource is held by another skill.
    Unavailable,  //!< A resource failed before the start.
    Precondition, //!< A precondition is false.
    ResourceLost, //!< A resource failed while running.
    Preempted,    //!< A higher priority skill took a resource.
    Cancelled,    //!< Explicit @ref SkillScheduler::cancel.
    Failed,       //!< The skill itself returned @ref Status::FAILURE.
};

[[nodiscard]] char const* toString(SkillState p_state);
[[nodiscard]] char const* toString(SkillReason p_reason);

// ****************************************************************************
//! @brief One phase of a skill run, for timelines and logs.
//!
//! A skill that waits, then runs, keeps the waiting phase: the next phase is a
//! new entry. A running phase is updated in place until it ends.
// ****************************************************************************
struct SkillRun
{
    SkillId skill = NO_SKILL;
    SkillState state = SkillState::Waiting;
    SkillReason reason = SkillReason::None;
    //!< Resource behind @ref reason, or @ref NO_RESOURCE.
    ResourceId blocker = NO_RESOURCE;
    //!< Time of the request (first update after it).
    Seconds start{};
    //!< Time of the last update of this run.
    Seconds end{};
};

// ****************************************************************************
//! @brief Owns the skills of a robot and arbitrates their execution.
//!
//! @code
//! robotik::SkillScheduler skills(robot.resources());
//! auto const home = skills.add<robotik::HomeSkill>(
//!     { .name = "Home", .resources = { robot.resources().require("arm") } });
//! skills.request(home);
//! while (running) {
//!     skills.update(context);   // admit, preempt, tick
//!     robot.step(dt);
//! }
//! @endcode
// ****************************************************************************
class SkillScheduler
{
public:

    explicit SkillScheduler(ResourceManager& p_resources);
    ~SkillScheduler();

    // -------------------------------------------------------------------------
    //! @brief Registers a skill.
    //! @throws std::invalid_argument on a duplicated name.
    // -------------------------------------------------------------------------
    SkillId add(SkillDescription p_description, std::unique_ptr<Skill> p_skill);

    template <std::derived_from<Skill> T, typename... Args>
    SkillId add(SkillDescription p_description, Args&&... p_args)
    {
        return add(std::move(p_description),
                   std::make_unique<T>(std::forward<Args>(p_args)...));
    }

    //! @brief Id of the skill named @p_name, or @ref NO_SKILL.
    [[nodiscard]] SkillId find(std::string_view p_name) const;

    [[nodiscard]] std::size_t size() const
    {
        return m_skills.size();
    }

    // -------------------------------------------------------------------------
    //! @brief Asks for a new run. No effect while waiting or running.
    // -------------------------------------------------------------------------
    void request(SkillId p_id);

    // -------------------------------------------------------------------------
    //! @brief Cancels the run at the next @ref update.
    // -------------------------------------------------------------------------
    void cancel(SkillId p_id);
    void cancelAll();

    // -------------------------------------------------------------------------
    //! @brief Stops lost runs, admits requests by priority, ticks the runs.
    // -------------------------------------------------------------------------
    void update(RobotContext& p_context);

    // -------------------------------------------------------------------------
    //! @brief Forgets every run and releases every resource (episode reset).
    // -------------------------------------------------------------------------
    void reset();

    // --- Introspection ------------------------------------------------------

    [[nodiscard]] SkillState state(SkillId p_id) const
    {
        return m_states[p_id];
    }

    [[nodiscard]] SkillReason reason(SkillId p_id) const
    {
        return m_reasons[p_id];
    }

    //! @brief Resource behind @ref reason, or @ref NO_RESOURCE.
    [[nodiscard]] ResourceId blocker(SkillId p_id) const
    {
        return m_blockers[p_id];
    }

    //! @brief False precondition behind @ref SkillReason::Precondition.
    [[nodiscard]] Precondition const* precondition(SkillId p_id) const;

    [[nodiscard]] SkillDescription const& description(SkillId p_id) const
    {
        return m_descriptions[p_id];
    }

    [[nodiscard]] std::string const& name(SkillId p_id) const
    {
        return m_descriptions[p_id].name;
    }

    [[nodiscard]] Skill& skill(SkillId p_id) const
    {
        return *m_skills[p_id];
    }

    //! @brief Chronological list of runs since the last @ref reset.
    [[nodiscard]] std::span<SkillRun const> trace() const
    {
        return m_trace;
    }

private:

    void admit(SkillId p_id, RobotContext& p_context);
    bool preempt(SkillId p_by, Conflict const& p_conflict, RobotContext& p_context);
    void finish(SkillId p_id, SkillState p_state, SkillReason p_reason, Seconds p_now);
    void stop(SkillId p_id,
              SkillState p_state,
              SkillReason p_reason,
              RobotContext& p_context);
    void record(SkillId p_id, Seconds p_now);

private:

    ResourceManager& m_resources;

    std::vector<std::unique_ptr<Skill>> m_skills;
    std::vector<SkillDescription> m_descriptions;
    std::vector<SkillState> m_states;
    std::vector<SkillReason> m_reasons;
    std::vector<ResourceId> m_blockers;
    std::vector<std::uint8_t> m_preconditions;
    std::vector<std::uint8_t> m_cancels;
    std::vector<std::uint64_t> m_orders;
    std::vector<std::int32_t> m_runs;
    std::vector<ResourceLease> m_leases;

    std::vector<SkillRun> m_trace;
    std::vector<SkillId> m_queue;
    std::uint64_t m_order = 0;
};

} // namespace robotik
