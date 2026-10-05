// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Runtime/Scheduler.hpp"

#include "Robotik/Runtime/RobotContext.hpp"

#include <algorithm>
#include <stdexcept>

namespace robotik
{

char const* toString(SkillState p_state)
{
    switch (p_state)
    {
        case SkillState::Idle:
            return "IDLE";
        case SkillState::Waiting:
            return "WAITING";
        case SkillState::Running:
            return "RUNNING";
        case SkillState::Succeeded:
            return "SUCCEEDED";
        case SkillState::Failed:
            return "FAILED";
        case SkillState::Cancelled:
            return "CANCELLED";
    }
    return "?";
}

char const* toString(SkillReason p_reason)
{
    switch (p_reason)
    {
        case SkillReason::None:
            return "";
        case SkillReason::Busy:
            return "busy";
        case SkillReason::Unavailable:
            return "unavailable";
        case SkillReason::Precondition:
            return "precondition";
        case SkillReason::ResourceLost:
            return "resource lost";
        case SkillReason::Preempted:
            return "preempted";
        case SkillReason::Cancelled:
            return "cancelled";
        case SkillReason::Failed:
            return "failed";
    }
    return "?";
}

SkillScheduler::SkillScheduler(ResourceManager& p_resources)
    : m_resources(p_resources)
{
}

SkillScheduler::~SkillScheduler() = default;

SkillId SkillScheduler::add(SkillDescription p_description,
                            std::unique_ptr<Skill> p_skill)
{
    if (find(p_description.name) != NO_SKILL)
    {
        throw std::invalid_argument("Duplicated skill '" + p_description.name +
                                    "'");
    }
    if (p_description.resources.size() > ResourceLease::CAPACITY)
    {
        throw std::invalid_argument("Skill '" + p_description.name +
                                    "' needs too many resources");
    }
    m_skills.push_back(std::move(p_skill));
    m_descriptions.push_back(std::move(p_description));
    m_states.push_back(SkillState::Idle);
    m_reasons.push_back(SkillReason::None);
    m_blockers.push_back(NO_RESOURCE);
    m_preconditions.push_back(0u);
    m_cancels.push_back(0u);
    m_orders.push_back(0u);
    m_runs.push_back(-1);
    m_leases.emplace_back();
    return static_cast<SkillId>(m_skills.size() - 1u);
}

SkillId SkillScheduler::find(std::string_view p_name) const
{
    for (std::size_t i = 0; i < m_descriptions.size(); ++i)
    {
        if (m_descriptions[i].name == p_name)
        {
            return static_cast<SkillId>(i);
        }
    }
    return NO_SKILL;
}

Precondition const* SkillScheduler::precondition(SkillId p_id) const
{
    if (m_reasons[p_id] != SkillReason::Precondition)
    {
        return nullptr;
    }
    return &m_descriptions[p_id].preconditions[m_preconditions[p_id]];
}

void SkillScheduler::request(SkillId p_id)
{
    if (m_states[p_id] == SkillState::Waiting ||
        m_states[p_id] == SkillState::Running)
    {
        return;
    }
    m_states[p_id] = SkillState::Waiting;
    m_reasons[p_id] = SkillReason::None;
    m_blockers[p_id] = NO_RESOURCE;
    m_cancels[p_id] = 0u;
    m_orders[p_id] = m_order++;
    m_runs[p_id] = -1;
}

void SkillScheduler::cancel(SkillId p_id)
{
    if (m_states[p_id] == SkillState::Waiting ||
        m_states[p_id] == SkillState::Running)
    {
        m_cancels[p_id] = 1u;
    }
}

void SkillScheduler::cancelAll()
{
    for (SkillId id = 0; id < m_skills.size(); ++id)
    {
        cancel(id);
    }
}

void SkillScheduler::reset()
{
    for (SkillId id = 0; id < m_skills.size(); ++id)
    {
        m_leases[id].release();
        m_states[id] = SkillState::Idle;
        m_reasons[id] = SkillReason::None;
        m_blockers[id] = NO_RESOURCE;
        m_cancels[id] = 0u;
        m_runs[id] = -1;
    }
    m_trace.clear();
    m_order = 0;
}

void SkillScheduler::record(SkillId p_id, Seconds p_now)
{
    if (m_runs[p_id] < 0)
    {
        m_runs[p_id] = static_cast<std::int32_t>(m_trace.size());
        m_trace.push_back(
            { p_id, m_states[p_id], m_reasons[p_id], m_blockers[p_id], p_now, p_now });
        return;
    }
    SkillRun& run = m_trace[static_cast<std::size_t>(m_runs[p_id])];
    run.state = m_states[p_id];
    run.reason = m_reasons[p_id];
    run.blocker = m_blockers[p_id];
    run.end = p_now;
}

void SkillScheduler::finish(SkillId p_id,
                            SkillState p_state,
                            SkillReason p_reason,
                            Seconds p_now)
{
    m_leases[p_id].release();
    m_states[p_id] = p_state;
    m_reasons[p_id] = p_reason;
    m_cancels[p_id] = 0u;
    record(p_id, p_now);
}

void SkillScheduler::stop(SkillId p_id,
                          SkillState p_state,
                          SkillReason p_reason,
                          RobotContext& p_context)
{
    if (m_states[p_id] == SkillState::Running)
    {
        m_skills[p_id]->cancel(p_context);
    }
    finish(p_id, p_state, p_reason, p_context.time);
}

bool SkillScheduler::preempt(SkillId p_by,
                             Conflict const& p_conflict,
                             RobotContext& p_context)
{
    Priority const priority = m_descriptions[p_by].priority;
    auto preemptible = [&](SkillId p_victim)
    {
        return p_victim < m_skills.size() && p_victim != p_by &&
               m_states[p_victim] == SkillState::Running &&
               m_descriptions[p_victim].cancellable &&
               m_descriptions[p_victim].priority < priority;
    };

    // Exclusive holder.
    if (p_conflict.owner != NO_OWNER)
    {
        if (!preemptible(p_conflict.owner))
        {
            return false;
        }
        m_blockers[p_conflict.owner] = p_conflict.resource;
        stop(p_conflict.owner, SkillState::Cancelled, SkillReason::Preempted, p_context);
        return true;
    }

    // Shared holders blocking an exclusive request: all must be preemptible.
    std::vector<SkillId> victims;
    for (SkillId id = 0; id < m_skills.size(); ++id)
    {
        for (ResourceRequirement const& held : m_leases[id].resources())
        {
            if (held.id == p_conflict.resource)
            {
                if (!preemptible(id))
                {
                    return false;
                }
                victims.push_back(id);
            }
        }
    }
    for (SkillId const victim : victims)
    {
        m_blockers[victim] = p_conflict.resource;
        stop(victim, SkillState::Cancelled, SkillReason::Preempted, p_context);
    }
    return !victims.empty();
}

void SkillScheduler::admit(SkillId p_id, RobotContext& p_context)
{
    SkillDescription const& description = m_descriptions[p_id];
    Seconds const now = p_context.time;

    for (ResourceRequirement const& requirement : description.resources)
    {
        if (!m_resources.available(requirement.id))
        {
            m_blockers[p_id] = requirement.id;
            finish(p_id, SkillState::Failed, SkillReason::Unavailable, now);
            return;
        }
    }

    for (std::size_t i = 0; i < description.preconditions.size(); ++i)
    {
        Precondition const& precondition = description.preconditions[i];
        if (precondition.holds && !precondition.holds(p_context))
        {
            m_preconditions[p_id] = static_cast<std::uint8_t>(i);
            m_blockers[p_id] = NO_RESOURCE;
            if (description.wait)
            {
                m_reasons[p_id] = SkillReason::Precondition;
                record(p_id, now);
            }
            else
            {
                finish(p_id, SkillState::Failed, SkillReason::Precondition, now);
            }
            return;
        }
    }

    // Each preemption frees at least one resource: bounded retries.
    Conflict conflict;
    for (std::size_t attempt = 0; attempt <= description.resources.size(); ++attempt)
    {
        conflict = m_resources.conflict(description.resources);
        if (!conflict)
        {
            m_leases[p_id] = m_resources.acquire(description.resources, p_id);
            m_states[p_id] = SkillState::Running;
            m_reasons[p_id] = SkillReason::None;
            m_blockers[p_id] = NO_RESOURCE;
            m_skills[p_id]->reset();
            record(p_id, now);
            return;
        }
        if (!preempt(p_id, conflict, p_context))
        {
            break;
        }
    }

    m_blockers[p_id] = conflict.resource;
    if (description.wait)
    {
        m_reasons[p_id] = SkillReason::Busy;
        record(p_id, now);
    }
    else
    {
        finish(p_id, SkillState::Failed, SkillReason::Busy, now);
    }
}

void SkillScheduler::update(RobotContext& p_context)
{
    // Explicit cancellations.
    for (SkillId id = 0; id < m_skills.size(); ++id)
    {
        if (m_cancels[id] != 0u)
        {
            stop(id, SkillState::Cancelled, SkillReason::Cancelled, p_context);
        }
    }

    // Runs whose resources failed.
    for (SkillId id = 0; id < m_skills.size(); ++id)
    {
        if (m_states[id] != SkillState::Running)
        {
            continue;
        }
        for (ResourceRequirement const& requirement : m_descriptions[id].resources)
        {
            if (!m_resources.available(requirement.id))
            {
                m_blockers[id] = requirement.id;
                stop(id, SkillState::Failed, SkillReason::ResourceLost, p_context);
                break;
            }
        }
    }

    // Admission by priority, then by request order.
    auto const by_priority = [this](SkillId p_a, SkillId p_b)
    {
        Priority const a = m_descriptions[p_a].priority;
        Priority const b = m_descriptions[p_b].priority;
        return a != b ? a > b : m_orders[p_a] < m_orders[p_b];
    };
    m_queue.clear();
    for (SkillId id = 0; id < m_skills.size(); ++id)
    {
        if (m_states[id] == SkillState::Waiting)
        {
            m_queue.push_back(id);
        }
    }
    std::sort(m_queue.begin(), m_queue.end(), by_priority);
    for (SkillId const id : m_queue)
    {
        if (m_states[id] == SkillState::Waiting)
        {
            admit(id, p_context);
        }
    }

    // Tick the runs, highest priority first.
    m_queue.clear();
    for (SkillId id = 0; id < m_skills.size(); ++id)
    {
        if (m_states[id] == SkillState::Running)
        {
            m_queue.push_back(id);
        }
    }
    std::sort(m_queue.begin(), m_queue.end(), by_priority);
    for (SkillId const id : m_queue)
    {
        if (m_states[id] != SkillState::Running)
        {
            continue;
        }
        Status const status = m_skills[id]->tick(p_context, p_context.dt);
        if (status == Status::SUCCESS)
        {
            finish(id, SkillState::Succeeded, SkillReason::None, p_context.time);
        }
        else if (status == Status::FAILURE)
        {
            finish(id, SkillState::Failed, SkillReason::Failed, p_context.time);
        }
        else
        {
            record(id, p_context.time);
        }
    }
}

} // namespace robotik
