// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Task/Scheduler.hpp"

#include "RobotIK/Robot/Robot.hpp"
#include "RobotIK/Task/RobotContext.hpp"

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
        case SkillState::Stopping:
            return "STOPPING";
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
        case SkillReason::Queued:
            return "queued";
        case SkillReason::Preempting:
            return "preempting";
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
    p_description.resources = normalize(p_description.resources);
    if (p_description.resources.size() > ResourceLease::CAPACITY)
    {
        throw std::invalid_argument("Skill '" + p_description.name +
                                    "' needs too many resources");
    }
    for (ResourceRequirement const& requirement : p_description.resources)
    {
        if (requirement.id >= m_resources.size())
        {
            throw std::invalid_argument("Skill '" + p_description.name +
                                        "' needs an unknown resource");
        }
    }
    m_skills.push_back(std::move(p_skill));
    m_descriptions.push_back(std::move(p_description));
    m_states.push_back(SkillState::Idle);
    m_reasons.push_back(SkillReason::None);
    m_blockers.push_back(NO_RESOURCE);
    m_preconditions.push_back(0U);
    m_cancels.push_back(0U);
    m_stop_states.push_back(SkillState::Cancelled);
    m_stop_reasons.push_back(SkillReason::Cancelled);
    m_stop_since.push_back(Seconds{});
    m_orders.push_back(0U);
    m_runs.push_back(-1);
    m_leases.emplace_back();
    m_generations.push_back(0U);
    m_requested.push_back(0U);
    m_queued.emplace_back();
    m_ended.emplace_back();
    return static_cast<SkillId>(m_skills.size() - 1U);
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

RunId SkillScheduler::request(SkillId p_id)
{
    std::uint32_t const generation = ++m_requested[p_id];
    if (m_states[p_id] == SkillState::Waiting ||
        m_states[p_id] == SkillState::Running ||
        m_states[p_id] == SkillState::Stopping)
    {
        // One run at a time: this one waits for the end of the current one.
        m_queued[p_id].push_back(generation);
        return { p_id, generation };
    }
    m_generations[p_id] = generation;
    m_states[p_id] = SkillState::Waiting;
    m_reasons[p_id] = SkillReason::None;
    m_blockers[p_id] = NO_RESOURCE;
    m_cancels[p_id] = 0U;
    m_orders[p_id] = m_order++;
    m_runs[p_id] = -1;
    changed(p_id, SkillState::Waiting);
    return { p_id, generation };
}

void SkillScheduler::dequeue(SkillId p_id)
{
    std::vector<std::uint32_t>& queued = m_queued[p_id];
    if (queued.empty())
    {
        return;
    }
    std::uint32_t const generation = queued.front();
    queued.erase(queued.begin());
    m_generations[p_id] = generation;
    m_states[p_id] = SkillState::Waiting;
    m_reasons[p_id] = SkillReason::None;
    m_blockers[p_id] = NO_RESOURCE;
    m_cancels[p_id] = 0U;
    m_orders[p_id] = m_order++;
    m_runs[p_id] = -1;
    changed(p_id, SkillState::Waiting);
}

RunEnd const* SkillScheduler::ended(RunId p_run) const
{
    for (RunEnd const& end : m_ended[p_run.skill])
    {
        if (end.generation == p_run.generation)
        {
            return &end;
        }
    }
    return nullptr;
}

SkillState SkillScheduler::state(RunId p_run) const
{
    if (p_run.skill >= m_skills.size() || p_run.generation == 0U)
    {
        return SkillState::Idle;
    }
    if (p_run.generation == m_generations[p_run.skill])
    {
        return m_states[p_run.skill];
    }
    if (std::ranges::find(m_queued[p_run.skill], p_run.generation) !=
        m_queued[p_run.skill].end())
    {
        return SkillState::Waiting;
    }
    RunEnd const* end = ended(p_run);
    return end != nullptr ? end->state : SkillState::Idle;
}

SkillReason SkillScheduler::reason(RunId p_run) const
{
    if (p_run.skill >= m_skills.size() || p_run.generation == 0U)
    {
        return SkillReason::None;
    }
    if (p_run.generation == m_generations[p_run.skill])
    {
        return m_reasons[p_run.skill];
    }
    if (std::ranges::find(m_queued[p_run.skill], p_run.generation) !=
        m_queued[p_run.skill].end())
    {
        return SkillReason::Queued;
    }
    RunEnd const* end = ended(p_run);
    return end != nullptr ? end->reason : SkillReason::None;
}

SkillFailure const& SkillScheduler::failure(RunId p_run) const
{
    static SkillFailure const NONE{};
    if (p_run.skill >= m_skills.size() || p_run.generation == 0U)
    {
        return NONE;
    }
    if (p_run.generation == m_generations[p_run.skill])
    {
        return m_reasons[p_run.skill] == SkillReason::Failed
                   ? m_skills[p_run.skill]->failure()
                   : NONE;
    }
    RunEnd const* end = ended(p_run);
    return end != nullptr ? end->failure : NONE;
}

void SkillScheduler::cancel(RunId p_run)
{
    if (p_run.skill >= m_skills.size())
    {
        return;
    }
    std::vector<std::uint32_t>& queued = m_queued[p_run.skill];
    if (auto const it = std::ranges::find(queued, p_run.generation);
        it != queued.end())
    {
        queued.erase(it);
        m_ended[p_run.skill].push_back(
            { p_run.generation, SkillState::Cancelled, SkillReason::Cancelled });
        return;
    }
    SkillState const state = m_states[p_run.skill];
    if (p_run.generation == m_generations[p_run.skill] &&
        (state == SkillState::Waiting || state == SkillState::Running))
    {
        m_cancels[p_run.skill] = 1U;
    }
}

void SkillScheduler::cancelAll(SkillId p_id)
{
    for (std::uint32_t const generation : m_queued[p_id])
    {
        m_ended[p_id].push_back(
            { generation, SkillState::Cancelled, SkillReason::Cancelled });
    }
    m_queued[p_id].clear();
    if (m_states[p_id] == SkillState::Waiting ||
        m_states[p_id] == SkillState::Running ||
        m_states[p_id] == SkillState::Stopping)
    {
        m_cancels[p_id] = std::max<std::uint8_t>(m_cancels[p_id], 1U);
    }
}

void SkillScheduler::cancelAll()
{
    for (SkillId id = 0; id < m_skills.size(); ++id)
    {
        cancelAll(id);
        if (m_cancels[id] != 0U)
        {
            m_cancels[id] = 2U; // at once
        }
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
        m_cancels[id] = 0U;
        m_runs[id] = -1;
        // The request counter goes on: an old ticket never names a new run.
        m_generations[id] = 0U;
        m_queued[id].clear();
        m_ended[id].clear();
    }
    m_trace.clear();
    m_order = 0;
}

void SkillScheduler::record(SkillId p_id, Seconds p_now)
{
    if (m_runs[p_id] < 0)
    {
        m_runs[p_id] = static_cast<std::int32_t>(m_trace.size());
        m_trace.push_back({ p_id,
                            m_states[p_id],
                            m_reasons[p_id],
                            m_blockers[p_id],
                            p_now,
                            p_now });
        return;
    }
    SkillRun& run = m_trace[static_cast<std::size_t>(m_runs[p_id])];
    // Keep the waiting interval on the timeline. The next phase starts here.
    if (run.state == SkillState::Waiting &&
        m_states[p_id] != SkillState::Waiting)
    {
        run.end = p_now;
        m_runs[p_id] = static_cast<std::int32_t>(m_trace.size());
        m_trace.push_back({ p_id,
                            m_states[p_id],
                            m_reasons[p_id],
                            m_blockers[p_id],
                            p_now,
                            p_now });
        return;
    }
    run.state = m_states[p_id];
    run.reason = m_reasons[p_id];
    run.blocker = m_blockers[p_id];
    run.end = p_now;
    if (run.reason == SkillReason::Failed)
    {
        run.failure = m_skills[p_id]->failure();
    }
}

void SkillScheduler::finish(SkillId p_id,
                            SkillState p_state,
                            SkillReason p_reason,
                            Seconds p_now)
{
    m_leases[p_id].release();
    if (p_reason == SkillReason::Preempted && m_descriptions[p_id].wait)
    {
        // The run is not over: it waits for its resources again, at its
        // place in the order, and starts again from Skill::reset.
        m_states[p_id] = SkillState::Cancelled;
        m_reasons[p_id] = SkillReason::Preempted;
        m_cancels[p_id] = 0U;
        record(p_id, p_now);
        m_states[p_id] = SkillState::Waiting;
        m_runs[p_id] = -1;
        record(p_id, p_now);
        changed(p_id, SkillState::Waiting);
        return;
    }
    m_states[p_id] = p_state;
    m_reasons[p_id] = p_reason;
    m_cancels[p_id] = 0U;
    m_ended[p_id].push_back({ m_generations[p_id],
                              p_state,
                              p_reason,
                              p_reason == SkillReason::Failed
                                  ? m_skills[p_id]->failure()
                                  : SkillFailure{} });
    record(p_id, p_now);
    changed(p_id, p_state);
    dequeue(p_id);
}

RobotContext& SkillScheduler::contextOf(SkillId p_id) const
{
    std::string const& robot = m_descriptions[p_id].robot;
    if (robot.empty())
    {
        return m_contexts.front();
    }
    for (RobotContext& context : m_contexts)
    {
        if (context.robot.id() == robot)
        {
            return context;
        }
    }
    throw std::runtime_error("Skill '" + m_descriptions[p_id].name +
                             "' drives robot '" + robot +
                             "', which is not in the world");
}

void SkillScheduler::stop(SkillId p_id,
                          SkillState p_state,
                          SkillReason p_reason,
                          bool p_at_once)
{
    RobotContext& context = contextOf(p_id);
    if (m_states[p_id] == SkillState::Stopping)
    {
        if (p_at_once)
        {
            finish(p_id, m_stop_states[p_id], m_stop_reasons[p_id], context.time);
        }
        return;
    }
    if (m_states[p_id] != SkillState::Running)
    {
        finish(p_id, p_state, p_reason, context.time);
        return;
    }
    {
        WriterScope const writer(m_resources, p_id);
        m_skills[p_id]->cancel(context);
    }
    if (p_at_once)
    {
        finish(p_id, p_state, p_reason, context.time);
        return;
    }
    // The resources stay held until the skill says the hardware is safe.
    m_states[p_id] = SkillState::Stopping;
    m_stop_states[p_id] = p_state;
    m_stop_reasons[p_id] = p_reason;
    m_stop_since[p_id] = context.time;
    record(p_id, context.time);
    changed(p_id, SkillState::Stopping);
    stopping(p_id);
}

void SkillScheduler::stopping(SkillId p_id)
{
    RobotContext& context = contextOf(p_id);
    Status status = Status::RUNNING;
    {
        WriterScope const writer(m_resources, p_id);
        status = m_skills[p_id]->stopping(context, context.dt);
    }
    bool const late = context.time - m_stop_since[p_id] >=
                      m_descriptions[p_id].stop_timeout;
    if (status == Status::RUNNING && !late)
    {
        record(p_id, context.time);
        return;
    }
    if (status == Status::RUNNING && m_runs[p_id] >= 0)
    {
        m_trace[static_cast<std::size_t>(m_runs[p_id])].forced = true;
    }
    finish(p_id, m_stop_states[p_id], m_stop_reasons[p_id], context.time);
}

bool SkillScheduler::victims(SkillId p_by, ResourceId& p_blocker)
{
    Priority const priority = m_descriptions[p_by].priority;
    auto preemptible = [&](OwnerId p_victim)
    {
        if (p_victim >= m_skills.size() || p_victim == p_by)
        {
            return false;
        }
        // A stopping skill gives its resources back soon, whoever waits.
        if (m_states[p_victim] == SkillState::Stopping)
        {
            return true;
        }
        return m_states[p_victim] == SkillState::Running &&
               m_descriptions[p_victim].cancellable &&
               m_descriptions[p_victim].priority < priority;
    };
    auto add = [this](SkillId p_victim, ResourceId p_resource)
    {
        auto const same = [p_victim](Victim const& p_other)
        { return p_other.skill == p_victim; };
        if (std::ranges::none_of(m_victims, same))
        {
            m_victims.push_back({ p_victim, p_resource });
        }
    };

    // Phase 1 only: find every holder of every conflict, stop nobody.
    m_victims.clear();
    for (ResourceRequirement const& requirement :
         m_descriptions[p_by].resources)
    {
        ResourceId const id = requirement.id;
        if (!m_resources.available(id))
        {
            p_blocker = id;
            return false;
        }
        if (OwnerId const owner = m_resources.owner(id); owner != NO_OWNER)
        {
            if (!preemptible(owner))
            {
                p_blocker = id;
                return false;
            }
            add(static_cast<SkillId>(owner), id);
            continue;
        }
        if (requirement.access == Access::Shared ||
            m_resources.users(id) == 0U)
        {
            continue;
        }
        // Shared holders blocking an exclusive request. A shared lease taken
        // outside the scheduler is not in m_leases: it cannot be preempted.
        std::uint16_t found = 0U;
        for (SkillId holder = 0; holder < m_skills.size(); ++holder)
        {
            for (ResourceRequirement const& held : m_leases[holder].resources())
            {
                if (held.id != id)
                {
                    continue;
                }
                if (!preemptible(holder))
                {
                    p_blocker = id;
                    return false;
                }
                add(holder, id);
                ++found;
            }
        }
        if (found < m_resources.users(id))
        {
            p_blocker = id;
            return false;
        }
    }
    return true;
}

void SkillScheduler::admit(SkillId p_id)
{
    RobotContext& p_context = contextOf(p_id);
    SkillDescription const& description = m_descriptions[p_id];
    Seconds const now = p_context.time;
    // A new run: the failure of the last one is no longer true.
    m_skills[p_id]->m_failure = {};

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
                finish(
                    p_id, SkillState::Failed, SkillReason::Precondition, now);
            }
            return;
        }
    }

    // Preemption is all or nothing: the victims are stopped only when every
    // holder of every conflict can be preempted.
    // An urgent skill (Stop) stops them at once; the others let them stop.
    ResourceId blocker = NO_RESOURCE;
    if (m_resources.conflict(description.resources))
    {
        if (victims(p_id, blocker))
        {
            for (Victim const& victim : m_victims)
            {
                if (m_states[victim.skill] == SkillState::Running)
                {
                    m_blockers[victim.skill] = victim.resource;
                }
                stop(victim.skill,
                     SkillState::Cancelled,
                     SkillReason::Preempted,
                     description.urgent);
            }
            if (m_resources.conflict(description.resources))
            {
                // Some victims are still stopping: wait for them.
                m_blockers[p_id] = m_victims.front().resource;
                m_reasons[p_id] = SkillReason::Preempting;
                record(p_id, now);
                return;
            }
        }
    }
    if (blocker == NO_RESOURCE && !m_resources.conflict(description.resources))
    {
        m_leases[p_id] = m_resources.acquire(description.resources, p_id);
        m_states[p_id] = SkillState::Running;
        m_reasons[p_id] = SkillReason::None;
        m_blockers[p_id] = NO_RESOURCE;
        {
            WriterScope const writer(m_resources, p_id);
            m_skills[p_id]->reset();
        }
        record(p_id, now);
        changed(p_id, SkillState::Running);
        return;
    }

    m_blockers[p_id] = blocker;
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
    update(std::span<RobotContext>(&p_context, 1U));
}

void SkillScheduler::update(std::span<RobotContext> p_contexts)
{
    if (p_contexts.empty())
    {
        throw std::invalid_argument(
            "SkillScheduler::update needs a robot context");
    }
    m_contexts = p_contexts;
    // Explicit cancellations.
    for (SkillId id = 0; id < m_skills.size(); ++id)
    {
        if (m_cancels[id] != 0U)
        {
            bool const at_once = m_cancels[id] == 2U;
            m_cancels[id] = 0U;
            stop(id, SkillState::Cancelled, SkillReason::Cancelled, at_once);
        }
    }

    // Stopping runs that were not stopped above: are they safe now?
    for (SkillId id = 0; id < m_skills.size(); ++id)
    {
        if (m_states[id] == SkillState::Stopping &&
            m_stop_since[id] < contextOf(id).time)
        {
            stopping(id);
        }
    }

    // Runs whose resources failed, stopping or not.
    for (SkillId id = 0; id < m_skills.size(); ++id)
    {
        if (m_states[id] != SkillState::Running &&
            m_states[id] != SkillState::Stopping)
        {
            continue;
        }
        for (ResourceRequirement const& requirement :
             m_descriptions[id].resources)
        {
            if (!m_resources.available(requirement.id))
            {
                // The resource is gone: no stopping phase can use it.
                m_blockers[id] = requirement.id;
                stop(id, SkillState::Failed, SkillReason::ResourceLost, true);
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
    std::ranges::sort(m_queue, by_priority);
    for (SkillId const id : m_queue)
    {
        if (m_states[id] == SkillState::Waiting)
        {
            admit(id);
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
    std::ranges::sort(m_queue, by_priority);
    for (SkillId const id : m_queue)
    {
        if (m_states[id] != SkillState::Running)
        {
            continue;
        }
        RobotContext& context = contextOf(id);
        Status status = Status::RUNNING;
        {
            // The skill commands only what its lease holds.
            WriterScope const writer(m_resources, id);
            status = m_skills[id]->tick(context, context.dt);
        }
        if (status == Status::SUCCESS)
        {
            finish(id, SkillState::Succeeded, SkillReason::None, context.time);
        }
        else if (status == Status::FAILURE)
        {
            finish(id, SkillState::Failed, SkillReason::Failed, context.time);
        }
        else
        {
            record(id, context.time);
        }
    }
}

} // namespace robotik
