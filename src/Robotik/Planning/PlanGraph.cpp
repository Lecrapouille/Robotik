// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Planning/PlanGraph.hpp"

#include <algorithm>
#include <tuple>

namespace robotik
{

/// @brief An argument that matches anything: a free variable, or a nested
/// term the schema does not read.
static bool wildcard(std::string const& p_arg)
{
    return p_arg.empty() || p_arg.front() == '?';
}

/// @brief Two atoms may be the same ground atom.
static bool unify(Atom const& p_a, Atom const& p_b)
{
    if (p_a.predicate != p_b.predicate || p_a.args.size() != p_b.args.size())
    {
        return false;
    }
    for (std::size_t i = 0u; i < p_a.args.size(); ++i)
    {
        if (!wildcard(p_a.args[i]) && !wildcard(p_b.args[i]) &&
            p_a.args[i] != p_b.args[i])
        {
            return false;
        }
    }
    return true;
}

/// @brief Some atom of @p_a may be some atom of @p_b.
static bool overlap(std::vector<Atom> const& p_a, std::vector<Atom> const& p_b)
{
    return std::any_of(p_a.begin(),
                       p_a.end(),
                       [&](Atom const& p_x)
                       {
                           return std::any_of(p_b.begin(),
                                              p_b.end(),
                                              [&](Atom const& p_y)
                                              { return unify(p_x, p_y); });
                       });
}

/// @brief One event reads what the other changes, or both change it.
static bool interfere(ActionSchema::Phase const& p_a,
                      ActionSchema::Phase const& p_b)
{
    return overlap(p_a.writes, p_b.reads) || overlap(p_b.writes, p_a.reads) ||
           overlap(p_a.writes, p_b.writes);
}

/// @brief Both need a resource, and one of them alone.
static bool contend(std::vector<ResourceRequirement> const& p_a,
                    std::vector<ResourceRequirement> const& p_b)
{
    for (ResourceRequirement const& x : p_a)
    {
        for (ResourceRequirement const& y : p_b)
        {
            if (x.id == y.id && (x.access == Access::Exclusive ||
                                 y.access == Access::Exclusive))
            {
                return true;
            }
        }
    }
    return false;
}

// ----------------------------------------------------------------------------
StepFootprint footprint(ActionSchema const& p_action,
                        GroundAction const& p_step)
{
    auto ground = [&](std::vector<Atom> const& p_atoms)
    {
        std::vector<Atom> result;
        for (Atom const& atom : p_atoms)
        {
            Atom ground{ atom.predicate, {} };
            for (std::string const& arg : atom.args)
            {
                auto const found =
                    std::find_if(p_action.parameters.begin(),
                                 p_action.parameters.end(),
                                 [&](Parameter const& p_parameter)
                                 { return "?" + p_parameter.name == arg; });
                std::size_t const index = static_cast<std::size_t>(
                    found - p_action.parameters.begin());
                ground.args.push_back(index < p_step.args.size()
                                          ? lowerCase(p_step.args[index])
                                          : lowerCase(arg));
            }
            result.push_back(std::move(ground));
        }
        return result;
    };
    StepFootprint result;
    result.start = { ground(p_action.start.reads),
                     ground(p_action.start.writes) };
    result.end = { ground(p_action.end.reads), ground(p_action.end.writes) };
    result.start_time = p_step.start;
    result.end_time = p_step.start + p_step.duration;
    return result;
}

// ----------------------------------------------------------------------------
std::vector<StepOrder> orderSteps(std::span<StepFootprint const> p_steps)
{
    // Events in plan order: by time, then step, then the start first.
    std::size_t const n = p_steps.size();
    std::vector<PlanEvent> events;
    for (std::size_t k = 0u; k < n; ++k)
    {
        events.push_back({ k, false });
        events.push_back({ k, true });
    }
    auto time = [&](PlanEvent const& p_event)
    {
        return p_event.end ? p_steps[p_event.step].end_time
                           : p_steps[p_event.step].start_time;
    };
    std::stable_sort(events.begin(),
                     events.end(),
                     [&](PlanEvent const& p_a, PlanEvent const& p_b)
                     {
                         return std::make_tuple(time(p_a), p_a.step, p_a.end) <
                                std::make_tuple(time(p_b), p_b.step, p_b.end);
                     });
    std::size_t const m = events.size();
    auto phase = [&](PlanEvent const& p_event) -> ActionSchema::Phase const&
    {
        return p_event.end ? p_steps[p_event.step].end
                           : p_steps[p_event.step].start;
    };

    // before[i][j]: event i must happen before event j (i < j in plan order).
    std::vector<std::vector<bool>> before(m, std::vector<bool>(m, false));
    for (std::size_t j = 0u; j < m; ++j)
    {
        for (std::size_t i = 0u; i < j; ++i)
        {
            PlanEvent const& a = events[i];
            PlanEvent const& b = events[j];
            if (a.step == b.step)
            {
                before[i][j] = true; // A step starts before it ends.
                continue;
            }
            // A shared arm: the first step ends before the second one starts.
            // Steps that overlap in a temporal plan and share one are left
            // to the scheduler (the second waits for the resource).
            before[i][j] =
                interfere(phase(a), phase(b)) ||
                (a.end && !b.end &&
                 contend(p_steps[a.step].resources, p_steps[b.step].resources));
        }
    }
    // Transitive closure, in plan order (i < k < j).
    for (std::size_t k = 0u; k < m; ++k)
    {
        for (std::size_t i = 0u; i < k; ++i)
        {
            if (!before[i][k])
            {
                continue;
            }
            for (std::size_t j = k + 1u; j < m; ++j)
            {
                before[i][j] = before[i][j] || before[k][j];
            }
        }
    }
    // Reduction: keep i -> j unless some i -> k -> j; drop start -> own end.
    std::vector<StepOrder> result(n);
    for (std::size_t j = 0u; j < m; ++j)
    {
        for (std::size_t i = 0u; i < j; ++i)
        {
            bool implied = false;
            for (std::size_t k = i + 1u; k < j && !implied; ++k)
            {
                implied = before[i][k] && before[k][j];
            }
            if (!before[i][j] || implied)
            {
                continue;
            }
            PlanEvent const& a = events[i];
            PlanEvent const& b = events[j];
            if (a.step == b.step)
            {
                continue;
            }
            StepOrder& order = result[b.step];
            (b.end ? order.end_after : order.start_after).push_back(a);
        }
    }
    return result;
}

// ----------------------------------------------------------------------------
bool sequential(std::span<StepOrder const> p_order)
{
    for (std::size_t k = 0u; k < p_order.size(); ++k)
    {
        std::vector<PlanEvent> const previous =
            k == 0u ? std::vector<PlanEvent>{}
                    : std::vector<PlanEvent>{ { k - 1u, true } };
        if (p_order[k].start_after != previous || !p_order[k].end_after.empty())
        {
            return false;
        }
    }
    return true;
}

} // namespace robotik
