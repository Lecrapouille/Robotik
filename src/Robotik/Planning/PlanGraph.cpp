// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Planning/PlanGraph.hpp"

#include <algorithm>

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
        if (!wildcard(p_a.args[i]) && !wildcard(p_b.args[i]) && p_a.args[i] != p_b.args[i])
        {
            return false;
        }
    }
    return true;
}

/// @brief Some atom of @p_a may be some atom of @p_b.
static bool overlap(std::vector<Atom> const& p_a, std::vector<Atom> const& p_b)
{
    return std::any_of(p_a.begin(), p_a.end(),
                       [&](Atom const& p_x)
                       {
                           return std::any_of(p_b.begin(), p_b.end(),
                                              [&](Atom const& p_y) { return unify(p_x, p_y); });
                       });
}

/// @brief Both need a resource, and one of them alone.
static bool contend(std::vector<ResourceRequirement> const& p_a,
                    std::vector<ResourceRequirement> const& p_b)
{
    for (ResourceRequirement const& x : p_a)
    {
        for (ResourceRequirement const& y : p_b)
        {
            if (x.id == y.id && (x.access == Access::Exclusive || y.access == Access::Exclusive))
            {
                return true;
            }
        }
    }
    return false;
}

// ----------------------------------------------------------------------------
StepFootprint footprint(ActionSchema const& p_action, GroundAction const& p_step)
{
    auto ground = [&](Atom const& p_atom)
    {
        Atom result{ p_atom.predicate, {} };
        for (std::string const& arg : p_atom.args)
        {
            auto const found = std::find_if(p_action.parameters.begin(), p_action.parameters.end(),
                                            [&](Parameter const& p_parameter)
                                            { return "?" + p_parameter.name == arg; });
            std::size_t const index = static_cast<std::size_t>(found - p_action.parameters.begin());
            result.args.push_back(index < p_step.args.size() ? lowerCase(p_step.args[index])
                                                             : lowerCase(arg));
        }
        return result;
    };
    StepFootprint result;
    for (Atom const& atom : p_action.reads)
    {
        result.reads.push_back(ground(atom));
    }
    for (Atom const& atom : p_action.writes)
    {
        result.writes.push_back(ground(atom));
    }
    return result;
}

// ----------------------------------------------------------------------------
std::vector<std::vector<std::size_t>> orderSteps(std::span<StepFootprint const> p_steps)
{
    std::size_t const n = p_steps.size();
    // before[i][j]: i must end before j starts (i < j).
    std::vector<std::vector<bool>> before(n, std::vector<bool>(n, false));
    for (std::size_t j = 0u; j < n; ++j)
    {
        for (std::size_t i = 0u; i < j; ++i)
        {
            StepFootprint const& a = p_steps[i];
            StepFootprint const& b = p_steps[j];
            before[i][j] = overlap(a.writes, b.reads) || overlap(b.writes, a.reads) ||
                           overlap(a.writes, b.writes) || contend(a.resources, b.resources);
        }
    }
    // Transitive closure, in plan order (i < k < j).
    for (std::size_t k = 0u; k < n; ++k)
    {
        for (std::size_t i = 0u; i < k; ++i)
        {
            if (!before[i][k])
            {
                continue;
            }
            for (std::size_t j = k + 1u; j < n; ++j)
            {
                before[i][j] = before[i][j] || before[k][j];
            }
        }
    }
    // Reduction: keep i -> j unless some i -> k -> j.
    std::vector<std::vector<std::size_t>> after(n);
    for (std::size_t j = 0u; j < n; ++j)
    {
        for (std::size_t i = 0u; i < j; ++i)
        {
            bool implied = false;
            for (std::size_t k = i + 1u; k < j && !implied; ++k)
            {
                implied = before[i][k] && before[k][j];
            }
            if (before[i][j] && !implied)
            {
                after[j].push_back(i);
            }
        }
    }
    return after;
}

// ----------------------------------------------------------------------------
bool sequential(std::vector<std::vector<std::size_t>> const& p_after)
{
    for (std::size_t j = 0u; j < p_after.size(); ++j)
    {
        bool const chained = j == 0u ? p_after[j].empty()
                                     : p_after[j].size() == 1u && p_after[j].front() == j - 1u;
        if (!chained)
        {
            return false;
        }
    }
    return true;
}

} // namespace robotik
