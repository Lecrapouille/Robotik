// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Planning/PlanningDomain.hpp"

#include <cctype>
#include <stdexcept>

namespace robotik
{

void PlanningDomain::observe(std::string_view p_predicate, Observer p_observer)
{
    m_observers[lowerCase(p_predicate)] = std::move(p_observer);
}

void PlanningDomain::bind(
    std::string_view p_action,
    std::vector<std::string> p_nodes,
    std::vector<std::pair<std::string, std::string>> p_parameters)
{
    m_bindings[lowerCase(p_action)] = { std::move(p_nodes),
                                        std::move(p_parameters) };
}

Observer const* PlanningDomain::observer(std::string_view p_predicate) const
{
    auto const found = m_observers.find(p_predicate);
    return found == m_observers.end() ? nullptr : &found->second;
}

ActionBinding const* PlanningDomain::binding(std::string_view p_action) const
{
    auto const found = m_bindings.find(p_action);
    return found == m_bindings.end() ? nullptr : &found->second;
}

std::string expand(std::string_view p_pattern,
                   std::span<std::string const> p_args)
{
    std::string result;
    for (std::size_t i = 0U; i < p_pattern.size(); ++i)
    {
        std::size_t const close = p_pattern.find('}', i);
        if (p_pattern[i] == '{' && close != std::string_view::npos &&
            close > i + 1U)
        {
            std::string_view const index =
                p_pattern.substr(i + 1U, close - i - 1U);
            bool digits = true;
            for (char const c : index)
            {
                digits =
                    digits && std::isdigit(static_cast<unsigned char>(c)) != 0;
            }
            if (digits)
            {
                std::size_t const k = std::stoul(std::string(index));
                if (k >= p_args.size())
                {
                    throw std::out_of_range(
                        "'" + std::string(p_pattern) + "' uses {" +
                        std::string(index) + "} but the action has " +
                        std::to_string(p_args.size()) + " argument(s)");
                }
                result += p_args[k];
                i = close;
                continue;
            }
        }
        result += p_pattern[i];
    }
    return result;
}

} // namespace robotik
