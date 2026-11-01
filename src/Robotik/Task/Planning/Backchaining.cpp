// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Planning/Backchaining.hpp"

#include <algorithm>
#include <optional>

namespace robotik
{

/// Index of parameter @p_name (written @c ?x) in @p_parameters, or nothing.
static std::optional<std::size_t>
parameterIndex(std::vector<Parameter> const& p_parameters,
               std::string_view p_name)
{
    for (std::size_t i = 0U; i < p_parameters.size(); ++i)
    {
        if (p_name.size() == p_parameters[i].name.size() + 1U &&
            p_name.front() == '?' && p_name.substr(1U) == p_parameters[i].name)
        {
            return i;
        }
    }
    return std::nullopt;
}

/// True when object @p_name is of type @p_type (or derives from it).
static bool
fits(DomainSchema const& p_schema,
     std::map<std::string, std::string, std::less<>> const& p_objects,
     std::string const& p_name,
     std::string const& p_type)
{
    auto const found = p_objects.find(p_name);
    return found != p_objects.end() && isA(p_schema, found->second, p_type);
}

// ----------------------------------------------------------------------------
BackchainTree backchainTree(std::vector<Literal> const& p_goal)
{
    BackchainTree tree;
    for (Literal const& literal : p_goal)
    {
        tree.goal.push_back(tree.conditions.size());
        tree.conditions.push_back({ literal, NO_ACTION, false, {} });
    }
    return tree;
}

// ----------------------------------------------------------------------------
std::vector<Literal> ancestors(BackchainTree const& p_tree,
                               std::size_t p_condition)
{
    std::vector<Literal> result;
    std::size_t condition = p_condition;
    while (condition < p_tree.conditions.size())
    {
        result.push_back(p_tree.conditions[condition].literal);
        std::size_t const action = p_tree.conditions[condition].action;
        condition = action < p_tree.actions.size()
                        ? p_tree.actions[action].condition
                        : p_tree.conditions.size();
    }
    return result;
}

// ----------------------------------------------------------------------------
std::vector<GroundAction>
achievers(DomainSchema const& p_schema,
          std::map<std::string, std::string, std::less<>> const& p_objects,
          Literal const& p_literal,
          std::size_t p_limit)
{
    std::vector<GroundAction> result;
    for (auto const& [name, action] : p_schema.actions)
    {
        for (Literal const& effect : action.effects)
        {
            if (effect.positive != p_literal.positive ||
                effect.atom.predicate != p_literal.atom.predicate ||
                effect.atom.args.size() != p_literal.atom.args.size())
            {
                continue;
            }
            // Fix the parameters the effect names.
            std::vector<std::optional<std::string>> bound(
                action.parameters.size());
            bool unified = true;
            for (std::size_t i = 0U; i < effect.atom.args.size() && unified;
                 ++i)
            {
                std::string const& want = p_literal.atom.args[i];
                std::optional<std::size_t> const index =
                    parameterIndex(action.parameters, effect.atom.args[i]);
                if (!index)
                {
                    unified = effect.atom.args[i] == want; // a constant
                }
                else if (bound[*index])
                {
                    unified = *bound[*index] == want;
                }
                else
                {
                    unified = fits(p_schema,
                                   p_objects,
                                   want,
                                   action.parameters[*index].type);
                    bound[*index] = want;
                }
            }
            if (!unified)
            {
                continue;
            }
            // The other parameters take every object of their type.
            std::vector<std::vector<std::string>> choices(
                action.parameters.size());
            for (std::size_t i = 0U; i < action.parameters.size(); ++i)
            {
                if (bound[i])
                {
                    choices[i] = { *bound[i] };
                    continue;
                }
                for (auto const& [object, type] : p_objects)
                {
                    if (isA(p_schema, type, action.parameters[i].type))
                    {
                        choices[i].push_back(object);
                    }
                }
            }
            if (std::ranges::any_of(
                    choices, [](auto const& p_c) { return p_c.empty(); }))
            {
                continue;
            }
            std::vector<std::size_t> pick(choices.size(), 0U);
            while (result.size() < p_limit)
            {
                GroundAction ground{ name, {}, 0.0, 0.0 };
                for (std::size_t i = 0U; i < choices.size(); ++i)
                {
                    ground.args.push_back(choices[i][pick[i]]);
                }
                if (std::ranges::find(result, ground) == result.end())
                {
                    result.push_back(std::move(ground));
                }
                // Next combination, the last parameter turning fastest.
                std::size_t i = choices.size();
                while (i > 0U && ++pick[i - 1U] == choices[i - 1U].size())
                {
                    pick[i - 1U] = 0U;
                    --i;
                }
                if (i == 0U)
                {
                    break;
                }
            }
        }
    }
    return result;
}

// ----------------------------------------------------------------------------
std::vector<Literal> groundPreconditions(DomainSchema const& p_schema,
                                         GroundAction const& p_action)
{
    std::vector<Literal> result;
    auto const action = p_schema.actions.find(p_action.name);
    if (action == p_schema.actions.end())
    {
        return result;
    }
    for (Literal const& precondition : action->second.preconditions)
    {
        Literal ground{ { precondition.atom.predicate, {} },
                        precondition.positive };
        for (std::string const& arg : precondition.atom.args)
        {
            std::optional<std::size_t> const index =
                parameterIndex(action->second.parameters, arg);
            ground.atom.args.push_back(index && *index < p_action.args.size()
                                           ? p_action.args[*index]
                                           : arg);
        }
        result.push_back(std::move(ground));
    }
    return result;
}

// ----------------------------------------------------------------------------
std::set<std::string, std::less<>>
staticPredicates(DomainSchema const& p_schema)
{
    std::set<std::string, std::less<>> result;
    for (auto const& [name, predicate] : p_schema.predicates)
    {
        if (!predicate.derived)
        {
            result.insert(name);
        }
    }
    for (auto const& [name, action] : p_schema.actions)
    {
        for (ActionSchema::Phase const* phase : { &action.start, &action.end })
        {
            for (Atom const& atom : phase->writes)
            {
                result.erase(atom.predicate);
            }
        }
    }
    return result;
}

// ----------------------------------------------------------------------------
std::size_t attach(BackchainTree& p_tree,
                   std::size_t p_condition,
                   GroundAction p_action,
                   std::vector<Literal> const& p_preconditions)
{
    std::size_t const index = p_tree.actions.size();
    BackchainAction action{ std::move(p_action), p_condition, {} };
    for (Literal const& literal : p_preconditions)
    {
        action.preconditions.push_back(p_tree.conditions.size());
        p_tree.conditions.push_back({ literal, index, false, {} });
    }
    p_tree.actions.push_back(std::move(action));
    p_tree.conditions[p_condition].achievers.push_back(index);
    return index;
}

} // namespace robotik
