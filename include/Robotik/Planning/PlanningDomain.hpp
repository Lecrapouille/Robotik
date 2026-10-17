// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PlanningDomain.hpp
//! @brief What a mission tells the planner: how to observe each predicate
//! of its PDDL domain and which skills carry out each action.
//!
//! The domain file holds the types, predicates and actions; the world gives
//! the objects (entities with a @c kind); the scenario gives the goal. The
//! mission only links names:
//!
//! @code
//! void setup(robotik::Simulation& p_simulation, robotik::SceneView*) override
//! {
//!     robotik::PlanningDomain& domain = p_simulation.planning();
//!     // State: computed on the beliefs, never on the ground truth.
//!     domain.observe("holding", [](robotik::RobotContext const& p_context,
//!                                  robotik::Args p_args) {
//!         return robotik::truth(heldObject(p_context) == p_args[0]);
//!     });
//!     // Action -> behavior tree nodes; {0} is the first argument.
//!     domain.bind("pick", { "Approach({0})", "Reach({0})", "Grasp({0})" });
//!     // A node reading a blackboard value: parameters set it per step.
//!     domain.bind("dip", { "Dip" }, { { "color", "{0}" } });
//! }
//! @endcode
//!
//! Arguments are entity ids as written in the world file. Every predicate
//! needs an observer, a fact of the scenario or a @c :derived rule, and
//! every action a binding: the scenario is refused otherwise.
#pragma once

#include "Robotik/Planning/Pddl.hpp"

#include <functional>
#include <map>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace robotik
{

struct RobotContext;

//! @brief Arguments of an observed atom: entity ids.
using Args = std::span<std::string const>;

//! @brief Value of a predicate for these arguments, from the beliefs.
using Observer = std::function<Truth(RobotContext const&, Args)>;

// ****************************************************************************
//! @brief Behavior tree nodes run for one action of the plan, in order.
// ****************************************************************************
struct ActionBinding
{
    //!< Node names; @c {0}, @c {1}... are replaced by the arguments.
    std::vector<std::string> nodes;
    //!< Blackboard values of the step (@c {0} replaced as well).
    std::vector<std::pair<std::string, std::string>> parameters;
};

// ****************************************************************************
//! @brief Observers and bindings of a mission, keyed by PDDL name.
// ****************************************************************************
class PlanningDomain
{
public:

    // -------------------------------------------------------------------------
    //! @brief How to read @p_predicate on the beliefs. Replaces a previous one.
    // -------------------------------------------------------------------------
    void observe(std::string_view p_predicate, Observer p_observer);

    // -------------------------------------------------------------------------
    //! @brief Nodes run for @p_action. Replaces a previous binding.
    // -------------------------------------------------------------------------
    void
    bind(std::string_view p_action,
         std::vector<std::string> p_nodes,
         std::vector<std::pair<std::string, std::string>> p_parameters = {});

    [[nodiscard]] Observer const* observer(std::string_view p_predicate) const;
    [[nodiscard]] ActionBinding const* binding(std::string_view p_action) const;

    [[nodiscard]] std::map<std::string, Observer, std::less<>> const&
    observers() const
    {
        return m_observers;
    }

    [[nodiscard]] std::map<std::string, ActionBinding, std::less<>> const&
    bindings() const
    {
        return m_bindings;
    }

    void clear()
    {
        m_observers.clear();
        m_bindings.clear();
    }

private:

    std::map<std::string, Observer, std::less<>> m_observers;
    std::map<std::string, ActionBinding, std::less<>> m_bindings;
};

// ----------------------------------------------------------------------------
//! @brief Replaces @c {0}, @c {1}... in @p_pattern by @p_args.
//! @throws std::out_of_range when an index has no argument.
// ----------------------------------------------------------------------------
[[nodiscard]] std::string expand(std::string_view p_pattern,
                                 std::span<std::string const> p_args);

} // namespace robotik
