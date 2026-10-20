// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Pddl.hpp
/// Symbols of the planner and what RobotIK reads of a PDDL domain.
///
/// The domain file is written in PDDL and solved by the planner. RobotIK
/// reads the part it needs to build problems and check the plan: types,
/// constants, predicates and the plain effects of the actions. Names are
/// lower case, as PDDL does not tell case apart.
#pragma once

#include "Robotik/Common/Result.hpp"

#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// Value of a predicate in the beliefs. Unknown is false for the planner
/// (closed world) and is not a failure when an effect is checked.
///
/// @code
/// domain.observe("on_table", [](robotik::RobotContext const& p_context,
/// robotik::Args p_args) {
///     auto const* o = p_context.world_model.find(p_args[0]);
///     if (o == nullptr || !o->observed())
///         return robotik::Truth::Unknown;
///     return robotik::truth(o->position.z < 0.05);
/// });
/// @endcode
// ****************************************************************************
enum class Truth : std::uint8_t
{
    False,   ///< The predicate does not hold.
    True,    ///< The predicate holds.
    Unknown, ///< The beliefs cannot tell.
};

// ----------------------------------------------------------------------------
/// @param p_value A boolean.
/// @return @c Truth::True or @c Truth::False.
// ----------------------------------------------------------------------------
[[nodiscard]] constexpr Truth truth(bool p_value)
{
    return p_value ? Truth::True : Truth::False;
}

// ----------------------------------------------------------------------------
/// @param p_truth Value to name.
/// @return "true", "false" or "unknown".
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(Truth p_truth);

// ****************************************************************************
/// A predicate applied to objects: @c in(red_cube, box). In an action schema
/// an argument may be a parameter (@c ?o).
///
/// @code
/// robotik::Atom const a{ "in", { "red_cube", "box" } };
/// a.toPddl();   // "(in red_cube box)"
/// a.toString(); // "in(red_cube, box)"
/// @endcode
// ****************************************************************************
struct Atom
{
    /// Predicate name.
    std::string predicate;
    /// Arguments: object names, or parameters @c ?x in a schema.
    std::vector<std::string> args;

    /// @return @c "(in red_cube box)".
    [[nodiscard]] std::string toPddl() const;
    /// @return @c "in(red_cube, box)", or @c "hand_empty" without argument.
    [[nodiscard]] std::string toString() const;

    /// Compares the predicate, then the arguments.
    auto operator<=>(Atom const&) const = default;
};

// ****************************************************************************
/// An atom or its negation.
///
/// @code
/// robotik::Literal const goal = robotik::Literal::parse("not in(red_cube,
/// box)"); goal.positive;  // false goal.toPddl();  // "(not (in red_cube
/// box))"
/// @endcode
// ****************************************************************************
struct Literal
{
    /// The atom.
    Atom atom;
    /// False for @c not atom.
    bool positive = true;

    // -------------------------------------------------------------------------
    /// Reads the scenario syntax. Names are lower-cased.
    /// @param p_text @c "in(red_cube, box)", @c "not in(red_cube, box)" or
    ///   @c "hand_empty".
    /// @return The literal; fails on a malformed literal.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Result<Literal> parse(std::string_view p_text);

    /// @return @c "(not (in red_cube box))".
    [[nodiscard]] std::string toPddl() const;
    /// @return @c "not in(red_cube, box)".
    [[nodiscard]] std::string toString() const;

    /// Compares the atom, then the polarity.
    auto operator<=>(Literal const&) const = default;
};

// ****************************************************************************
/// One action of a plan: @c (pick red_cube).
///
/// @code
/// for (robotik::GroundAction const& step : result.steps)
///     std::cout << step.start << ": " << step.toPddl() << '\n';
/// @endcode
// ****************************************************************************
struct GroundAction
{
    /// Action name of the domain.
    std::string name;
    /// Object names, in the order of the parameters.
    std::vector<std::string> args;
    /// Start in the plan: the time of a temporal plan (s, as the domain
    /// counts it), the position in a sequential one (0, 1, 2...).
    double start = 0.0;
    /// Duration in a temporal plan; 0 for a plain action.
    double duration = 0.0;

    /// @return @c "(pick red_cube)".
    [[nodiscard]] std::string toPddl() const;

    /// @return True for the same action and arguments (the timing is not
    ///   compared).
    bool operator==(GroundAction const& p_other) const
    {
        return name == p_other.name && args == p_other.args;
    }
};

// ****************************************************************************
/// A typed parameter of a predicate or an action (name without @c ?).
///
/// @code
/// // (:action pick :parameters (?o - block) ...)
/// robotik::Parameter const p{ "o", "block" };
/// @endcode
// ****************************************************************************
struct Parameter
{
    /// Name without @c ?.
    std::string name;
    /// PDDL type (@c object when not given).
    std::string type;
};

// ****************************************************************************
/// A predicate declared in @c :predicates.
///
/// @code
/// auto const& in = domain.predicates.at("in");
/// in.types; // { "block", "container" }
/// @endcode
// ****************************************************************************
struct PredicateSchema
{
    /// Predicate name.
    std::string name;
    /// Type of each argument.
    std::vector<std::string> types;
    /// Defined by a @c :derived rule: the planner computes it.
    bool derived = false;
};

// ****************************************************************************
/// An action of the domain, as RobotIK reads it.
///
/// @code
/// robotik::ActionSchema const& pick = domain.actions.at("pick");
/// for (robotik::Literal const& e : pick.effects)
///     std::cout << e.toString() << '\n'; // e.g. "holding(?o)"
/// @endcode
// ****************************************************************************
struct ActionSchema
{
    /// Action name.
    std::string name;
    /// Typed parameters, in order.
    std::vector<Parameter> parameters;
    /// Plain effects (@c at @c end for a durative action): what the executor
    /// checks once the action is done. Arguments are parameters (@c ?o) or
    /// constants.
    std::vector<Literal> effects;
    /// Some effects are conditional, quantified or numeric: not checked.
    bool partial = false;
    /// Declared with @c :durative-action.
    bool durative = false;
    // ------------------------------------------------------------------------
    /// Atoms one end of the action reads and changes, whatever their
    /// polarity or nesting (numeric fluents included). A quantified variable
    /// stays @c ?x: it matches any object. Used to tell which steps of a plan
    /// are independent (PlanGraph.hpp).
    // ------------------------------------------------------------------------
    struct Phase
    {
        /// Atoms the conditions read.
        std::vector<Atom> reads;
        /// Atoms the effects change.
        std::vector<Atom> writes;
    };
    /// Start: the precondition, or @c at @c start and @c over @c all
    /// conditions and @c at @c start effects of a durative action.
    Phase start;
    /// End: the effects, or @c at @c end and @c over @c all conditions and
    /// @c at @c end effects of a durative action.
    Phase end;
};

// ****************************************************************************
/// What RobotIK reads of a PDDL domain.
///
/// @code
/// robotik::DomainSchema const domain = robotik::DomainSchema::parse(text);
/// bool const ok = domain.isA("cube", "block");
/// for (auto const& [name, action] : domain.actions)
///     std::cout << name << ": " << action.parameters.size() << "
///     parameters\n";
/// @endcode
// ****************************************************************************
struct DomainSchema
{
    // -------------------------------------------------------------------------
    /// Reads a domain. The planner checks the full syntax; this reads the
    /// sections above and ignores the rest.
    /// @param p_text PDDL text of the domain.
    /// @return The schema; fails on unbalanced parentheses or a missing
    ///   @c define / @c domain.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Result<DomainSchema> parse(std::string_view p_text);

    // -------------------------------------------------------------------------
    /// @param p_type Type to test.
    /// @param p_ancestor Possible ancestor.
    /// @return True when @p_type is @p_ancestor or derives from it.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool isA(std::string_view p_type,
                           std::string_view p_ancestor) const;

    /// Domain name.
    std::string name;
    /// Each type and its parent; the root is @c object.
    std::map<std::string, std::string, std::less<>> types;
    /// Constants of the domain and their type.
    std::map<std::string, std::string, std::less<>> constants;
    /// Predicates by name.
    std::map<std::string, PredicateSchema, std::less<>> predicates;
    /// Actions by name.
    std::map<std::string, ActionSchema, std::less<>> actions;
};

// ----------------------------------------------------------------------------
/// @param p_text Any text.
/// @return Lower case copy (PDDL names do not tell case apart).
// ----------------------------------------------------------------------------
[[nodiscard]] std::string lowerCase(std::string_view p_text);

} // namespace robotik
