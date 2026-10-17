// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Pddl.hpp
//! @brief Symbols of the planner and what RobotIK reads of a PDDL domain.
//!
//! The domain file is written in PDDL and solved by the planner. RobotIK
//! reads the part it needs to build problems and check the plan: types,
//! constants, predicates and the plain effects of the actions. Names are
//! lower case, as PDDL does not tell case apart.
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief Value of a predicate in the beliefs. Unknown is false for the
//! planner (closed world) and is not a failure when an effect is checked.
// ****************************************************************************
enum class Truth : std::uint8_t
{
    False,
    True,
    Unknown,
};

[[nodiscard]] constexpr Truth truth(bool p_value)
{
    return p_value ? Truth::True : Truth::False;
}

[[nodiscard]] char const* toString(Truth p_truth);

// ****************************************************************************
//! @brief A predicate applied to objects: @c in(red_cube, box). In an action
//! schema an argument may be a parameter (@c ?o).
// ****************************************************************************
struct Atom
{
    std::string predicate;
    std::vector<std::string> args;

    //! @brief @c "(in red_cube box)".
    [[nodiscard]] std::string toPddl() const;
    //! @brief @c "in(red_cube, box)", or @c "hand_empty" without argument.
    [[nodiscard]] std::string toString() const;

    auto operator<=>(Atom const&) const = default;
};

// ****************************************************************************
//! @brief An atom or its negation.
// ****************************************************************************
struct Literal
{
    Atom atom;
    bool positive = true;

    // -------------------------------------------------------------------------
    //! @brief Reads the scenario syntax: @c "in(red_cube, box)",
    //! @c "not in(red_cube, box)" or @c "hand_empty". Names are lower-cased.
    //! @throws std::runtime_error on a malformed literal.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Literal parse(std::string_view p_text);

    //! @brief @c "(not (in red_cube box))".
    [[nodiscard]] std::string toPddl() const;
    //! @brief @c "not in(red_cube, box)".
    [[nodiscard]] std::string toString() const;

    auto operator<=>(Literal const&) const = default;
};

// ****************************************************************************
//! @brief One action of a plan: @c (pick red_cube).
// ****************************************************************************
struct GroundAction
{
    std::string name;
    std::vector<std::string> args;

    //! @brief @c "(pick red_cube)".
    [[nodiscard]] std::string toPddl() const;

    bool operator==(GroundAction const&) const = default;
};

//! @brief A typed parameter of a predicate or an action (name without @c ?).
struct Parameter
{
    std::string name;
    std::string type;
};

struct PredicateSchema
{
    std::string name;
    //!< Type of each argument.
    std::vector<std::string> types;
    //!< Defined by a @c :derived rule: the planner computes it.
    bool derived = false;
};

struct ActionSchema
{
    std::string name;
    std::vector<Parameter> parameters;
    //!< Plain effects (@c at @c end for a durative action): what the executor
    //!< checks once the action is done. Arguments are parameters (@c ?o) or
    //!< constants.
    std::vector<Literal> effects;
    //!< Some effects are conditional, quantified or numeric: not checked.
    bool partial = false;
    bool durative = false;
};

// ****************************************************************************
//! @brief What RobotIK reads of a PDDL domain.
// ****************************************************************************
struct DomainSchema
{
    // -------------------------------------------------------------------------
    //! @brief Reads a domain. The planner checks the full syntax; this reads
    //! the sections above and ignores the rest.
    //! @throws std::runtime_error on unbalanced parentheses or a missing
    //! @c define / @c domain.
    // -------------------------------------------------------------------------
    [[nodiscard]] static DomainSchema parse(std::string_view p_text);

    //! @brief True when @p_type is @p_ancestor or derives from it.
    [[nodiscard]] bool isA(std::string_view p_type,
                           std::string_view p_ancestor) const;

    std::string name;
    //!< Each type and its parent; the root is @c object.
    std::map<std::string, std::string, std::less<>> types;
    //!< Constants of the domain and their type.
    std::map<std::string, std::string, std::less<>> constants;
    std::map<std::string, PredicateSchema, std::less<>> predicates;
    std::map<std::string, ActionSchema, std::less<>> actions;
};

//! @brief Lower case copy (PDDL names).
[[nodiscard]] std::string lowerCase(std::string_view p_text);

} // namespace robotik
