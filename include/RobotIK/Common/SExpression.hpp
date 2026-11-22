// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file SExpression.hpp
/// Reads the Lisp notation of RobotIK conditions, the one of PDDL.
///
/// A scenario writes conditions such as
/// @c (< (distance robot.tcp red_cube) 0.05). PDDL domains use the same
/// notation, so a reader learns one way to write a fact. The notation has
/// three kinds of word only (a name or number, a quoted text, a list in
/// parentheses), so the reader is small and its errors point to a column.
///
/// How: @ref robotik::parseSExpression turns the text into a tree of
/// @ref robotik::SExpression; it does not know what the words mean. The
/// condition compiler (Task/Condition.hpp) gives them a meaning.
///
/// @code
/// robotik::SExpression const e =
///     robotik::parseSExpression("(and (inside cube box) (< time 30))").take();
/// // e.items[0].text == "and", e.items[1].items[0].text == "inside"
/// @endcode
///
/// @par Contract
/// - Promises: one expression per text; @c ; starts a comment up to the end
///   of the line; every node keeps where it starts and ends in the text.
/// - Errors: a missing or extra parenthesis, an empty list, an unclosed
///   quote or a second expression give a @c Result failure naming the
///   column (from 1).
#pragma once

#include "RobotIK/Common/Result.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// One node of an S-expression: a word, a quoted text or a list.
///
/// It keeps the place of the node in the text (@ref begin, @ref end), so a
/// later step (the condition compiler) can name the column of its own
/// errors. It knows nothing of conditions or PDDL: it is only the tree.
///
/// @code
/// // (distance robot.tcp "red cube")
/// // List{ Atom "distance", Atom "robot.tcp", Text "red cube" }
/// @endcode
// ****************************************************************************
struct SExpression
{
    /// What the node holds.
    enum class Kind : std::uint8_t
    {
        Atom, ///< A name or a number, written without quotes.
        Text, ///< A text between double quotes (quotes removed).
        List, ///< Nodes between parentheses.
    };

    /// What the node holds.
    Kind kind = Kind::Atom;
    /// The word of an atom or a text; empty for a list.
    std::string text;
    /// The nodes of a list.
    std::vector<SExpression> items;
    /// Offset of the first character in the source text.
    std::size_t begin = 0U;
    /// Offset after the last character in the source text.
    std::size_t end = 0U;
};

// ----------------------------------------------------------------------------
/// Reads one S-expression.
/// @param p_text Source text, e.g. @c "(< time 30)" or @c "controller.success".
/// @return The tree; fails with "column N: why" on a syntax error.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<SExpression> parseSExpression(std::string_view p_text);

// ----------------------------------------------------------------------------
/// @param p_word A word the reader does not know.
/// @param p_known Words the reader knows.
/// @return The known word closest to @p_word (a few letters changed), or
///   empty when none is close: the "did you mean" of the error messages.
// ----------------------------------------------------------------------------
[[nodiscard]] std::string closestWord(std::string_view p_word,
                                      std::vector<std::string_view> const& p_known);

} // namespace robotik
