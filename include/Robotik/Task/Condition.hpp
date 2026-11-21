// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Condition.hpp
/// A condition of a scenario, read once and then evaluated at each step.
///
/// Assertions and @c stop_when ask a question about the run at every
/// step, such as "is the cube in the box, and before 30 s?". The text is
/// read when the scenario loads, so a typo stops the load (exit code 1)
/// instead of failing the run, and a step does not read text again.
///
/// The notation is the one of PDDL (see doc/reference/glossary.md:
/// "Condition"):
/// @code
/// controller.success                        ; a metric: true when not 0
/// (< time 30)                               ; a comparison
/// (< (distance robot.tcp red_cube) 0.05)    ; a world query
/// (and (inside red_cube box) (not (holding gripper)))
/// @endcode
///
/// Words: a metric is a number the run publishes (@c time, @c laps); a
/// query asks the world with arguments (@c (inside red_cube box)); the
/// comparisons are @c < @c <= @c > @c >= @c = @c == @c != ; @c and, @c or,
/// @c not combine conditions. A number or a value used as a condition holds
/// when it is not 0.
///
/// @par Contract
/// - Promises: @ref compileCondition resolves every name once: an unknown
///   metric, an unknown query, a wrong number of arguments or an argument
///   the query refuses (an unknown entity) gives a @c Result failure with
///   "did you mean". @ref evaluate reads no text.
/// - Expects: the metrics known at compile time are still published at
///   each step; a query does not change the world.
/// - Errors: a value missing at evaluation (a metric not published this
///   step, a query that throws) gives @c known false and the reason.
#pragma once

#include "Robotik/Common/Result.hpp"
#include "Robotik/Task/Metrics.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// One evaluation of a condition, kept for the reports and the interface.
///
/// @code
/// robotik::Evaluation const e = robotik::evaluate("(< time 60)", metrics);
/// if (!e.known)
///     std::cerr << e.detail << '\n';   // why it could not be measured
/// else if (!e.holds)
///     std::cerr << e.detail << ", expected " << e.expected << '\n';
/// @endcode
// ****************************************************************************
struct Evaluation
{
    /// False when the value could not be obtained (see @ref detail).
    bool known = false;
    /// The condition is true.
    bool holds = false;
    /// Measured value: the left side of a comparison, or the value itself.
    double value = 0.0;
    /// What was expected, e.g. "< 0.002", or "!= 0" for a bare value.
    std::string expected;
    /// "(distance robot.tcp red_cube) = 0.03", or why nothing was measured.
    std::string detail;
};

// ****************************************************************************
/// A question a condition may ask the world: its name, how many arguments
/// it takes, and the function that answers.
///
/// The world queries (@c inside, @c distance...) belong to the
/// @ref RoboticCell, which knows the entities; a plugin may add its own. The
/// compiler only needs this card to check a condition.
///
/// @code
/// robotik::ConditionQuery const near{
///     .name = "near", .min_args = 2U, .max_args = 2U,
///     .answer = [&cell](std::vector<std::string> const& p_args) {
///         return cell.requireQuery("distance", p_args).value() < 0.1 ? 1.0 : 0.0;
///     } };
/// @endcode
// ****************************************************************************
struct ConditionQuery
{
    /// Name used in conditions, e.g. @c "inside".
    std::string name;
    /// Fewest arguments.
    std::size_t min_args = 0U;
    /// Most arguments.
    std::size_t max_args = 0U;
    /// Answers for these arguments. Throws @c std::invalid_argument for an
    /// argument it refuses (unknown entity): the compiler calls it once to
    /// check the arguments, so it must not change the world.
    std::function<std::optional<double>(std::vector<std::string> const&)> answer;
};

// ****************************************************************************
/// What a condition may name: the metrics and the queries.
///
/// The compiler checks every name against this list, so a typo is found at
/// load and not at the end of a run. The @ref RoboticCell builds it from what it
/// measures (@c time, the plugin metrics, the planner counters) and from
/// its queries (@ref RoboticCell::vocabulary).
// ****************************************************************************
struct ConditionVocabulary
{
    /// Metric names published by the run.
    std::vector<std::string> metrics;
    /// Queries on the world.
    std::vector<ConditionQuery> queries;
};

// ****************************************************************************
/// One node of a compiled condition (flat tree, root first).
///
/// The nodes are stored in one array and point to their children by index,
/// so a condition is one allocation and an evaluation walks it without
/// reading text. A query node keeps its arguments, so a step does not parse
/// them again; the query itself still finds its entities by name at each
/// call.
// ****************************************************************************
struct ConditionNode
{
    /// What the node computes.
    enum class Op : std::uint8_t
    {
        Number,       ///< A constant.
        Metric,       ///< A published metric, read by name.
        Query,        ///< A world query with its arguments bound.
        Not,          ///< One child.
        And,          ///< All children hold.
        Or,           ///< A child holds.
        Less,         ///< Two children.
        LessEqual,    ///< Two children.
        Greater,      ///< Two children.
        GreaterEqual, ///< Two children.
        Equal,        ///< Two children, within 1e-9.
        NotEqual,     ///< Two children, beyond 1e-9.
    };

    /// What the node computes.
    Op op = Op::Number;
    /// Value of a @c Number.
    double number = 0.0;
    /// Name of a @c Metric.
    std::string metric;
    /// Bound query of a @c Query: no text is read when it is called.
    std::function<std::optional<double>()> query;
    /// Index of the children in @ref Condition::nodes.
    std::vector<std::uint32_t> children;
    /// Source text of the node, for the messages.
    std::string text;
};

// ****************************************************************************
/// A condition read and checked once (see @ref compileCondition).
///
/// @code
/// robotik::Condition const c =
///     robotik::compileCondition("(> laps 2)", vocabulary).take();
/// robotik::Evaluation const e = robotik::evaluate(c, metrics);
/// @endcode
// ****************************************************************************
struct Condition
{
    /// Text as written in the scenario.
    std::string text;
    /// Nodes; the root is the first one. Empty: no condition.
    std::vector<ConditionNode> nodes;
};

// ----------------------------------------------------------------------------
/// Reads and checks a condition.
/// @param p_text Condition, e.g. @c "(< (distance robot.tcp cube) 0.05)".
/// @param p_vocabulary Metrics and queries it may name.
/// @return The condition; fails on a syntax error, an unknown name, a
///   wrong number of arguments, or an argument a query refuses.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<Condition>
compileCondition(std::string_view p_text, ConditionVocabulary const& p_vocabulary);

// ----------------------------------------------------------------------------
/// Evaluates a compiled condition.
/// @param p_condition The condition.
/// @param p_metrics Metrics published this step.
/// @return What was measured and whether it holds.
// ----------------------------------------------------------------------------
[[nodiscard]] Evaluation evaluate(Condition const& p_condition,
                                  Metrics const& p_metrics);

// ----------------------------------------------------------------------------
/// Reads and evaluates a condition on a table of metrics, without queries
/// (tests, a one-off question). A condition that does not compile gives
/// @c known false and the reason.
/// @param p_text Condition, e.g. @c "(< time 60)".
/// @param p_metrics Metrics; their names are the vocabulary.
/// @return What was measured and whether it holds.
// ----------------------------------------------------------------------------
[[nodiscard]] Evaluation evaluate(std::string_view p_text,
                                  Metrics const& p_metrics);

} // namespace robotik
