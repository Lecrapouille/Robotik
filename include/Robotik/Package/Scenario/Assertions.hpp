// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Assertions.hpp
/// What a scenario checks on the world, and when.
///
/// @code
/// assertions:
///   - controller.success                       # shorthand: phase end
///   - { condition: (== (collision_count robot) 0), phase: always }
///   - { condition: (< (pose_error easel) 0.002), phase: always }
///   - { condition: (inside red_cube box), phase: end }
///   - { condition: (< (distance robot.tcp red_cube) 0.05), within: 5.0 }
///   - { condition: (< (belief_error red_cube) 0.01), phase: always,
///       for: 0.5, unknown: skip }
///   - { condition: (holding gripper), eventually: true }
///   - { condition: "time > 1", at: 2.0 }
///   - name: attached before painting
///     sequence:
///       - (tool_is_attached brush robot.flange)
///       - (skill_started mark)
///   - { condition: (joint_limits_respected robot), phase: always,
///       stop: true }
/// @endcode
///
/// A failed assertion keeps its measured value, the expected bound and the
/// simulated time; it does not stop the run unless it says @c stop: true.
///
/// @par Contract
/// - Promises: assertions only read metrics and truth queries; @c readAssertion
///   returns a @c Result.
/// - Expects: conditions name metrics the scenario logic or the cell publish.
#pragma once

#include "Robotik/Common/Result.hpp"
#include "Robotik/Common/Math.hpp"
#include "Robotik/Task/Condition.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

class ConfigNode;

// ****************************************************************************
/// When a condition is judged.
// ****************************************************************************
enum class AssertionPhase : std::uint8_t
{
    Always,     ///< Must hold at every step; the first breach fails it.
    End,        ///< Judged when the run ends (shown live meanwhile).
    At,         ///< Judged once, at @ref AssertionSpec::time.
    Within,     ///< Must become true before @ref AssertionSpec::time.
    Eventually, ///< Must become true before the run ends.
    Sequence,   ///< Conditions must first become true in the listed order.
};

// ----------------------------------------------------------------------------
/// @param p_phase Phase to name.
/// @return Its name, for reports.
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(AssertionPhase p_phase);

// ****************************************************************************
/// One declared assertion, as written in the scenario.
///
/// @code
/// robotik::AssertionSpec spec;
/// spec.condition = "(== (collision_count robot) 0)";
/// spec.phase = robotik::AssertionPhase::Always;
/// spec.stop = true;
/// @endcode
// ****************************************************************************
struct AssertionSpec
{
    /// Shown in reports; the condition by default.
    std::string name;
    /// When the condition is judged.
    AssertionPhase phase = AssertionPhase::End;
    /// Empty for a sequence.
    std::string condition;
    /// Conditions of a @ref AssertionPhase::Sequence, in order.
    std::vector<std::string> sequence;
    /// Instant of @ref AssertionPhase::At, deadline of Within.
    Seconds time{};
    /// @ref AssertionPhase::Always only: how long the condition may stay
    /// false in a row before it fails (@c for: in YAML). For the short
    /// transients of a belief: the label one step late, a perception
    /// latency.
    Seconds tolerance{};
    /// A step where the condition has no answer is not judged (@c unknown:
    /// @c skip), instead of failing it. A condition never answered still
    /// fails at the end.
    bool skip_unknown = false;
    /// A failure ends the run.
    bool stop = false;
};

// ----------------------------------------------------------------------------
/// Reads one entry of @c assertions:.
/// @param p_node A string (shorthand for phase end) or a map, see the file
///   doc.
/// @return The assertion; fails on an invalid entry.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<AssertionSpec> readAssertion(ConfigNode const& p_node);

// ****************************************************************************
/// State of one assertion.
///
/// @code
/// for (robotik::Check const& c : engine.checks())
///     if (c.verdict == robotik::Check::Verdict::Failed)
///         std::cout << c.text << ": " << c.detail << ", expected " <<
///         c.expected
///                   << " at " << c.time.value() << " s\n";
/// @endcode
// ****************************************************************************
struct Check
{
    // ------------------------------------------------------------------------
    /// Final or temporary result.
    // ------------------------------------------------------------------------
    enum class Verdict : std::uint8_t
    {
        Pending, ///< Not judged yet.
        Passed,  ///< Judged true.
        Failed,  ///< Judged false.
    };

    /// Name of the assertion, shown in reports.
    std::string text;
    /// When the condition is judged.
    AssertionPhase phase = AssertionPhase::End;
    /// Current verdict.
    Verdict verdict = Verdict::Pending;
    /// Last evaluation: the condition holds now.
    bool holds = false;
    /// Value measured at the verdict (or the last one while pending).
    double value = 0.0;
    /// Right-hand side, e.g. "< 0.002".
    std::string expected;
    /// "name = value", or why it failed.
    std::string detail;
    /// Simulated time of the verdict, or of the last evaluation.
    Seconds time{};
};

/// @return True to show it green in a live view: passed, or pending and
///   holding now.
[[nodiscard]] inline bool ok(Check const& p_check)
{
    return p_check.verdict == Check::Verdict::Passed ||
           (p_check.verdict == Check::Verdict::Pending && p_check.holds);
}

// ----------------------------------------------------------------------------
/// @param p_verdict Verdict to name.
/// @return Its name, for reports.
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(Check::Verdict p_verdict);

/// Evaluates condition number @c i of @ref AssertionEngine::conditions on the
/// current state; the caller compiles them once (see Condition.hpp).
using ConditionProbe = std::function<Evaluation(std::size_t)>;

// ****************************************************************************
/// Judges a scenario's assertions over a run.
///
/// A test of a robot is rarely "true at the end": a collision must never
/// happen, the tool must arrive within 5 s, the skills must run in an
/// order. The engine turns each assertion into one of these moments
/// (@c always, @c at, @c within, @c eventually, @c sequence) and keeps its
/// verdict up to date at each step, so the interface shows it turning green
/// or red during the run. The @ref Simulation owns one and calls it every
/// step. The verdicts live here, never in plugin C++ code.
///
/// The engine reads no text: it asks the caller for the value of condition
/// @c i through a probe, so the conditions are compiled once by the cell.
/// A condition that cannot be measured at a step (a metric not published)
/// counts as unknown, never as false.
///
/// @code
/// robotik::AssertionEngine engine(scenario.assertions);
/// std::vector<robotik::Condition> compiled;
/// for (std::string const& text : engine.conditions())
///     compiled.push_back(cell.compile(text).take());
/// auto probe = [&](std::size_t p_condition) {
///     return robotik::evaluate(compiled[p_condition], cell.metrics());
/// };
/// while (running && !engine.stopRequested())
///     engine.update(now, probe);
/// engine.conclude(now, probe);
/// bool const ok = engine.passed();
/// @endcode
// ****************************************************************************
class AssertionEngine
{
public:

    AssertionEngine() = default;

    /// @param p_specs Assertions of the scenario.
    explicit AssertionEngine(std::vector<AssertionSpec> p_specs);

    /// Puts every assertion back to pending (episode reset).
    void reset();

    // -------------------------------------------------------------------------
    /// Evaluates the assertions due now (after a step).
    /// @param p_now Simulated time.
    /// @param p_probe Evaluates a condition on the current state.
    // -------------------------------------------------------------------------
    void update(Seconds p_now, ConditionProbe const& p_probe);

    // -------------------------------------------------------------------------
    /// Gives every pending assertion its final verdict (the run ends).
    /// Calling it again changes nothing.
    /// @param p_now Simulated time at the end.
    /// @param p_probe Evaluates a condition on the final state.
    // -------------------------------------------------------------------------
    void conclude(Seconds p_now, ConditionProbe const& p_probe);

    /// @return True once @ref conclude was called.
    [[nodiscard]] bool concluded() const
    {
        return m_concluded;
    }

    /// @return The state of each assertion, aligned with @ref specs.
    [[nodiscard]] std::vector<Check> const& checks() const
    {
        return m_checks;
    }

    /// @return The declared assertions.
    [[nodiscard]] std::vector<AssertionSpec> const& specs() const
    {
        return m_specs;
    }

    // -------------------------------------------------------------------------
    /// @return Every condition of the assertions, in order: the condition of
    ///   each assertion, or the items of a sequence. A @ref ConditionProbe
    ///   receives an index in this list.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<std::string> const& conditions() const
    {
        return m_conditions;
    }

    /// @return True when an assertion declared @c stop: true has failed.
    [[nodiscard]] bool stopRequested() const;

    /// @return True when concluded and every assertion passed.
    [[nodiscard]] bool passed() const;

private:

    /// Evaluates assertion @p_index; @p_final gives pending ones a verdict.
    void judge(std::size_t p_index,
               Seconds p_now,
               ConditionProbe const& p_probe,
               bool p_final);
    /// Judges an @c always with its tolerance (@ref AssertionSpec::tolerance).
    void
    judgeAlways(std::size_t p_index, Seconds p_now, bool p_holds, bool p_final);
    /// Sets a failed verdict with its reason.
    void fail(Check& p_check, Seconds p_now, std::string p_why);

    /// Declared assertions.
    std::vector<AssertionSpec> m_specs;
    /// State of each assertion.
    std::vector<Check> m_checks;
    /// Next condition expected by each sequence.
    std::vector<std::size_t> m_progress;
    /// Start of the current run of false steps of each @c always (with a
    /// tolerance), or nothing while it holds.
    std::vector<std::optional<Seconds>> m_breach;
    /// The condition had an answer once (for @c unknown: @c skip).
    std::vector<bool> m_answered;
    /// Every condition, see @ref conditions.
    std::vector<std::string> m_conditions;
    /// Index in @ref m_conditions of the first condition of each assertion.
    std::vector<std::size_t> m_first;
    /// @ref conclude was called.
    bool m_concluded = false;
};

} // namespace robotik
