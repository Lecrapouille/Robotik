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
///   - { condition: "collision_count(robot) == 0", phase: always }
///   - { condition: "pose_error(easel) < 0.002", phase: always }
///   - { condition: "inside(red_cube, box)", phase: end }
///   - { condition: "distance(robot.tcp, red_cube) < 0.05", within: 5.0 }
///   - { condition: "holding(gripper)", eventually: true }
///   - { condition: "time > 1", at: 2.0 }
///   - name: attached before painting
///     sequence:
///       - tool_is_attached(brush, robot.flange)
///       - skill_started(mark)
///   - { condition: "joint_limits_respected(robot)", phase: always, stop: true
///   }
/// @endcode
///
/// A failed assertion keeps its measured value, the expected bound and the
/// simulated time; it does not stop the run unless it says @c stop: true.
#pragma once

#include "Robotik/Math/Si.hpp"
#include "Robotik/Runtime/Metrics.hpp"

#include <cstdint>
#include <functional>
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
/// spec.condition = "collision_count(robot) == 0";
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
    /// A failure ends the run.
    bool stop = false;

    // -------------------------------------------------------------------------
    /// Reads one entry of @c assertions:.
    /// @param p_node A string (shorthand for phase end) or a map, see the
    ///   file doc.
    /// @return The assertion.
    /// @throws std::runtime_error on an invalid entry.
    // -------------------------------------------------------------------------
    [[nodiscard]] static AssertionSpec read(ConfigNode const& p_node);
};

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

    /// @return True to show it green in a live view: passed, or pending and
    ///   holding now.
    [[nodiscard]] bool ok() const
    {
        return verdict == Verdict::Passed ||
               (verdict == Verdict::Pending && holds);
    }
};

// ----------------------------------------------------------------------------
/// @param p_verdict Verdict to name.
/// @return Its name, for reports.
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(Check::Verdict p_verdict);

/// Evaluates one condition on the current state (see @ref evaluate).
using ConditionProbe = std::function<Evaluation(std::string_view)>;

// ****************************************************************************
/// Judges a scenario's assertions over a run.
///
/// The @ref Simulation owns one and calls it every step. The verdicts live
/// here, never in plugin C++ code.
///
/// @code
/// robotik::AssertionEngine engine(scenario.assertions);
/// engine.reset();
/// auto probe = [&](std::string_view p_condition) {
///     return robotik::evaluate(p_condition, metrics, resolver);
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
    /// Sets a failed verdict with its reason.
    void fail(Check& p_check, Seconds p_now, std::string p_why);

    /// Declared assertions.
    std::vector<AssertionSpec> m_specs;
    /// State of each assertion.
    std::vector<Check> m_checks;
    /// Next condition expected by each sequence.
    std::vector<std::size_t> m_progress;
    /// @ref conclude was called.
    bool m_concluded = false;
};

} // namespace robotik
