// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Assertions.hpp
//! @brief What a scenario checks on the world, and when.
//!
//! @code
//! assertions:
//!   - controller.success                       # shorthand: phase end
//!   - { condition: "collision_count(robot) == 0", phase: always }
//!   - { condition: "pose_error(easel) < 0.002", phase: always }
//!   - { condition: "inside(red_cube, box)", phase: end }
//!   - { condition: "distance(robot.tcp, red_cube) < 0.05", within: 5.0 }
//!   - { condition: "holding(gripper)", eventually: true }
//!   - { condition: "time > 1", at: 2.0 }
//!   - name: attached before painting
//!     sequence:
//!       - tool_is_attached(brush, robot.flange)
//!       - skill_started(mark)
//!   - { condition: "joint_limits_respected(robot)", phase: always, stop: true }
//! @endcode
//!
//! A failed assertion keeps its measured value, the expected bound and the
//! simulated time; it does not stop the run unless it says @c stop: true.
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
//! @brief When a condition is judged.
// ****************************************************************************
enum class AssertionPhase : std::uint8_t
{
    Always,     //!< Must hold at every step; the first breach fails it.
    End,        //!< Judged when the run ends (shown live meanwhile).
    At,         //!< Judged once, at @ref AssertionSpec::time.
    Within,     //!< Must become true before @ref AssertionSpec::time.
    Eventually, //!< Must become true before the run ends.
    Sequence,   //!< Conditions must first become true in the listed order.
};

[[nodiscard]] char const* toString(AssertionPhase p_phase);

// ****************************************************************************
//! @brief One declared assertion.
// ****************************************************************************
struct AssertionSpec
{
    //!< Shown in reports; the condition by default.
    std::string name;
    AssertionPhase phase = AssertionPhase::End;
    //!< Empty for a sequence.
    std::string condition;
    std::vector<std::string> sequence;
    //!< Instant of @ref AssertionPhase::At, deadline of Within.
    Seconds time{};
    //!< A failure ends the run.
    bool stop = false;

    //! @brief A string (shorthand for phase end) or a map, see the file doc.
    //! @throws std::runtime_error on an invalid entry.
    [[nodiscard]] static AssertionSpec read(ConfigNode const& p_node);
};

// ****************************************************************************
//! @brief State of one assertion.
// ****************************************************************************
struct Check
{
    enum class Verdict : std::uint8_t
    {
        Pending,
        Passed,
        Failed,
    };

    std::string text;
    AssertionPhase phase = AssertionPhase::End;
    Verdict verdict = Verdict::Pending;
    //!< Last evaluation: the condition holds now.
    bool holds = false;
    //!< Value measured at the verdict (or the last one while pending).
    double value = 0.0;
    std::string expected;
    std::string detail;
    //!< Simulated time of the verdict, or of the last evaluation.
    Seconds time{};

    //! @brief Green in a live view: passed, or pending and holding now.
    [[nodiscard]] bool ok() const
    {
        return verdict == Verdict::Passed || (verdict == Verdict::Pending && holds);
    }
};

[[nodiscard]] char const* toString(Check::Verdict p_verdict);

//! @brief Measures one condition on the current state.
using ConditionProbe = std::function<Measurement(std::string_view)>;

// ****************************************************************************
//! @brief Judges a scenario's assertions over a run.
// ****************************************************************************
class AssertionEngine
{
public:

    AssertionEngine() = default;
    explicit AssertionEngine(std::vector<AssertionSpec> p_specs);

    //! @brief Every assertion back to pending (episode reset).
    void reset();

    //! @brief Evaluates the assertions due at @p_now (after a step).
    void update(Seconds p_now, ConditionProbe const& p_probe);

    //! @brief Gives every pending assertion its final verdict. Idempotent.
    void conclude(Seconds p_now, ConditionProbe const& p_probe);

    [[nodiscard]] bool concluded() const
    {
        return m_concluded;
    }

    [[nodiscard]] std::vector<Check> const& checks() const
    {
        return m_checks;
    }

    [[nodiscard]] std::vector<AssertionSpec> const& specs() const
    {
        return m_specs;
    }

    //! @brief An assertion declared @c stop: true has failed.
    [[nodiscard]] bool stopRequested() const;

    //! @brief Concluded and every assertion passed.
    [[nodiscard]] bool passed() const;

private:

    void judge(std::size_t p_index, Seconds p_now, ConditionProbe const& p_probe, bool p_final);
    void fail(Check& p_check, Seconds p_now, std::string p_why);

    std::vector<AssertionSpec> m_specs;
    std::vector<Check> m_checks;
    //!< Next condition expected by each sequence.
    std::vector<std::size_t> m_progress;
    bool m_concluded = false;
};

} // namespace robotik
