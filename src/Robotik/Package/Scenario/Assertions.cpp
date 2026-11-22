// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/Scenario/Assertions.hpp"

#include "Robotik/Common/Config.hpp"

#include <algorithm>

namespace robotik
{

char const* toString(AssertionPhase p_phase)
{
    switch (p_phase)
    {
        case AssertionPhase::Always:
            return "always";
        case AssertionPhase::End:
            return "end";
        case AssertionPhase::At:
            return "at";
        case AssertionPhase::Within:
            return "within";
        case AssertionPhase::Eventually:
            return "eventually";
        case AssertionPhase::Sequence:
            return "sequence";
    }
    return "?";
}

char const* toString(Check::Verdict p_verdict)
{
    switch (p_verdict)
    {
        case Check::Verdict::Pending:
            return "PENDING";
        case Check::Verdict::Passed:
            return "PASS";
        case Check::Verdict::Failed:
            return "FAIL";
    }
    return "?";
}

Result<AssertionSpec> readAssertion(ConfigNode const& p_node)
{
    AssertionSpec spec;
    if (p_node.isScalar())
    {
        spec.condition = p_node.scalar();
        spec.name = spec.condition;
        return spec;
    }
    if (!p_node.isMap())
    {
        return failure("an assertion is a condition or a map");
    }
    ROBOTIK_TRY(p_node.checkKeys({ "name",
                                   "condition",
                                   "phase",
                                   "at",
                                   "within",
                                   "eventually",
                                   "sequence",
                                   "for",
                                   "unknown",
                                   "stop" },
                                 "assertion"));
    spec.condition = p_node.text("condition");
    ROBOTIK_TRY_ASSIGN(spec.stop, p_node.flag("stop", false));
    std::string const phase = p_node.text("phase");
    bool eventually = false;
    ROBOTIK_TRY_ASSIGN(eventually, p_node.flag("eventually", false));
    double time = 0.0;
    if (p_node.has("sequence"))
    {
        spec.phase = AssertionPhase::Sequence;
        spec.sequence = p_node.strings("sequence");
        if (spec.sequence.size() < 2U || !spec.condition.empty())
        {
            return failure(
                "a sequence lists two conditions or more, and no condition");
        }
    }
    else if (p_node.has("at"))
    {
        spec.phase = AssertionPhase::At;
        ROBOTIK_TRY_ASSIGN(time, p_node.number("at", 0.0));
        spec.time = Seconds(time);
    }
    else if (p_node.has("within"))
    {
        spec.phase = AssertionPhase::Within;
        ROBOTIK_TRY_ASSIGN(time, p_node.number("within", 0.0));
        spec.time = Seconds(time);
    }
    else if (eventually || phase == "eventually")
    {
        spec.phase = AssertionPhase::Eventually;
    }
    else if (phase == "always")
    {
        spec.phase = AssertionPhase::Always;
    }
    else if (phase.empty() || phase == "end")
    {
        spec.phase = AssertionPhase::End;
    }
    else
    {
        return failure("phase must be always, end or eventually (or use at:, "
                       "within:, sequence:)");
    }
    if (p_node.has("for"))
    {
        if (spec.phase != AssertionPhase::Always)
        {
            return failure("for: is a tolerance of phase: always only");
        }
        double tolerance = 0.0;
        ROBOTIK_TRY_ASSIGN(tolerance, p_node.number("for", 0.0));
        if (!(tolerance >= 0.0))
        {
            return failure("for: is a duration in seconds, 0 or more");
        }
        spec.tolerance = Seconds(tolerance);
    }
    if (std::string const unknown = p_node.text("unknown"); unknown == "skip")
    {
        spec.skip_unknown = true;
    }
    else if (!unknown.empty() && unknown != "fail")
    {
        return failure("unknown: must be fail (default) or skip");
    }
    if (spec.phase != AssertionPhase::Sequence && spec.condition.empty())
    {
        return failure("an assertion needs a condition");
    }
    spec.name = p_node.text("name");
    if (spec.name.empty())
    {
        spec.name = spec.phase == AssertionPhase::Sequence ? "sequence"
                                                           : spec.condition;
    }
    return spec;
}

AssertionEngine::AssertionEngine(std::vector<AssertionSpec> p_specs)
    : m_specs(std::move(p_specs))
{
    for (AssertionSpec const& spec : m_specs)
    {
        m_first.push_back(m_conditions.size());
        if (spec.phase == AssertionPhase::Sequence)
        {
            m_conditions.insert(m_conditions.end(),
                                spec.sequence.begin(),
                                spec.sequence.end());
        }
        else
        {
            m_conditions.push_back(spec.condition);
        }
    }
    reset();
}

void AssertionEngine::reset()
{
    m_checks.clear();
    for (AssertionSpec const& spec : m_specs)
    {
        Check check;
        check.text = spec.name;
        check.phase = spec.phase;
        if (spec.phase == AssertionPhase::Sequence)
        {
            check.expected = "in order";
        }
        m_checks.push_back(std::move(check));
    }
    m_progress.assign(m_specs.size(), 0U);
    m_breach.assign(m_specs.size(), std::nullopt);
    m_answered.assign(m_specs.size(), false);
    m_concluded = false;
}

bool AssertionEngine::stopRequested() const
{
    for (std::size_t i = 0; i < m_specs.size(); ++i)
    {
        if (m_specs[i].stop && m_checks[i].verdict == Check::Verdict::Failed)
        {
            return true;
        }
    }
    return false;
}

bool AssertionEngine::passed() const
{
    return m_concluded &&
           std::ranges::all_of(
               m_checks,
               [](Check const& p_check)
               { return p_check.verdict == Check::Verdict::Passed; });
}

void AssertionEngine::fail(Check& p_check, Seconds p_now, std::string p_why)
{
    p_check.verdict = Check::Verdict::Failed;
    p_check.time = p_now;
    if (!p_why.empty())
    {
        p_check.detail = std::move(p_why);
    }
}

void AssertionEngine::judgeAlways(std::size_t p_index,
                                  Seconds p_now,
                                  bool p_holds,
                                  bool p_final)
{
    AssertionSpec const& spec = m_specs[p_index];
    Check& check = m_checks[p_index];
    std::optional<Seconds>& breach = m_breach[p_index];
    if (p_holds)
    {
        breach.reset();
    }
    else if (!breach)
    {
        breach = p_now;
    }
    // Without a tolerance, the first false step fails; with one, a run of
    // false steps longer than it.
    if (breach &&
        (spec.tolerance <= Seconds{} || p_now - *breach > spec.tolerance))
    {
        fail(check,
             spec.tolerance <= Seconds{} ? p_now : *breach,
             spec.tolerance <= Seconds{}
                 ? std::string{}
                 : check.detail + ", false for more than " +
                       std::to_string(spec.tolerance.value()) + " s");
        return;
    }
    if (p_final)
    {
        check.verdict = Check::Verdict::Passed;
    }
}

void AssertionEngine::judge(std::size_t p_index,
                            Seconds p_now,
                            ConditionProbe const& p_probe,
                            bool p_final)
{
    AssertionSpec const& spec = m_specs[p_index];
    Check& check = m_checks[p_index];
    if (check.verdict != Check::Verdict::Pending)
    {
        return;
    }

    if (spec.phase == AssertionPhase::Sequence)
    {
        std::size_t& next = m_progress[p_index];
        // The next expected condition may become true; a later one must not
        // become true first.
        while (next < spec.sequence.size())
        {
            Evaluation const step = p_probe(m_first[p_index] + next);
            if (!step.known)
            {
                fail(check, p_now, step.detail);
                return;
            }
            if (!step.holds)
            {
                break;
            }
            ++next;
        }
        for (std::size_t later = next + 1U; later < spec.sequence.size();
             ++later)
        {
            if (p_probe(m_first[p_index] + later).holds)
            {
                fail(check,
                     p_now,
                     "'" + spec.sequence[later] + "' before '" +
                         spec.sequence[next] + "'");
                return;
            }
        }
        check.time = p_now;
        check.value = static_cast<double>(next);
        check.holds = next == spec.sequence.size();
        check.detail = std::to_string(next) + "/" +
                       std::to_string(spec.sequence.size()) + " reached";
        if (check.holds)
        {
            check.verdict = Check::Verdict::Passed;
        }
        else if (p_final)
        {
            fail(check, p_now, "'" + spec.sequence[next] + "' never came");
        }
        return;
    }

    if (spec.phase == AssertionPhase::At && p_now < spec.time && !p_final)
    {
        return;
    }
    Evaluation const measured = p_probe(m_first[p_index]);
    if (!measured.known && spec.skip_unknown &&
        (!p_final ||
         (spec.phase == AssertionPhase::Always && m_answered[p_index])))
    {
        // Not judged at this step; an always that had answers ends on them.
        check.time = p_now;
        if (p_final)
        {
            judgeAlways(p_index, p_now, true, true);
        }
        return;
    }
    m_answered[p_index] = m_answered[p_index] || measured.known;
    check.holds = measured.holds;
    check.value = measured.value;
    check.expected = measured.expected;
    check.detail = measured.detail;
    check.time = p_now;
    if (!measured.known)
    {
        fail(check, p_now, {});
        return;
    }

    switch (spec.phase)
    {
        case AssertionPhase::Always:
            judgeAlways(p_index, p_now, measured.holds, p_final);
            break;
        case AssertionPhase::End:
            if (p_final)
            {
                check.verdict = measured.holds ? Check::Verdict::Passed
                                               : Check::Verdict::Failed;
            }
            break;
        case AssertionPhase::At:
            if (p_now < spec.time)
            {
                fail(check,
                     p_now,
                     "the run ended before " +
                         std::to_string(spec.time.value()) + " s");
            }
            else
            {
                check.verdict = measured.holds ? Check::Verdict::Passed
                                               : Check::Verdict::Failed;
            }
            break;
        case AssertionPhase::Within:
            if (measured.holds && p_now <= spec.time)
            {
                check.verdict = Check::Verdict::Passed;
            }
            else if (p_now > spec.time || p_final)
            {
                fail(check,
                     p_now,
                     check.detail + ", not true within " +
                         std::to_string(spec.time.value()) + " s");
            }
            break;
        case AssertionPhase::Eventually:
            if (measured.holds)
            {
                check.verdict = Check::Verdict::Passed;
            }
            else if (p_final)
            {
                fail(check, p_now, check.detail + ", never true");
            }
            break;
        case AssertionPhase::Sequence:
            break;
    }
}

void AssertionEngine::update(Seconds p_now, ConditionProbe const& p_probe)
{
    if (m_concluded)
    {
        return;
    }
    for (std::size_t i = 0; i < m_specs.size(); ++i)
    {
        judge(i, p_now, p_probe, false);
    }
}

void AssertionEngine::conclude(Seconds p_now, ConditionProbe const& p_probe)
{
    if (m_concluded)
    {
        return;
    }
    for (std::size_t i = 0; i < m_specs.size(); ++i)
    {
        judge(i, p_now, p_probe, true);
    }
    m_concluded = true;
}

} // namespace robotik
