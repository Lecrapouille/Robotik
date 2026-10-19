// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Scenario/Assertions.hpp"

#include "Robotik/Scenario/Config.hpp"

#include <algorithm>
#include <stdexcept>

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

AssertionSpec AssertionSpec::read(ConfigNode const& p_node)
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
        throw std::runtime_error("an assertion is a condition or a map");
    }
    spec.condition = p_node.text("condition");
    spec.stop = p_node.flag("stop", false);
    std::string const phase = p_node.text("phase");
    if (p_node.has("sequence"))
    {
        spec.phase = AssertionPhase::Sequence;
        spec.sequence = p_node.strings("sequence");
        if (spec.sequence.size() < 2u || !spec.condition.empty())
        {
            throw std::runtime_error(
                "a sequence lists two conditions or more, and no condition");
        }
    }
    else if (p_node.has("at"))
    {
        spec.phase = AssertionPhase::At;
        spec.time = Seconds(p_node.number("at", 0.0));
    }
    else if (p_node.has("within"))
    {
        spec.phase = AssertionPhase::Within;
        spec.time = Seconds(p_node.number("within", 0.0));
    }
    else if (p_node.flag("eventually", false) || phase == "eventually")
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
        throw std::runtime_error("phase must be always, end or eventually (or "
                                 "use at:, within:, sequence:)");
    }
    if (spec.phase != AssertionPhase::Sequence && spec.condition.empty())
    {
        throw std::runtime_error("an assertion needs a condition");
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
    m_progress.assign(m_specs.size(), 0u);
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
           std::all_of(m_checks.begin(),
                       m_checks.end(),
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
            Evaluation const step = p_probe(spec.sequence[next]);
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
        for (std::size_t later = next + 1u; later < spec.sequence.size();
             ++later)
        {
            if (p_probe(spec.sequence[later]).holds)
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
    Evaluation const measured = p_probe(spec.condition);
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
            if (!measured.holds)
            {
                fail(check, p_now, {});
            }
            else if (p_final)
            {
                check.verdict = Check::Verdict::Passed;
            }
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
