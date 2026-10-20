// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/RunReport.hpp"

#include "Robotik/Task/Planning/PlanExecutor.hpp"
#include "Robotik/Package/Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <ostream>
#include <sstream>

namespace robotik
{

/// Escapes a string as a JSON string.
static std::string quoted(std::string_view p_text)
{
    std::string result = "\"";
    for (char const character : p_text)
    {
        switch (character)
        {
            case '"':
                result += "\\\"";
                break;
            case '\\':
                result += "\\\\";
                break;
            case '\n':
                result += "\\n";
                break;
            case '\t':
                result += "\\t";
                break;
            default:
                if (static_cast<unsigned char>(character) < 0x20u)
                {
                    std::ostringstream code;
                    code << "\\u" << std::hex << std::setw(4)
                         << std::setfill('0') << static_cast<int>(character);
                    result += code.str();
                }
                else
                {
                    result += character;
                }
        }
    }
    return result + "\"";
}

/// Prints a finite number, or null when it is not.
static std::string number(double p_value)
{
    if (!std::isfinite(p_value))
    {
        return "null";
    }
    std::ostringstream stream;
    stream << std::setprecision(10) << p_value;
    return stream.str();
}

RunRecord RunRecord::capture(Simulation const& p_simulation,
                             std::string p_case,
                             std::vector<std::string> const& p_metrics)
{
    RunRecord record;
    record.case_id = std::move(p_case);
    record.scenario = p_simulation.scenario().id;
    record.seed = p_simulation.seed().value;
    record.outcome = toString(p_simulation.outcome());
    record.time = p_simulation.time().value();
    record.passed = p_simulation.passed();
    record.oracle = p_simulation.cell().oracle();
    PhysicsSpec const& physics = p_simulation.scenario().physics;
    record.physics = toString(physics.fidelity);
    for (auto const& [id, level] : physics.overrides)
    {
        record.physics += ", " + id + ": " + toString(level);
    }
    record.checks = p_simulation.checks();
    if (PlanExecutor const* planner = p_simulation.cell().planner())
    {
        PlannerSummary& summary = record.planner;
        summary.used = true;
        summary.state = toString(planner->state());
        summary.replans = planner->replans();
        summary.failure = planner->failure();
        for (PlanningRecord const& plan : planner->history())
        {
            PlanSummary entry;
            entry.time = plan.time.value();
            entry.cause = plan.cause;
            entry.result = toString(plan.result.status);
            for (GroundAction const& step : plan.result.steps)
            {
                entry.steps.push_back(Atom{ step.name, step.args }.toString());
            }
            entry.expanded = plan.result.expanded;
            entry.milliseconds = plan.result.milliseconds;
            summary.plans.push_back(std::move(entry));
        }
    }
    for (std::string const& name : p_metrics)
    {
        Evaluation const measured = p_simulation.evaluate(name);
        record.metrics.emplace_back(name,
                                    measured.known
                                        ? std::optional<double>(measured.value)
                                        : std::nullopt);
    }
    return record;
}

bool RunReport::passed() const
{
    return !runs.empty() &&
           std::all_of(runs.begin(),
                       runs.end(),
                       [](RunRecord const& p_run) { return p_run.passed; });
}

void RunReport::writeText(std::ostream& p_out) const
{
    std::size_t passed_runs = 0;
    for (RunRecord const& run : runs)
    {
        passed_runs += run.passed ? 1u : 0u;
        p_out << (run.passed ? "[PASS] " : "[FAIL] ") << std::left
              << std::setw(24) << run.case_id << " seed=" << std::setw(6)
              << run.seed;
        if (!run.error.empty())
        {
            p_out << " error: " << run.error << '\n';
            continue;
        }
        p_out << " " << run.outcome << " at " << run.time << " s"
              << (run.oracle ? " (oracle perception)" : "")
              << (run.physics.empty() || run.physics == "abstract"
                      ? ""
                      : " (physics " + run.physics + ")");
        for (auto const& [name, value] : run.metrics)
        {
            p_out << "  " << name << '=';
            if (value)
            {
                p_out << *value;
            }
            else
            {
                p_out << '?';
            }
        }
        p_out << '\n';
        if (run.planner.used)
        {
            p_out << "         planner " << run.planner.state << ": "
                  << run.planner.plans.size() << " plan(s), "
                  << run.planner.replans << " replan(s)";
            if (!run.planner.failure.empty())
            {
                p_out << "  (" << run.planner.failure << ')';
            }
            p_out << '\n';
        }
        for (Check const& check : run.checks)
        {
            if (check.verdict != Check::Verdict::Passed)
            {
                p_out << "         " << toString(check.verdict) << ' '
                      << check.text << "  (" << check.detail << ", expected "
                      << check.expected << ", at " << check.time.value()
                      << " s)\n";
            }
        }
    }
    p_out << experiment << ": " << passed_runs << '/' << runs.size()
          << " runs passed\n";
}

void RunReport::writeJson(std::ostream& p_out) const
{
    p_out << "{\n  \"experiment\": " << quoted(experiment)
          << ",\n  \"passed\": " << (passed() ? "true" : "false")
          << ",\n  \"runs\": [";
    for (std::size_t i = 0; i < runs.size(); ++i)
    {
        RunRecord const& run = runs[i];
        p_out << (i == 0u ? "\n" : ",\n") << "    {\n"
              << "      \"case\": " << quoted(run.case_id) << ",\n"
              << "      \"scenario\": " << quoted(run.scenario) << ",\n"
              << "      \"seed\": " << run.seed << ",\n"
              << "      \"outcome\": " << quoted(run.outcome) << ",\n"
              << "      \"time\": " << number(run.time) << ",\n"
              << "      \"passed\": " << (run.passed ? "true" : "false")
              << ",\n"
              << "      \"perception\": "
              << (run.oracle ? "\"oracle\"" : "\"sensors\"") << ",\n"
              << "      \"physics\": " << quoted(run.physics) << ",\n";
        if (!run.error.empty())
        {
            p_out << "      \"error\": " << quoted(run.error) << ",\n";
        }
        if (run.planner.used)
        {
            PlannerSummary const& planner = run.planner;
            p_out << "      \"planner\": {\n"
                  << "        \"state\": " << quoted(planner.state) << ",\n"
                  << "        \"replans\": " << planner.replans << ",\n"
                  << "        \"failure\": " << quoted(planner.failure) << ",\n"
                  << "        \"plans\": [";
            for (std::size_t p = 0; p < planner.plans.size(); ++p)
            {
                PlanSummary const& plan = planner.plans[p];
                p_out << (p == 0u ? "\n" : ",\n")
                      << "          { \"time\": " << number(plan.time)
                      << ", \"cause\": " << quoted(plan.cause)
                      << ", \"result\": " << quoted(plan.result)
                      << ", \"expanded\": " << plan.expanded
                      << ", \"milliseconds\": " << number(plan.milliseconds)
                      << ", \"steps\": [";
                for (std::size_t k = 0; k < plan.steps.size(); ++k)
                {
                    p_out << (k == 0u ? "" : ", ") << quoted(plan.steps[k]);
                }
                p_out << "] }";
            }
            p_out << (planner.plans.empty() ? "]\n" : "\n        ]\n")
                  << "      },\n";
        }
        p_out << "      \"metrics\": {";
        for (std::size_t m = 0; m < run.metrics.size(); ++m)
        {
            auto const& [name, value] = run.metrics[m];
            p_out << (m == 0u ? "" : ", ") << quoted(name) << ": "
                  << (value ? number(*value) : std::string("null"));
        }
        p_out << "},\n      \"assertions\": [";
        for (std::size_t c = 0; c < run.checks.size(); ++c)
        {
            Check const& check = run.checks[c];
            p_out << (c == 0u ? "\n" : ",\n")
                  << "        { \"name\": " << quoted(check.text)
                  << ", \"phase\": " << quoted(toString(check.phase))
                  << ", \"verdict\": " << quoted(toString(check.verdict))
                  << ", \"value\": " << number(check.value)
                  << ", \"expected\": " << quoted(check.expected)
                  << ", \"time\": " << number(check.time.value())
                  << ", \"detail\": " << quoted(check.detail) << " }";
        }
        p_out << (run.checks.empty() ? "]\n" : "\n      ]\n") << "    }";
    }
    p_out << (runs.empty() ? "]\n}\n" : "\n  ]\n}\n");
}

} // namespace robotik
