// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The only file that includes Kairos: no Kairos type leaves it.

#include "Robotik/Task/Planning/PlanningEngine.hpp"

#include <kairos/kairos.hpp>

#include <optional>
#include <sstream>

namespace robotik
{

/// One line per diagnostic, with the source line under it.
static std::string
describe(std::vector<kairos::Diagnostic> const& p_diagnostics,
         std::string_view p_source)
{
    std::string text;
    for (kairos::Diagnostic const& diagnostic : p_diagnostics)
    {
        text += kairos::to_string(diagnostic, p_source);
        text += '\n';
    }
    return text;
}

// ****************************************************************************
/// @ref PlanningEngine on Kairos.
// ****************************************************************************
class KairosPlanner final: public PlanningEngine
{
public:

    std::string load(std::string const& p_domain,
                     std::string const& p_file) override
    {
        m_domain.reset();
        auto parsed = kairos::parse_domain(p_domain, p_file);
        if (!parsed)
        {
            std::string const text = describe(parsed.diagnostics(), p_domain);
            return text.empty() ? std::string("invalid domain") : text;
        }
        m_domain.emplace(std::move(parsed.value()));
        return {};
    }

    PlanningResult solve(std::string const& p_problem,
                         PlanningOptions const& p_options) override
    {
        PlanningResult result;
        if (!m_domain)
        {
            result.diagnostics = "no domain loaded";
            return result;
        }
        auto problem =
            kairos::parse_problem(p_problem, *m_domain, "problem.pddl");
        if (!problem)
        {
            result.diagnostics = describe(problem.diagnostics(), p_problem);
            return result;
        }

        kairos::SolveOptions options;
        options.timeout = p_options.timeout;
        options.anytime = false;
        options.threads = 1U;
        options.seed = p_options.seed;
        options.search = p_options.search;
        // Silent: what matters comes back as diagnostics and in the report.
        options.log.level = kairos::LogLevel::Error;
        options.log.callback = [](kairos::LogLevel,
                                  kairos::LogChannel,
                                  std::string_view) noexcept {};
        kairos::SolveResult const solved =
            kairos::Planner().solve(*problem, options);

        result.diagnostics = describe(solved.diagnostics, p_problem);
        result.expanded = solved.stats.expanded;
        result.milliseconds = solved.stats.total_ms;
        switch (solved.status)
        {
            case kairos::Status::Solved:
                result.status = PlanningResult::Status::Solved;
                break;
            case kairos::Status::Unsolvable:
                result.status = PlanningResult::Status::Unsolvable;
                break;
            case kairos::Status::Timeout:
            case kairos::Status::Cancelled:
            case kairos::Status::OutOfMemory:
            case kairos::Status::GroundingTooLarge:
            case kairos::Status::NoPlanFound:
                result.status = PlanningResult::Status::NoPlanFound;
                break;
            case kairos::Status::Unsupported:
            case kairos::Status::InvalidInput:
            case kairos::Status::InternalError:
                result.status = PlanningResult::Status::Invalid;
                break;
        }
        std::ostringstream report;
        report << "; status " << kairos::to_string(solved.status) << ", "
               << kairos::to_string(solved.stop_reason) << "\n; "
               << solved.stats.expanded << " expanded, " << solved.stats.atoms
               << " atoms, " << solved.stats.actions << " ground actions, "
               << solved.stats.total_ms << " ms\n";
        if (solved.plan)
        {
            report << kairos::to_string(*solved.plan, kairos::PlanFormat::Ipc);
            for (kairos::PlanStep const& step : solved.plan->steps)
            {
                result.steps.push_back(
                    { step.name, step.args, step.time, step.duration });
            }
        }
        result.report = report.str();
        return result;
    }

private:

    std::optional<kairos::Domain> m_domain;
};

char const* toString(PlanningResult::Status p_status)
{
    switch (p_status)
    {
        case PlanningResult::Status::Solved:
            return "solved";
        case PlanningResult::Status::Unsolvable:
            return "unsolvable";
        case PlanningResult::Status::NoPlanFound:
            return "no plan found";
        case PlanningResult::Status::Invalid:
            return "invalid";
    }
    return "?";
}

std::unique_ptr<PlanningEngine> makeKairosPlanner()
{
    return std::make_unique<KairosPlanner>();
}

} // namespace robotik
