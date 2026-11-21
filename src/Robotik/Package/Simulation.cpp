// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.


#include "Robotik/Package/Simulation.hpp"

#include <stdexcept>

namespace robotik
{

// ----------------------------------------------------------------------------
char const* toString(Simulation::Outcome p_outcome)
{
    switch (p_outcome)
    {
        case Simulation::Outcome::Running:
            return "running";
        case Simulation::Outcome::ControllerSucceeded:
            return "controller succeeded";
        case Simulation::Outcome::ControllerFailed:
            return "controller failed";
        case Simulation::Outcome::StopCondition:
            return "stop condition";
        case Simulation::Outcome::Timeout:
            return "timeout";
        case Simulation::Outcome::AssertionFailed:
            return "assertion failed";
        case Simulation::Outcome::Interrupted:
            return "interrupted";
    }
    return "?";
}

// ----------------------------------------------------------------------------
Simulation::Simulation(compages::world::World& p_world,
                       Scenario p_scenario,
                       SceneView* p_view,
                       ScenarioLogic* p_logic)
    : m_scenario(std::move(p_scenario)),
      m_cell(p_world, m_scenario, p_view, p_logic),
      m_assertions(m_scenario.assertions)
{
    // Read every condition now: a typo stops the load, not the run.
    std::string const where = m_scenario.file.string() + ": ";
    for (std::string const& text : m_assertions.conditions())
    {
        Result<Condition> condition = m_cell.compile(text);
        if (!condition)
        {
            throw std::runtime_error(where + "assertion " + condition.error());
        }
        m_conditions.push_back(condition.take());
    }
    if (!m_scenario.stop_when.empty())
    {
        Result<Condition> condition = m_cell.compile(m_scenario.stop_when);
        if (!condition)
        {
            throw std::runtime_error(where + "stop_when " + condition.error());
        }
        m_stop_when = condition.take();
    }
}

// ----------------------------------------------------------------------------
void Simulation::reset(Seed p_seed)
{
    m_cell.reset(p_seed);
    m_outcome = Outcome::Running;
    m_assertions = AssertionEngine(m_scenario.assertions);
    m_colors.clear();
}

// ----------------------------------------------------------------------------
void Simulation::step(Seconds p_dt)
{
    m_cell.step(p_dt);
    if (!m_cell.suspended())
    {
        settle();
    }
}

// ----------------------------------------------------------------------------
void Simulation::settle()
{
    if (finished())
    {
        return;
    }
    // One measure per step, shared by every condition.
    Metrics const metrics = m_cell.metrics();
    m_assertions.update(time(),
                        [this, &metrics](std::size_t p_condition)
                        { return robotik::evaluate(m_conditions[p_condition], metrics); });
    announceChecks();

    Status const status = m_cell.status();
    if (status == Status::SUCCESS)
    {
        finish(Outcome::ControllerSucceeded);
    }
    else if (status == Status::FAILURE)
    {
        finish(Outcome::ControllerFailed);
    }
    else if (m_assertions.stopRequested())
    {
        finish(Outcome::AssertionFailed);
    }
    else if (!m_stop_when.nodes.empty() &&
             robotik::evaluate(m_stop_when, metrics).holds)
    {
        finish(Outcome::StopCondition);
    }
    else if (m_scenario.timeout.value() > 0.0 && time() >= m_scenario.timeout)
    {
        finish(Outcome::Timeout);
    }
}

// ----------------------------------------------------------------------------
void Simulation::finish(Outcome p_outcome)
{
    m_outcome = p_outcome;
    m_cell.stopController();
    Metrics const metrics = m_cell.metrics();
    m_assertions.conclude(time(),
                          [this, &metrics](std::size_t p_condition)
                          { return robotik::evaluate(m_conditions[p_condition], metrics); });
    announceChecks();
    ended(p_outcome);
}

// ----------------------------------------------------------------------------
void Simulation::announceChecks()
{
    std::vector<Check> const& checks = m_assertions.checks();
    if (m_colors.size() != checks.size())
    {
        m_colors.assign(checks.size(), 2U);
    }
    for (std::size_t i = 0; i < checks.size(); ++i)
    {
        std::uint8_t const green = ok(checks[i]) ? 1U : 0U;
        if (m_colors[i] != green)
        {
            m_colors[i] = green;
            assertionChanged(i, green == 1U);
        }
    }
}

// ----------------------------------------------------------------------------
void Simulation::conclude()
{
    if (!finished())
    {
        finish(Outcome::Interrupted);
    }
}

} // namespace robotik
