// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.


#include "Robotik/Package/Simulation.hpp"

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
                       Mission* p_mission)
    : m_scenario(std::move(p_scenario)),
      m_cell(p_world, m_scenario, p_view, p_mission),
      m_assertions(m_scenario.assertions)
{
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
    m_assertions.update(time(),
                        [this](std::string_view p_condition)
                        { return evaluate(p_condition); });
    announceChecks();

    bt::Status const status = m_cell.status();
    if (status == bt::Status::SUCCESS)
    {
        finish(Outcome::ControllerSucceeded);
    }
    else if (status == bt::Status::FAILURE)
    {
        finish(Outcome::ControllerFailed);
    }
    else if (m_assertions.stopRequested())
    {
        finish(Outcome::AssertionFailed);
    }
    else if (!m_scenario.stop_when.empty() &&
             evaluate(m_scenario.stop_when).holds)
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
    m_assertions.conclude(time(),
                          [this](std::string_view p_condition)
                          { return evaluate(p_condition); });
    announceChecks();
    ended(p_outcome);
}

// ----------------------------------------------------------------------------
void Simulation::announceChecks()
{
    std::vector<Check> const& checks = m_assertions.checks();
    if (m_colors.size() != checks.size())
    {
        m_colors.assign(checks.size(), 2u);
    }
    for (std::size_t i = 0; i < checks.size(); ++i)
    {
        std::uint8_t const green = ok(checks[i]) ? 1u : 0u;
        if (m_colors[i] != green)
        {
            m_colors[i] = green;
            assertionChanged(i, green == 1u);
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
