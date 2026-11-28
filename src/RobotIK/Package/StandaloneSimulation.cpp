// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Package/StandaloneSimulation.hpp"

#include "RobotIK/Robot/Assets/AssetResolver.hpp"

#include <exception>

namespace robotik
{

// ----------------------------------------------------------------------------
StandaloneSimulation::StandaloneSimulation()
    : m_world(std::make_unique<compages::world::World>())
{
}

// ----------------------------------------------------------------------------
StandaloneSimulation::~StandaloneSimulation()
{
    // The simulation points to the scenario logic of the plugin and to the
    // world: it goes first; then the plugin frees its scenario logic and
    // closes its library; the world goes last.
    if (m_plugin)
    {
        m_plugin->stop();
        m_plugin->detach();
    }
    m_simulation.reset();
    if (m_plugin)
    {
        m_plugin->unload();
        m_plugin.reset();
    }
}

// ----------------------------------------------------------------------------
Result<StandaloneSimulation>
StandaloneSimulation::open(Scenario p_scenario, ScenarioLogic* p_logic)
{
    // The cell and the assertions report a wrong world by throwing: this
    // is the border where it becomes a message.
    try
    {
        StandaloneSimulation opened;
        opened.m_simulation = std::make_unique<Simulation>(
            *opened.m_world, std::move(p_scenario), nullptr, p_logic);
        return opened;
    }
    catch (std::exception const& error)
    {
        return failure(error.what());
    }
}

// ----------------------------------------------------------------------------
Result<StandaloneSimulation>
StandaloneSimulation::open(std::filesystem::path const& p_file,
                           ScenarioLogic* p_logic)
{
    Scenario scenario;
    ROBOTIK_TRY_ASSIGN(scenario,
                       loadScenario(p_file, AssetResolver::standard()));
    return open(std::move(scenario), p_logic);
}

// ----------------------------------------------------------------------------
Result<StandaloneSimulation>
StandaloneSimulation::openPlugin(std::filesystem::path const& p_file)
{
    return openPlugin(p_file, PluginSetup{});
}

// ----------------------------------------------------------------------------
Result<StandaloneSimulation>
StandaloneSimulation::openPlugin(std::filesystem::path const& p_file,
                                 PluginSetup p_setup)
{
    StandaloneSimulation opened;
    opened.m_plugin = std::make_unique<PluginSession>();
    PluginSession& plugin = *opened.m_plugin;
    plugin.setHeadless(true);
    plugin.scan(p_setup.plugins);
    if (plugin.prepare(p_file) != PluginPrepare::Ready)
    {
        return failure(p_file.string() + ": " +
                       (plugin.error().empty()
                            ? std::string("scenario is not in a plugin package")
                            : plugin.error()));
    }
    Scenario scenario;
    if (p_setup.scenario)
    {
        ROBOTIK_TRY_ASSIGN(scenario, p_setup.scenario(plugin.assets()));
    }
    else
    {
        ROBOTIK_TRY_ASSIGN(scenario, loadScenario(p_file, plugin.assets()));
    }
    // The cell and the assertions report a wrong world by throwing.
    try
    {
        opened.m_simulation = std::make_unique<Simulation>(
            *opened.m_world, std::move(scenario), nullptr, plugin.logic());
    }
    catch (std::exception const& error)
    {
        return failure(error.what());
    }
    plugin.attach(opened.m_simulation.get());
    if (plugin.start() != ROBOTIK_PLUGIN_OK)
    {
        return failure(p_file.string() + ": " + plugin.error());
    }
    return opened;
}

// ----------------------------------------------------------------------------
void StandaloneSimulation::step(Seconds p_dt)
{
    if (!m_plugin || !m_plugin->ownsClock())
    {
        m_simulation->step(p_dt);
    }
    if (m_plugin)
    {
        m_plugin->afterStep(p_dt.value());
    }
}

} // namespace robotik
