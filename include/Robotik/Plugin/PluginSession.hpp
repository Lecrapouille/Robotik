// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PluginSession.hpp
//! @brief Catalog, library and scenario instance used by Simulator and Headless.
//!
//! Tear-down order: stop the plugin, destroy the @ref Simulation (it still
//! uses the mission), then @ref release which shuts the plugin down and
//! deletes the mission. @ref unload closes the library after that.
#pragma once

#include "Robotik/Plugin/PluginCatalog.hpp"
#include "Robotik/Plugin/PluginHost.hpp"
#include "Robotik/Plugin/PluginManager.hpp"

#include "Robotik/Runtime/Scheduler.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

class Mission;
class Simulation;

enum class PluginPrepare : std::uint8_t
{
    //! @brief The path is not a scenario of a discovered package.
    NotPlugin,
    //! @brief The plugin is set up and its mission can be passed to Simulation.
    Ready,
    Failed,
};

// ****************************************************************************
//! @brief One loaded plugin and the scenario currently prepared on it.
// ****************************************************************************
class PluginSession
{
public:

    PluginSession() = default;
    PluginSession(PluginSession const&) = delete;
    PluginSession& operator=(PluginSession const&) = delete;
    ~PluginSession();

    void scan(std::filesystem::path const& p_extra = {});

    //! @brief Refuse packages that declare @c graphics: true.
    void setHeadless(bool p_headless)
    {
        m_headless = p_headless;
    }

    [[nodiscard]] PluginCatalog const& catalog() const
    {
        return m_catalog;
    }

    [[nodiscard]] PluginHost& host()
    {
        return m_host;
    }

    [[nodiscard]] PluginHost const& host() const
    {
        return m_host;
    }

    [[nodiscard]] std::string const& error() const;
    [[nodiscard]] std::string const& scenarioName() const
    {
        return m_scenario_name;
    }

    //! @brief The plugin advances the simulation inside @c update.
    [[nodiscard]] bool ownsClock() const
    {
        return m_owns_clock;
    }

    [[nodiscard]] bool active() const
    {
        return m_manager.state() == PluginState::Setup ||
               m_manager.state() == PluginState::Running ||
               m_manager.state() == PluginState::Paused ||
               m_manager.state() == PluginState::Stopped;
    }

    //! @brief Loads the owning plugin and runs @c setup. The simulation does
    //! not exist yet: the returned mission is what the host passes to it.
    PluginPrepare prepare(std::filesystem::path const& p_scenario);

    [[nodiscard]] Mission* mission() const
    {
        return m_host.mission();
    }

    void attach(Simulation* p_simulation);
    void detach();

    RobotikPluginStatus start();
    void setPaused(bool p_paused);
    //! @brief Plugin @c update, then assertion and skill events.
    void afterStep(double p_dt);
    void publishKey(std::int32_t p_code);
    void publish(RobotikEvent const& p_event);

    //! @brief Plugin @c stop. The simulation may still be alive.
    void stop();

    //! @brief Drops menus, shuts the plugin down and destroys the instance.
    //! The simulation must already be destroyed.
    void release();

    //! @brief @ref release, then closes the library.
    void unload();

private:

    void publishKind(RobotikEventKind p_kind, std::string_view p_name, std::int32_t p_flag);
    void emitRuntimeEvents();
    void fallback(std::string p_text) const;

    PluginCatalog m_catalog;
    PluginHost m_host;
    PluginManager m_manager;
    std::string m_package_id;
    std::string m_scenario_id;
    std::string m_scenario_name;
    bool m_graphics = false;
    bool m_owns_clock = false;
    bool m_headless = false;
    Simulation* m_simulation = nullptr;
    std::vector<std::uint8_t> m_checks;
    std::vector<SkillState> m_skills;
    mutable std::string m_fallback;
};

} // namespace robotik
