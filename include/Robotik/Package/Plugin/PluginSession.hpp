// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PluginSession.hpp
/// Catalog, library and scenario instance used by Simulator and Headless.
///
/// Tear-down order: stop the plugin, destroy the @ref Simulation (it still
/// uses the mission), then @ref release which shuts the plugin down and
/// deletes the mission. @ref unload closes the library after that.
#pragma once

#include "Robotik/Robot/Assets/AssetResolver.hpp"
#include "Robotik/Package/Plugin/PluginCatalog.hpp"
#include "Robotik/Package/Plugin/PluginHost.hpp"
#include "Robotik/Package/Plugin/PluginManager.hpp"

#include "Robotik/Common/Signal.hpp"
#include "Robotik/Task/Scheduler.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

class Mission;
class Simulation;

// ****************************************************************************
/// Result of @ref PluginSession::prepare.
// ****************************************************************************
enum class PluginPrepare : std::uint8_t
{
    /// The path is not a scenario of a discovered package.
    NotPlugin,
    /// The plugin is set up and its mission can be passed to Simulation.
    Ready,
    /// A dependency is missing or the plugin refused: see
    /// @ref PluginSession::error.
    Failed,
};

// ****************************************************************************
/// One loaded plugin and the scenario currently prepared on it.
///
/// @code
/// robotik::PluginSession session;
/// session.scan();
/// if (session.prepare(file) != robotik::PluginPrepare::Ready)
///     return fail(session.error());
/// {
///     robotik::Simulation simulation(world, scenario, nullptr,
///     session.mission()); session.attach(&simulation); session.start(); while
///     (running) {
///         simulation.step(Seconds(dt));
///         session.afterStep(dt);
///     }
///     session.stop();
///     session.detach();
/// }                      // the simulation is destroyed here
/// session.release();
/// session.unload();
/// @endcode
// ****************************************************************************
class PluginSession
{
public:

    PluginSession() = default;
    PluginSession(PluginSession const&) = delete;
    PluginSession& operator=(PluginSession const&) = delete;

    /// Releases the plugin and closes the library.
    ~PluginSession();

    // -------------------------------------------------------------------------
    /// Lists the packages, then the asset packs: the standard roots and the
    /// own pack of every package.
    /// @param p_extra Extra package or folder of packages; empty for none.
    // -------------------------------------------------------------------------
    void scan(std::filesystem::path const& p_extra = {});

    // -------------------------------------------------------------------------
    /// Refuses packages that declare @c graphics: true.
    /// @param p_headless True for a host without rendering.
    // -------------------------------------------------------------------------
    void setHeadless(bool p_headless)
    {
        m_headless = p_headless;
    }

    /// @return The packages found by @ref scan.
    [[nodiscard]] PluginCatalog const& catalog() const
    {
        return m_catalog;
    }

    /// @return Asset packs visible to the scenarios of the catalog.
    [[nodiscard]] AssetResolver const& assets() const
    {
        return m_assets;
    }

    /// @return Menus, panels and the mission of the plugin.
    [[nodiscard]] PluginHost& host()
    {
        return m_host;
    }

    /// @copydoc host()
    [[nodiscard]] PluginHost const& host() const
    {
        return m_host;
    }

    // -------------------------------------------------------------------------
    /// @return The error of the library, else of the plugin, else of the
    ///   session (missing dependency...); empty when all is fine.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& error() const;

    /// @return Name of the prepared scenario, as in @c plugin.yaml.
    [[nodiscard]] std::string const& scenarioName() const
    {
        return m_scenario_name;
    }

    /// @return True when the plugin advances the simulation inside @c update.
    [[nodiscard]] bool ownsClock() const
    {
        return m_owns_clock;
    }

    /// @return True from @ref prepare until @ref release.
    [[nodiscard]] bool active() const
    {
        return m_manager.state() == PluginState::Setup ||
               m_manager.state() == PluginState::Running ||
               m_manager.state() == PluginState::Paused ||
               m_manager.state() == PluginState::Stopped;
    }

    // -------------------------------------------------------------------------
    /// Checks the asset packs and plugins the package depends on, loads the
    /// owning plugin and runs @c setup. The simulation does not exist yet:
    /// the returned mission is what the host passes to it.
    /// @param p_scenario Scenario file.
    /// @return Whether the scenario is a plugin one, and ready.
    // -------------------------------------------------------------------------
    PluginPrepare prepare(std::filesystem::path const& p_scenario);

    /// @return The mission bound by the plugin in @c setup, or null.
    [[nodiscard]] Mission* mission() const
    {
        return m_host.mission();
    }

    // -------------------------------------------------------------------------
    /// Gives the plugin the simulation built with its mission.
    /// @param p_simulation Simulation; not owned.
    // -------------------------------------------------------------------------
    void attach(Simulation* p_simulation);

    /// Forgets the simulation, before it is destroyed.
    void detach();

    /// Starts the plugin. @return The status of the plugin.
    RobotikPluginStatus start();

    /// Pauses (true) or resumes (false) the plugin.
    void setPaused(bool p_paused);

    // -------------------------------------------------------------------------
    /// Calls the plugin @c update, then sends the assertion and skill events
    /// that changed. Call it after each simulation step.
    /// @param p_dt Duration of the step (s).
    // -------------------------------------------------------------------------
    void afterStep(double p_dt);

    /// Sends a key event (if the plugin subscribed to it).
    void publishKey(std::int32_t p_code);

    /// Sends an event, if the plugin subscribed to its kind and is set up.
    void publish(RobotikEvent const& p_event);

    // -------------------------------------------------------------------------
    /// Calls the plugin @c stop. The simulation may still be alive.
    // -------------------------------------------------------------------------
    void stop();

    // -------------------------------------------------------------------------
    /// Drops menus, shuts the plugin down and destroys the instance. The
    /// simulation must already be destroyed.
    // -------------------------------------------------------------------------
    void release();

    // -------------------------------------------------------------------------
    /// Calls @ref release, then closes the library.
    // -------------------------------------------------------------------------
    void unload();

private:

    /// Checks the asset packs and the plugins @p_package needs.
    [[nodiscard]] bool dependenciesMet(PluginPackage const& p_package);
    /// Sends an event of @p_kind with a name and a flag.
    void publishKind(RobotikEventKind p_kind,
                     std::string_view p_name,
                     std::int32_t p_flag);
    /// Sends the assertion and skill events announced since the last call.
    void emitRuntimeEvents();
    /// Sets the error of the session itself (see @ref error).
    void fallback(std::string p_text) const;

    /// Packages found.
    PluginCatalog m_catalog;
    /// Asset packs of the standard roots and the packages.
    AssetResolver m_assets;
    /// Services given to the plugin.
    PluginHost m_host;
    /// Library and instance of the plugin.
    PluginManager m_manager;
    /// Id of the loaded package.
    std::string m_package_id;
    /// Id of the prepared scenario.
    std::string m_scenario_id;
    /// Name of the prepared scenario.
    std::string m_scenario_name;
    /// The package needs a graphical host.
    bool m_graphics = false;
    /// The plugin advances the simulation.
    bool m_owns_clock = false;
    /// The host has no rendering.
    bool m_headless = false;
    /// Simulation attached, or null.
    Simulation* m_simulation = nullptr;
    /// Last state sent of each assertion: 0 failed, 1 passed, 2 unknown.
    std::vector<std::uint8_t> m_checks;
    /// Assertion colors announced by the simulation since the last step.
    std::vector<std::pair<std::size_t, bool>> m_pending_checks;
    /// Skills ended since the last step: name and final state.
    std::vector<std::pair<std::string, SkillState>> m_pending_skills;
    /// Listens to the assertions of the attached simulation.
    ScopedConnection<std::size_t, bool> m_check_link;
    /// Listens to the skills of the attached simulation.
    ScopedConnection<SkillId, SkillState> m_skill_link;
    /// Error of the session itself.
    mutable std::string m_fallback;
};

} // namespace robotik
