// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PluginHost.hpp
/// Host services behind @ref RobotikHostAPI. No user interface toolkit.
#pragma once

#include "Robotik/Package/Plugin/PluginABI.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace robotik
{

class Mission;
class Simulation;

// ****************************************************************************
/// A menu entry added by a plugin. The host draws it and calls
/// @ref callback when the user picks it.
///
/// @code
/// for (robotik::PluginMenuItem const& item : host.menus())
///     if (ImGui::MenuItem(item.label.c_str()))
///         item.callback(item.user);
/// @endcode
// ****************************************************************************
struct PluginMenuItem
{
    /// Handle given to the plugin, to remove the entry.
    std::uint32_t id = 0;
    /// Menu holding the entry (e.g. "Robot").
    std::string menu;
    /// Text of the entry.
    std::string label;
    /// Function of the plugin called on a click.
    void (*callback)(void*) = nullptr;
    /// Pointer given back to @ref callback.
    void* user = nullptr;
};

// ****************************************************************************
/// A panel added by a plugin. The host opens a window and calls @ref draw
/// every frame with a canvas to draw on.
///
/// @code
/// for (robotik::PluginPanel const& panel : host.panels())
///     panel.draw(panel.user, &canvas);
/// @endcode
// ****************************************************************************
struct PluginPanel
{
    /// Handle given to the plugin, to remove the panel.
    std::uint32_t id = 0;
    /// Stable key, to remember the layout.
    std::string key;
    /// Window title.
    std::string title;
    /// Function of the plugin drawing the content.
    void (*draw)(void*, RobotikCanvas const*) = nullptr;
    /// Pointer given back to @ref draw.
    void* user = nullptr;
};

// ****************************************************************************
/// Menus, panels, event subscriptions and the bound mission.
///
/// The object address is the @ref RobotikHost pointer. It must not move for
/// the lifetime of a plugin instance.
///
/// @code
/// robotik::PluginHost host;
/// plugin.create(host.api());
/// host.setScenarioPath(file.string());
/// host.attach(&simulation);
/// plugin.setup("wave");
/// robotik::Mission* mission = host.mission(); // bound by the plugin
/// @endcode
// ****************************************************************************
class PluginHost
{
public:

    PluginHost();
    PluginHost(PluginHost const&) = delete;
    PluginHost& operator=(PluginHost const&) = delete;
    PluginHost(PluginHost&&) = delete;
    PluginHost& operator=(PluginHost&&) = delete;

    // -------------------------------------------------------------------------
    /// @return The C functions to give to the plugin; they call this object.
    // -------------------------------------------------------------------------
    [[nodiscard]] RobotikHostAPI const* api() const
    {
        return &m_api;
    }

    // -------------------------------------------------------------------------
    /// @return The last error reported by the plugin, or empty.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& error() const
    {
        return m_error;
    }

    // -------------------------------------------------------------------------
    /// @return The mission bound by the plugin, or null.
    // -------------------------------------------------------------------------
    [[nodiscard]] Mission* mission() const
    {
        return m_mission;
    }

    // -------------------------------------------------------------------------
    /// @return The menu entries added by the plugin.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<PluginMenuItem> const& menus() const
    {
        return m_menus;
    }

    // -------------------------------------------------------------------------
    /// @return The panels added by the plugin.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<PluginPanel> const& panels() const
    {
        return m_panels;
    }

    // -------------------------------------------------------------------------
    /// Tells whether the plugin wants a kind of event.
    /// @param p_kind A @c RobotikEventKind value.
    /// @return True when the plugin subscribed to it.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool subscribed(std::uint32_t p_kind) const
    {
        return m_subscriptions.contains(p_kind);
    }

    // -------------------------------------------------------------------------
    /// Sets the scenario file the plugin may ask for.
    /// @param p_path Path of the scenario file.
    // -------------------------------------------------------------------------
    void setScenarioPath(std::string p_path);

    // -------------------------------------------------------------------------
    /// Gives the plugin access to a simulation.
    /// @param p_simulation Simulation to expose; not owned.
    // -------------------------------------------------------------------------
    void attach(Simulation* p_simulation);

    /// Forgets the simulation (before it is destroyed).
    void detach();

    /// Removes every menu entry and panel.
    void clearUi();
    /// Removes every event subscription.
    void clearSubscriptions();
    /// Forgets the bound mission.
    void clearMission();
    /// Forgets the last error.
    void clearError();

private:

    friend void
    hostLog(RobotikHost* p_host, int p_level, char const* p_message);
    friend void hostError(RobotikHost* p_host, char const* p_message);
    friend int hostSubscribe(RobotikHost* p_host, std::uint32_t p_kind);
    friend std::uint32_t hostAddMenu(RobotikHost* p_host,
                                     char const* p_menu,
                                     char const* p_item,
                                     void (*p_callback)(void*),
                                     void* p_user);
    friend std::uint32_t hostAddPanel(RobotikHost* p_host,
                                      char const* p_id,
                                      char const* p_title,
                                      void (*p_draw)(void*,
                                                     RobotikCanvas const*),
                                      void* p_user);
    friend void hostRemoveUi(RobotikHost* p_host, std::uint32_t p_handle);
    friend char const* hostScenarioPath(RobotikHost* p_host);
    friend void* hostSimulation(RobotikHost* p_host);
    friend void hostBindMission(RobotikHost* p_host, void* p_mission);

    /// C functions given to the plugin.
    RobotikHostAPI m_api{};
    /// Last error reported by the plugin.
    std::string m_error;
    /// Scenario file the plugin may ask for.
    std::string m_scenario_path;
    /// Simulation exposed to the plugin, or null.
    Simulation* m_simulation = nullptr;
    /// Mission bound by the plugin, or null.
    Mission* m_mission = nullptr;
    /// Menu entries added by the plugin.
    std::vector<PluginMenuItem> m_menus;
    /// Panels added by the plugin.
    std::vector<PluginPanel> m_panels;
    /// Event kinds the plugin subscribed to.
    std::unordered_set<std::uint32_t> m_subscriptions;
    /// Next handle of a menu entry or a panel.
    std::uint32_t m_next_id = 1;
};

} // namespace robotik
