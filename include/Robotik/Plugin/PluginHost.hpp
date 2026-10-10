// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PluginHost.hpp
//! @brief Host services behind @ref RobotikHostAPI. No user interface toolkit.
#pragma once

#include "Robotik/Plugin/PluginABI.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace robotik
{

class Mission;
class Simulation;

struct PluginMenuItem
{
    std::uint32_t id = 0;
    std::string menu;
    std::string label;
    void (*callback)(void*) = nullptr;
    void* user = nullptr;
};

struct PluginPanel
{
    std::uint32_t id = 0;
    std::string key;
    std::string title;
    void (*draw)(void*, RobotikCanvas const*) = nullptr;
    void* user = nullptr;
};

// ****************************************************************************
//! @brief Menus, panels, event subscriptions and the bound mission.
//!
//! The object address is the @ref RobotikHost pointer. It must not move for
//! the lifetime of a plugin instance.
// ****************************************************************************
class PluginHost
{
public:

    PluginHost();
    PluginHost(PluginHost const&) = delete;
    PluginHost& operator=(PluginHost const&) = delete;
    PluginHost(PluginHost&&) = delete;
    PluginHost& operator=(PluginHost&&) = delete;

    [[nodiscard]] RobotikHostAPI const* api() const
    {
        return &m_api;
    }

    [[nodiscard]] std::string const& error() const
    {
        return m_error;
    }

    [[nodiscard]] Mission* mission() const
    {
        return m_mission;
    }

    [[nodiscard]] std::vector<PluginMenuItem> const& menus() const
    {
        return m_menus;
    }

    [[nodiscard]] std::vector<PluginPanel> const& panels() const
    {
        return m_panels;
    }

    [[nodiscard]] bool subscribed(std::uint32_t p_kind) const
    {
        return m_subscriptions.find(p_kind) != m_subscriptions.end();
    }

    void setScenarioPath(std::string p_path);
    void attach(Simulation* p_simulation);
    void detach();

    void clearUi();
    void clearSubscriptions();
    void clearMission();
    void clearError();

private:

    friend void hostLog(RobotikHost* p_host, int p_level, char const* p_message);
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
                                      void (*p_draw)(void*, RobotikCanvas const*),
                                      void* p_user);
    friend void hostRemoveUi(RobotikHost* p_host, std::uint32_t p_handle);
    friend char const* hostScenarioPath(RobotikHost* p_host);
    friend void* hostSimulation(RobotikHost* p_host);
    friend void hostBindMission(RobotikHost* p_host, void* p_mission);

    RobotikHostAPI m_api{};
    std::string m_error;
    std::string m_scenario_path;
    Simulation* m_simulation = nullptr;
    Mission* m_mission = nullptr;
    std::vector<PluginMenuItem> m_menus;
    std::vector<PluginPanel> m_panels;
    std::unordered_set<std::uint32_t> m_subscriptions;
    std::uint32_t m_next_id = 1;
};

} // namespace robotik
