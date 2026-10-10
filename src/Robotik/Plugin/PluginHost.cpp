// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Plugin/PluginHost.hpp"

#include "Robotik/Runtime/Mission.hpp"

#include <algorithm>
#include <iostream>

namespace robotik
{

namespace
{

PluginHost* self(RobotikHost* p_host)
{
    return reinterpret_cast<PluginHost*>(p_host);
}

} // namespace

void hostLog(RobotikHost* p_host, int p_level, char const* p_message)
{
    char const* label = "info";
    if (p_level == ROBOTIK_LOG_WARNING)
    {
        label = "warning";
    }
    else if (p_level >= ROBOTIK_LOG_ERROR)
    {
        label = "error";
    }
    std::clog << "plugin " << label << ": " << (p_message != nullptr ? p_message : "")
              << '\n';
    (void)self(p_host);
}

void hostError(RobotikHost* p_host, char const* p_message)
{
    if (PluginHost* host = self(p_host))
    {
        host->m_error = p_message != nullptr ? p_message : "";
    }
}

int hostSubscribe(RobotikHost* p_host, std::uint32_t p_kind)
{
    PluginHost* host = self(p_host);
    if (host == nullptr)
    {
        return 0;
    }
    host->m_subscriptions.insert(p_kind);
    return 1;
}

std::uint32_t hostAddMenu(RobotikHost* p_host,
                          char const* p_menu,
                          char const* p_item,
                          void (*p_callback)(void*),
                          void* p_user)
{
    PluginHost* host = self(p_host);
    if (host == nullptr || p_menu == nullptr || p_item == nullptr || p_callback == nullptr)
    {
        return 0;
    }
    PluginMenuItem item;
    item.id = host->m_next_id++;
    item.menu = p_menu;
    item.label = p_item;
    item.callback = p_callback;
    item.user = p_user;
    host->m_menus.push_back(std::move(item));
    return host->m_menus.back().id;
}

std::uint32_t hostAddPanel(RobotikHost* p_host,
                           char const* p_id,
                           char const* p_title,
                           void (*p_draw)(void*, RobotikCanvas const*),
                           void* p_user)
{
    PluginHost* host = self(p_host);
    if (host == nullptr || p_id == nullptr || p_title == nullptr || p_draw == nullptr)
    {
        return 0;
    }
    PluginPanel panel;
    panel.id = host->m_next_id++;
    panel.key = p_id;
    panel.title = p_title;
    panel.draw = p_draw;
    panel.user = p_user;
    host->m_panels.push_back(std::move(panel));
    return host->m_panels.back().id;
}

void hostRemoveUi(RobotikHost* p_host, std::uint32_t p_handle)
{
    PluginHost* host = self(p_host);
    if (host == nullptr || p_handle == 0)
    {
        return;
    }
    auto const menu = std::remove_if(host->m_menus.begin(),
                                     host->m_menus.end(),
                                     [p_handle](PluginMenuItem const& p_item) {
                                         return p_item.id == p_handle;
                                     });
    host->m_menus.erase(menu, host->m_menus.end());
    auto const panel = std::remove_if(host->m_panels.begin(),
                                      host->m_panels.end(),
                                      [p_handle](PluginPanel const& p_item) {
                                          return p_item.id == p_handle;
                                      });
    host->m_panels.erase(panel, host->m_panels.end());
}

char const* hostScenarioPath(RobotikHost* p_host)
{
    PluginHost* host = self(p_host);
    return host != nullptr ? host->m_scenario_path.c_str() : "";
}

void* hostSimulation(RobotikHost* p_host)
{
    PluginHost* host = self(p_host);
    return host != nullptr ? host->m_simulation : nullptr;
}

void hostBindMission(RobotikHost* p_host, void* p_mission)
{
    if (PluginHost* host = self(p_host))
    {
        host->m_mission = static_cast<Mission*>(p_mission);
    }
}

PluginHost::PluginHost()
{
    m_api.size = static_cast<std::uint32_t>(sizeof(m_api));
    m_api.abi_version = ROBOTIK_PLUGIN_ABI_VERSION;
    m_api.host = reinterpret_cast<RobotikHost*>(this);
    m_api.log = &hostLog;
    m_api.report_error = &hostError;
    m_api.subscribe = &hostSubscribe;
    m_api.add_menu_item = &hostAddMenu;
    m_api.add_panel = &hostAddPanel;
    m_api.remove_ui = &hostRemoveUi;
    m_api.scenario_path = &hostScenarioPath;
    m_api.simulation = &hostSimulation;
    m_api.bind_mission = &hostBindMission;
}

void PluginHost::setScenarioPath(std::string p_path)
{
    m_scenario_path = std::move(p_path);
}

void PluginHost::attach(Simulation* p_simulation)
{
    m_simulation = p_simulation;
}

void PluginHost::detach()
{
    m_simulation = nullptr;
}

void PluginHost::clearUi()
{
    m_menus.clear();
    m_panels.clear();
}

void PluginHost::clearSubscriptions()
{
    m_subscriptions.clear();
}

void PluginHost::clearMission()
{
    m_mission = nullptr;
}

void PluginHost::clearError()
{
    m_error.clear();
}

} // namespace robotik
