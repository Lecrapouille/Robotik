// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Plugin/PluginSession.hpp"

#include "Robotik/Runtime/Simulation.hpp"

#include <algorithm>
#include <cstring>
#include <span>
#include <utility>

namespace robotik
{

namespace
{

void copyName(char* p_dest, std::size_t p_capacity, std::string_view p_value)
{
    if (p_dest == nullptr || p_capacity == 0u)
    {
        return;
    }
    std::size_t const count = std::min(p_value.size(), p_capacity - 1u);
    if (count > 0u)
    {
        std::memcpy(p_dest, p_value.data(), count);
    }
    p_dest[count] = '\0';
}

} // namespace

PluginSession::~PluginSession()
{
    detach();
    unload();
}

void PluginSession::scan(std::filesystem::path const& p_extra)
{
    m_catalog.scan(p_extra);
}

std::string const& PluginSession::error() const
{
    if (!m_manager.error().empty())
    {
        return m_manager.error();
    }
    if (!m_host.error().empty())
    {
        return m_host.error();
    }
    return m_fallback;
}

void PluginSession::fallback(std::string p_text) const
{
    m_fallback = std::move(p_text);
}

PluginPrepare PluginSession::prepare(std::filesystem::path const& p_scenario)
{
    m_fallback.clear();
    m_host.clearError();
    m_owns_clock = false;
    if (m_catalog.packages().empty())
    {
        scan();
    }
    PluginMatch const matched = m_catalog.matchScenario(p_scenario);
    PluginPackage const* package = matched.package;
    if (package == nullptr)
    {
        unload();
        m_package_id.clear();
        return PluginPrepare::NotPlugin;
    }
    PluginScenario const* scenario = matched.scenario;
    if (scenario == nullptr)
    {
        unload();
        return PluginPrepare::Failed;
    }
    PluginPackage const copy = *package;
    PluginScenario const chosen = *scenario;
    if (m_headless && copy.graphics)
    {
        unload();
        fallback("this plugin requires the simulator");
        return PluginPrepare::Failed;
    }
    if (m_manager.loaded() && m_manager.id() != copy.id)
    {
        unload();
    }
    if (!m_manager.loaded())
    {
        std::error_code error;
        if (!std::filesystem::is_regular_file(copy.library, error))
        {
            fallback("plugin library not found: " + copy.library.string());
            return PluginPrepare::Failed;
        }
        if (m_manager.load(copy.library.string()) != ROBOTIK_PLUGIN_OK)
        {
            return PluginPrepare::Failed;
        }
        if (m_manager.id() != copy.id)
        {
            unload();
            fallback("plugin id does not match plugin.yaml");
            return PluginPrepare::Failed;
        }
    }
    else
    {
        release();
    }
    m_host.setScenarioPath(chosen.file.string());
    m_package_id = copy.id;
    m_scenario_id = chosen.id;
    m_scenario_name = chosen.name;
    m_graphics = copy.graphics;
    m_owns_clock = copy.owns_clock;
    if (m_manager.create(m_host.api()) != ROBOTIK_PLUGIN_OK)
    {
        return PluginPrepare::Failed;
    }
    if (m_manager.setup(chosen.id) != ROBOTIK_PLUGIN_OK)
    {
        if (m_host.error().empty() && m_manager.error().empty())
        {
            fallback("scenario setup failed");
        }
        release();
        return PluginPrepare::Failed;
    }
    return PluginPrepare::Ready;
}

void PluginSession::attach(Simulation* p_simulation)
{
    m_simulation = p_simulation;
    m_host.attach(p_simulation);
}

void PluginSession::detach()
{
    m_simulation = nullptr;
    m_host.detach();
}

RobotikPluginStatus PluginSession::start()
{
    m_checks.clear();
    m_skills.clear();
    RobotikPluginStatus const status = m_manager.start();
    if (status == ROBOTIK_PLUGIN_OK)
    {
        publishKind(ROBOTIK_EVENT_SCENARIO_STARTED, m_scenario_id, 0);
    }
    return status;
}

void PluginSession::setPaused(bool p_paused)
{
    if (p_paused && m_manager.state() == PluginState::Running)
    {
        m_manager.pause(true);
    }
    else if (!p_paused && m_manager.state() == PluginState::Paused)
    {
        m_manager.pause(false);
    }
}

void PluginSession::afterStep(double p_dt)
{
    if (m_manager.state() == PluginState::Running)
    {
        m_manager.update(p_dt);
    }
    emitRuntimeEvents();
}

void PluginSession::publishKey(std::int32_t p_code)
{
    RobotikEvent event{};
    event.size = static_cast<std::uint32_t>(sizeof(event));
    event.kind = static_cast<std::uint32_t>(ROBOTIK_EVENT_KEY_PRESSED);
    event.code = p_code;
    publish(event);
}

void PluginSession::publish(RobotikEvent const& p_event)
{
    if (!m_host.subscribed(p_event.kind))
    {
        return;
    }
    if (m_manager.state() != PluginState::Setup &&
        m_manager.state() != PluginState::Running &&
        m_manager.state() != PluginState::Paused)
    {
        return;
    }
    m_manager.onEvent(p_event);
}

void PluginSession::publishKind(RobotikEventKind p_kind,
                                std::string_view p_name,
                                std::int32_t p_flag)
{
    RobotikEvent event{};
    event.size = static_cast<std::uint32_t>(sizeof(event));
    event.kind = static_cast<std::uint32_t>(p_kind);
    event.flag = p_flag;
    copyName(event.name, sizeof(event.name), p_name);
    publish(event);
}

void PluginSession::emitRuntimeEvents()
{
    if (m_simulation == nullptr)
    {
        return;
    }
    std::vector<Check> const checks = m_simulation->checks();
    if (m_checks.size() != checks.size())
    {
        m_checks.assign(checks.size(), 2u);
    }
    for (std::size_t i = 0; i < checks.size(); ++i)
    {
        std::uint8_t const passed = checks[i].passed ? 1u : 0u;
        if (m_checks[i] != passed)
        {
            m_checks[i] = passed;
            publishKind(ROBOTIK_EVENT_ASSERTION_CHANGED,
                        checks[i].text,
                        static_cast<std::int32_t>(passed));
        }
    }

    std::span<SkillRun const> const trace = m_simulation->skills().trace();
    if (m_skills.size() > trace.size())
    {
        m_skills.clear();
    }
    m_skills.resize(trace.size(), SkillState::Idle);
    for (std::size_t i = 0; i < trace.size(); ++i)
    {
        if (m_skills[i] == trace[i].state)
        {
            continue;
        }
        m_skills[i] = trace[i].state;
        if (trace[i].state == SkillState::Succeeded)
        {
            publishKind(ROBOTIK_EVENT_SKILL_COMPLETED,
                        m_simulation->skills().name(trace[i].skill),
                        0);
        }
        else if (trace[i].state == SkillState::Failed)
        {
            publishKind(ROBOTIK_EVENT_SKILL_FAILED,
                        m_simulation->skills().name(trace[i].skill),
                        0);
        }
    }
}

void PluginSession::stop()
{
    if (m_manager.state() == PluginState::Running ||
        m_manager.state() == PluginState::Paused)
    {
        publishKind(ROBOTIK_EVENT_SCENARIO_STOPPED, m_scenario_id, 0);
        m_manager.stop();
    }
}

void PluginSession::release()
{
    detach();
    m_host.clearUi();
    m_host.clearSubscriptions();
    if (m_manager.state() == PluginState::Running ||
        m_manager.state() == PluginState::Paused)
    {
        m_manager.stop();
    }
    m_manager.shutdown();
    m_host.clearMission();
    m_manager.destroy();
    m_checks.clear();
    m_skills.clear();
}

void PluginSession::unload()
{
    release();
    m_manager.unload();
    m_package_id.clear();
    m_scenario_id.clear();
    m_scenario_name.clear();
}

} // namespace robotik
