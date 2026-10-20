// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/Plugin/PluginSession.hpp"

#include "Robotik/Package/Simulation.hpp"

#include <algorithm>
#include <cstring>
#include <span>
#include <utility>

namespace robotik
{

/// Copies a name into a fixed C buffer and terminates it.
static void
copyName(char* p_dest, std::size_t p_capacity, std::string_view p_value)
{
    if (p_dest == nullptr || p_capacity == 0U)
    {
        return;
    }
    std::size_t const count = std::min(p_value.size(), p_capacity - 1U);
    if (count > 0U)
    {
        std::memcpy(p_dest, p_value.data(), count);
    }
    p_dest[count] = '\0';
}

PluginSession::~PluginSession()
{
    detach();
    unload();
}

void PluginSession::scan(std::filesystem::path const& p_extra)
{
    m_catalog.scan(p_extra);
    m_assets = AssetResolver::standard();
    for (PluginPackage const& package : m_catalog.packages())
    {
        if (!package.resources.empty())
        {
            m_assets.addRoot(package.resources);
        }
    }
}

bool PluginSession::dependenciesMet(PluginPackage const& p_package)
{
    for (PackRequirement const& pack : p_package.packs)
    {
        std::string why;
        if (m_assets.require(pack.id, pack.version, why) == nullptr)
        {
            fallback(p_package.id + ": " + why);
            return false;
        }
    }
    for (std::string const& plugin : p_package.plugins)
    {
        if (m_catalog.findPackage(plugin) == nullptr)
        {
            fallback(p_package.id + ": plugin '" + plugin +
                     "' is not installed");
            return false;
        }
    }
    return true;
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
    PluginMatch matched = m_catalog.matchScenario(p_scenario);
    if (matched.package == nullptr)
    {
        // A scenario given by path: its own package, wherever it is.
        if (PluginPackage const* found = m_catalog.add(p_scenario))
        {
            if (!found->resources.empty())
            {
                m_assets.addRoot(found->resources);
            }
            matched = m_catalog.matchScenario(p_scenario);
        }
    }
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
    if (!dependenciesMet(copy))
    {
        unload();
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
    m_owns_clock = chosen.owns_clock;
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
    m_pending_checks.clear();
    m_pending_skills.clear();
    if (p_simulation == nullptr)
    {
        m_check_link = {};
        m_skill_link = {};
        return;
    }
    // The events wait for the next afterStep: the plugin sees them after its
    // own update, as before.
    m_check_link = ScopedConnection<std::size_t, bool>(
        p_simulation->assertionChanged,
        [this](std::size_t p_index, bool p_green) { m_pending_checks.emplace_back(p_index, p_green); });
    SkillScheduler& skills = p_simulation->cell().skills();
    m_skill_link = ScopedConnection<SkillId, SkillState>(
        skills.changed,
        [this, &skills](SkillId p_id, SkillState p_state)
        {
            if (p_state == SkillState::Succeeded || p_state == SkillState::Failed)
            {
                m_pending_skills.emplace_back(skills.name(p_id), p_state);
            }
        });
}

void PluginSession::detach()
{
    m_check_link = {};
    m_skill_link = {};
    m_pending_checks.clear();
    m_pending_skills.clear();
    m_simulation = nullptr;
    m_host.detach();
}

RobotikPluginStatus PluginSession::start()
{
    m_checks.clear();
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
    // Assertions first, then skills; one event per assertion and step, only
    // when its color differs from the last one sent.
    std::vector<Check> const& checks = m_simulation->checks();
    if (m_checks.size() != checks.size())
    {
        m_checks.assign(checks.size(), 2U);
        for (std::size_t i = 0; i < checks.size(); ++i)
        {
            m_pending_checks.emplace_back(i, ok(checks[i]));
        }
    }
    std::vector<std::uint8_t> latest(checks.size(), 2U);
    for (auto const& [index, green] : m_pending_checks)
    {
        if (index < latest.size())
        {
            latest[index] = green ? 1U : 0U;
        }
    }
    m_pending_checks.clear();
    for (std::size_t i = 0; i < checks.size(); ++i)
    {
        if (latest[i] != 2U && latest[i] != m_checks[i])
        {
            m_checks[i] = latest[i];
            publishKind(ROBOTIK_EVENT_ASSERTION_CHANGED,
                        checks[i].text,
                        static_cast<std::int32_t>(latest[i]));
        }
    }
    for (auto const& [name, state] : m_pending_skills)
    {
        publishKind(state == SkillState::Succeeded ? ROBOTIK_EVENT_SKILL_COMPLETED
                                                   : ROBOTIK_EVENT_SKILL_FAILED,
                    name,
                    0);
    }
    m_pending_skills.clear();
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
