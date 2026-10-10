// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Plugin/PluginManager.hpp"

#include <cstdint>
#include <cstring>
#include <dlfcn.h>
#include <utility>

namespace robotik
{

namespace
{

template <class Function>
Function symbol(void* p_library, char const* p_name, std::string& p_error)
{
    dlerror();
    void* address = dlsym(p_library, p_name);
    char const* failure = dlerror();
    if (failure != nullptr || address == nullptr)
    {
        p_error = failure != nullptr ? failure : std::string("missing symbol ") + p_name;
        return nullptr;
    }
    Function function = nullptr;
    static_assert(sizeof(Function) == sizeof(address));
    std::memcpy(&function, &address, sizeof(function));
    return function;
}

} // namespace

PluginManager::~PluginManager()
{
    unload();
}

void PluginManager::clearSymbols()
{
    m_api = {};
    m_create = nullptr;
    m_destroy = nullptr;
    m_instance = nullptr;
    m_id.clear();
}

RobotikPluginStatus PluginManager::fail(RobotikPluginStatus p_status, std::string p_message)
{
    m_error = std::move(p_message);
    return p_status;
}

RobotikPluginStatus PluginManager::load(std::string const& p_path)
{
    if (m_state != PluginState::Unloaded)
    {
        return fail(ROBOTIK_PLUGIN_ERR_STATE, "plugin is already loaded");
    }
    m_error.clear();
    m_library = dlopen(p_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (m_library == nullptr)
    {
        char const* failure = dlerror();
        return fail(ROBOTIK_PLUGIN_ERR_NOT_FOUND,
                    failure != nullptr ? failure : "dlopen failed");
    }

    using AbiFn = std::uint32_t (*)();
    using QueryFn = bool (*)(RobotikPluginInfo*, RobotikPluginVTable*);
    using CreateFn = RobotikPlugin* (*)(RobotikHostAPI const*);
    using DestroyFn = void (*)(RobotikPlugin*);

    auto missing = [this](std::string p_message) {
        unload();
        return fail(ROBOTIK_PLUGIN_ERR_NOT_FOUND, std::move(p_message));
    };
    AbiFn const abi = symbol<AbiFn>(m_library, "robotik_plugin_abi_version", m_error);
    if (abi == nullptr)
    {
        return missing(m_error);
    }
    if (abi() != ROBOTIK_PLUGIN_ABI_VERSION)
    {
        unload();
        return fail(ROBOTIK_PLUGIN_ERR_ABI, "plugin ABI version is not compatible");
    }
    QueryFn const query = symbol<QueryFn>(m_library, "robotik_plugin_query", m_error);
    if (query == nullptr)
    {
        return missing(m_error);
    }
    m_create = symbol<CreateFn>(m_library, "robotik_plugin_create", m_error);
    if (m_create == nullptr)
    {
        return missing(m_error);
    }
    m_destroy = symbol<DestroyFn>(m_library, "robotik_plugin_destroy", m_error);
    if (m_destroy == nullptr)
    {
        return missing(m_error);
    }

    RobotikPluginInfo info{};
    info.size = static_cast<std::uint32_t>(sizeof(info));
    info.abi_version = ROBOTIK_PLUGIN_ABI_VERSION;
    m_api = {};
    m_api.size = static_cast<std::uint32_t>(sizeof(m_api));
    m_api.abi_version = ROBOTIK_PLUGIN_ABI_VERSION;
    if (!query(&info, &m_api) || m_api.setup == nullptr || m_api.start == nullptr ||
        m_api.update == nullptr || m_api.stop == nullptr || m_api.shutdown == nullptr)
    {
        unload();
        return fail(ROBOTIK_PLUGIN_ERR_ABI, "plugin query failed");
    }
    m_id = info.id;
    m_state = PluginState::Loaded;
    m_error.clear();
    return ROBOTIK_PLUGIN_OK;
}

RobotikPluginStatus PluginManager::create(RobotikHostAPI const* p_host)
{
    if (m_state != PluginState::Loaded || m_create == nullptr)
    {
        return fail(ROBOTIK_PLUGIN_ERR_STATE, "plugin is not loaded");
    }
    m_instance = m_create(p_host);
    if (m_instance == nullptr)
    {
        return fail(ROBOTIK_PLUGIN_ERR_RUNTIME, "plugin create failed");
    }
    m_state = PluginState::Created;
    m_error.clear();
    return ROBOTIK_PLUGIN_OK;
}

RobotikPluginStatus PluginManager::setup(std::string const& p_scenario_id)
{
    if ((m_state != PluginState::Created && m_state != PluginState::Stopped &&
         m_state != PluginState::Shutdown) ||
        m_instance == nullptr || m_api.setup == nullptr)
    {
        return fail(ROBOTIK_PLUGIN_ERR_STATE, "plugin cannot be prepared");
    }
    RobotikPluginStatus const status = m_api.setup(m_instance, p_scenario_id.c_str());
    if (status != ROBOTIK_PLUGIN_OK)
    {
        return status;
    }
    m_state = PluginState::Setup;
    m_error.clear();
    return ROBOTIK_PLUGIN_OK;
}

RobotikPluginStatus PluginManager::start()
{
    if ((m_state != PluginState::Setup && m_state != PluginState::Stopped) ||
        m_instance == nullptr)
    {
        return fail(ROBOTIK_PLUGIN_ERR_STATE, "plugin cannot start");
    }
    RobotikPluginStatus const status = m_api.start(m_instance);
    if (status != ROBOTIK_PLUGIN_OK)
    {
        return status;
    }
    m_state = PluginState::Running;
    m_error.clear();
    return ROBOTIK_PLUGIN_OK;
}

RobotikPluginStatus PluginManager::update(double p_dt)
{
    if (m_state != PluginState::Running || m_instance == nullptr)
    {
        return fail(ROBOTIK_PLUGIN_ERR_STATE, "plugin is not running");
    }
    RobotikPluginStatus const status = m_api.update(m_instance, p_dt);
    if (status != ROBOTIK_PLUGIN_OK)
    {
        return status;
    }
    return ROBOTIK_PLUGIN_OK;
}

RobotikPluginStatus PluginManager::onEvent(RobotikEvent const& p_event)
{
    if (m_instance == nullptr ||
        (m_state != PluginState::Setup && m_state != PluginState::Running &&
         m_state != PluginState::Paused))
    {
        return fail(ROBOTIK_PLUGIN_ERR_STATE, "plugin cannot receive events");
    }
    if (m_api.on_event == nullptr)
    {
        return ROBOTIK_PLUGIN_OK;
    }
    return m_api.on_event(m_instance, &p_event);
}

RobotikPluginStatus PluginManager::pause(bool p_paused)
{
    if (p_paused)
    {
        if (m_state != PluginState::Running)
        {
            return fail(ROBOTIK_PLUGIN_ERR_STATE, "plugin is not running");
        }
    }
    else if (m_state != PluginState::Paused)
    {
        return fail(ROBOTIK_PLUGIN_ERR_STATE, "plugin is not paused");
    }
    if (m_api.pause != nullptr && m_instance != nullptr)
    {
        RobotikPluginStatus const status = m_api.pause(m_instance, p_paused ? 1 : 0);
        if (status != ROBOTIK_PLUGIN_OK)
        {
            return status;
        }
    }
    m_state = p_paused ? PluginState::Paused : PluginState::Running;
    m_error.clear();
    return ROBOTIK_PLUGIN_OK;
}

RobotikPluginStatus PluginManager::stop()
{
    if (m_state != PluginState::Running && m_state != PluginState::Paused)
    {
        return ROBOTIK_PLUGIN_OK;
    }
    RobotikPluginStatus const status =
        m_instance != nullptr ? m_api.stop(m_instance) : ROBOTIK_PLUGIN_OK;
    m_state = PluginState::Stopped;
    if (status != ROBOTIK_PLUGIN_OK)
    {
        return status;
    }
    m_error.clear();
    return ROBOTIK_PLUGIN_OK;
}

RobotikPluginStatus PluginManager::shutdown()
{
    if (m_state == PluginState::Unloaded || m_state == PluginState::Loaded ||
        m_state == PluginState::Shutdown || m_instance == nullptr)
    {
        return ROBOTIK_PLUGIN_OK;
    }
    if (m_state == PluginState::Running || m_state == PluginState::Paused)
    {
        stop();
    }
    RobotikPluginStatus const status = m_api.shutdown(m_instance);
    m_state = PluginState::Shutdown;
    if (status != ROBOTIK_PLUGIN_OK)
    {
        return status;
    }
    m_error.clear();
    return ROBOTIK_PLUGIN_OK;
}

RobotikPluginStatus PluginManager::destroy()
{
    if (m_instance == nullptr)
    {
        if (m_state == PluginState::Created || m_state == PluginState::Shutdown ||
            m_state == PluginState::Stopped || m_state == PluginState::Setup)
        {
            m_state = PluginState::Loaded;
        }
        return ROBOTIK_PLUGIN_OK;
    }
    if (m_state == PluginState::Running || m_state == PluginState::Paused)
    {
        stop();
    }
    if (m_state != PluginState::Shutdown && m_state != PluginState::Created &&
        m_state != PluginState::Stopped && m_state != PluginState::Setup)
    {
        shutdown();
    }
    if (m_destroy != nullptr)
    {
        m_destroy(m_instance);
    }
    m_instance = nullptr;
    m_state = m_library != nullptr ? PluginState::Loaded : PluginState::Unloaded;
    m_error.clear();
    return ROBOTIK_PLUGIN_OK;
}

RobotikPluginStatus PluginManager::unload()
{
    if (m_instance != nullptr)
    {
        destroy();
    }
    clearSymbols();
    if (m_library != nullptr)
    {
        dlclose(m_library);
        m_library = nullptr;
    }
    m_state = PluginState::Unloaded;
    return ROBOTIK_PLUGIN_OK;
}

} // namespace robotik
