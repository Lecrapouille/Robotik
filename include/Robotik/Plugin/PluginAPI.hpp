// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PluginAPI.hpp
//! @brief C++ helpers for a demo plugin. The exported symbols stay C.
#pragma once

#include "Robotik/Plugin/PluginABI.h"

#include <cstdint>
#include <cstring>
#include <exception>
#include <new>
#include <string>
#include <string_view>

namespace robotik
{

class Mission;
class Simulation;

// ****************************************************************************
//! @brief Identity published by @c robotik_plugin_query, before any instance.
// ****************************************************************************
struct PluginInfo
{
    std::string id;
    std::string name;
    std::string version;
    std::string category;
};

// ****************************************************************************
//! @brief Drawing surface implemented by the host. Valid only during a draw.
// ****************************************************************************
class Canvas
{
public:

    explicit Canvas(RobotikCanvas const* p_canvas = nullptr) : m_canvas(p_canvas)
    {
    }

    void text(std::string_view p_line) const
    {
        if (m_canvas == nullptr || m_canvas->text == nullptr ||
            m_canvas->size < sizeof(RobotikCanvas))
        {
            return;
        }
        std::string line(p_line);
        m_canvas->text(m_canvas, line.c_str());
    }

    void textColored(float p_red,
                     float p_green,
                     float p_blue,
                     std::string_view p_line) const
    {
        if (m_canvas == nullptr || m_canvas->text_colored == nullptr ||
            m_canvas->size < sizeof(RobotikCanvas))
        {
            return;
        }
        std::string line(p_line);
        m_canvas->text_colored(m_canvas, p_red, p_green, p_blue, line.c_str());
    }

    [[nodiscard]] bool button(std::string_view p_label) const
    {
        if (m_canvas == nullptr || m_canvas->button == nullptr)
        {
            return false;
        }
        std::string label(p_label);
        return m_canvas->button(m_canvas, label.c_str()) != 0;
    }

    [[nodiscard]] bool checkbox(std::string_view p_label, bool& p_value) const
    {
        if (m_canvas == nullptr || m_canvas->checkbox == nullptr)
        {
            return false;
        }
        int value = p_value ? 1 : 0;
        std::string label(p_label);
        bool const changed = m_canvas->checkbox(m_canvas, label.c_str(), &value) != 0;
        p_value = value != 0;
        return changed;
    }

    [[nodiscard]] bool sliderInt(std::string_view p_label, int& p_value, int p_min, int p_max) const
    {
        if (m_canvas == nullptr || m_canvas->slider_int == nullptr)
        {
            return false;
        }
        std::string label(p_label);
        return m_canvas->slider_int(m_canvas, label.c_str(), &p_value, p_min, p_max) != 0;
    }

    void separator() const
    {
        if (m_canvas != nullptr && m_canvas->separator != nullptr)
        {
            m_canvas->separator(m_canvas);
        }
    }

private:

    RobotikCanvas const* m_canvas = nullptr;
};

// ****************************************************************************
//! @brief Services of the host, seen from the plugin.
// ****************************************************************************
class Host
{
public:

    Host() = default;

    Host(RobotikHostAPI const* p_api) : m_api(p_api) {}

    [[nodiscard]] bool valid() const
    {
        return m_api != nullptr && m_api->host != nullptr;
    }

    void log(int p_level, std::string_view p_message) const
    {
        call(m_api != nullptr ? m_api->log : nullptr, p_level, p_message);
    }

    void error(std::string_view p_message) const
    {
        if (m_api == nullptr || m_api->report_error == nullptr || m_api->host == nullptr)
        {
            return;
        }
        std::string text(p_message);
        m_api->report_error(m_api->host, text.c_str());
    }

    void subscribe(RobotikEventKind p_kind) const
    {
        if (m_api != nullptr && m_api->subscribe != nullptr && m_api->host != nullptr)
        {
            m_api->subscribe(m_api->host, static_cast<std::uint32_t>(p_kind));
        }
    }

    std::uint32_t addMenuItem(std::string_view p_menu,
                              std::string_view p_item,
                              void (*p_callback)(void*),
                              void* p_user) const
    {
        if (m_api == nullptr || m_api->add_menu_item == nullptr || m_api->host == nullptr)
        {
            return 0;
        }
        std::string menu(p_menu);
        std::string item(p_item);
        return m_api->add_menu_item(m_api->host, menu.c_str(), item.c_str(), p_callback, p_user);
    }

    std::uint32_t addPanel(std::string_view p_id,
                           std::string_view p_title,
                           void (*p_draw)(void*, RobotikCanvas const*),
                           void* p_user) const
    {
        if (m_api == nullptr || m_api->add_panel == nullptr || m_api->host == nullptr)
        {
            return 0;
        }
        std::string id(p_id);
        std::string title(p_title);
        return m_api->add_panel(m_api->host, id.c_str(), title.c_str(), p_draw, p_user);
    }

    void removeUi(std::uint32_t p_handle) const
    {
        if (m_api != nullptr && m_api->remove_ui != nullptr && m_api->host != nullptr)
        {
            m_api->remove_ui(m_api->host, p_handle);
        }
    }

    [[nodiscard]] std::string scenarioPath() const
    {
        if (m_api == nullptr || m_api->scenario_path == nullptr || m_api->host == nullptr)
        {
            return {};
        }
        char const* path = m_api->scenario_path(m_api->host);
        return path != nullptr ? std::string(path) : std::string{};
    }

    [[nodiscard]] Simulation* simulation() const;

    void bindMission(Mission& p_mission) const;

private:

    void call(RobotikLogFn p_function, int p_level, std::string_view p_message) const
    {
        if (p_function == nullptr || m_api == nullptr || m_api->host == nullptr)
        {
            return;
        }
        std::string text(p_message);
        p_function(m_api->host, p_level, text.c_str());
    }

    RobotikHostAPI const* m_api = nullptr;
};

// ****************************************************************************
//! @brief Base of a plugin instance. Override the steps the demo needs.
//!
//! @c describe is static because the host reads it before @c create.
// ****************************************************************************
class Plugin
{
public:

    virtual ~Plugin() = default;

    virtual RobotikPluginStatus setup(Host& /*p_host*/, std::string_view /*p_scenario*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }

    virtual RobotikPluginStatus start(Host& /*p_host*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }

    virtual RobotikPluginStatus update(Host& /*p_host*/, double /*p_dt*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }

    virtual RobotikPluginStatus onEvent(Host& /*p_host*/, RobotikEvent const& /*p_event*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }

    virtual RobotikPluginStatus pause(Host& /*p_host*/, bool /*p_paused*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }

    virtual RobotikPluginStatus stop(Host& /*p_host*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }

    virtual RobotikPluginStatus shutdown(Host& /*p_host*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }
};

namespace plugin_detail
{

template <class T>
struct Box
{
    RobotikHostAPI api{};
    T object;
};

template <class T>
Box<T>* boxOf(RobotikPlugin* p_plugin)
{
    return reinterpret_cast<Box<T>*>(p_plugin);
}

template <class T>
Host hostOf(Box<T>* p_box)
{
    return Host(p_box != nullptr ? &p_box->api : nullptr);
}

inline void report(RobotikHostAPI const* p_api, char const* p_message)
{
    if (p_api != nullptr && p_api->report_error != nullptr && p_api->host != nullptr &&
        p_message != nullptr)
    {
        p_api->report_error(p_api->host, p_message);
    }
}

inline bool copyField(char* p_dest, std::size_t p_capacity, std::string_view p_value)
{
    if (p_dest == nullptr || p_value.size() + 1u > p_capacity)
    {
        return false;
    }
    if (!p_value.empty())
    {
        std::memcpy(p_dest, p_value.data(), p_value.size());
    }
    p_dest[p_value.size()] = '\0';
    return true;
}

template <class T>
RobotikPluginStatus setupThunk(RobotikPlugin* p_plugin, char const* p_scenario)
{
    Box<T>* self = boxOf<T>(p_plugin);
    if (self == nullptr)
    {
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    try
    {
        Host host = hostOf(self);
        return self->object.setup(host, p_scenario != nullptr ? p_scenario : "");
    }
    catch (std::exception const& error)
    {
        report(&self->api, error.what());
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    catch (...)
    {
        report(&self->api, "plugin setup failed");
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
}

template <class T>
RobotikPluginStatus startThunk(RobotikPlugin* p_plugin)
{
    Box<T>* self = boxOf<T>(p_plugin);
    if (self == nullptr)
    {
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    try
    {
        Host host = hostOf(self);
        return self->object.start(host);
    }
    catch (std::exception const& error)
    {
        report(&self->api, error.what());
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    catch (...)
    {
        report(&self->api, "plugin start failed");
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
}

template <class T>
RobotikPluginStatus updateThunk(RobotikPlugin* p_plugin, double p_dt)
{
    Box<T>* self = boxOf<T>(p_plugin);
    if (self == nullptr)
    {
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    try
    {
        Host host = hostOf(self);
        return self->object.update(host, p_dt);
    }
    catch (std::exception const& error)
    {
        report(&self->api, error.what());
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    catch (...)
    {
        report(&self->api, "plugin update failed");
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
}

template <class T>
RobotikPluginStatus eventThunk(RobotikPlugin* p_plugin, RobotikEvent const* p_event)
{
    Box<T>* self = boxOf<T>(p_plugin);
    if (self == nullptr || p_event == nullptr)
    {
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    try
    {
        Host host = hostOf(self);
        return self->object.onEvent(host, *p_event);
    }
    catch (std::exception const& error)
    {
        report(&self->api, error.what());
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    catch (...)
    {
        report(&self->api, "plugin event failed");
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
}

template <class T>
RobotikPluginStatus pauseThunk(RobotikPlugin* p_plugin, int p_paused)
{
    Box<T>* self = boxOf<T>(p_plugin);
    if (self == nullptr)
    {
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    try
    {
        Host host = hostOf(self);
        return self->object.pause(host, p_paused != 0);
    }
    catch (std::exception const& error)
    {
        report(&self->api, error.what());
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    catch (...)
    {
        report(&self->api, "plugin pause failed");
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
}

template <class T>
RobotikPluginStatus stopThunk(RobotikPlugin* p_plugin)
{
    Box<T>* self = boxOf<T>(p_plugin);
    if (self == nullptr)
    {
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    try
    {
        Host host = hostOf(self);
        return self->object.stop(host);
    }
    catch (std::exception const& error)
    {
        report(&self->api, error.what());
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    catch (...)
    {
        report(&self->api, "plugin stop failed");
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
}

template <class T>
RobotikPluginStatus shutdownThunk(RobotikPlugin* p_plugin)
{
    Box<T>* self = boxOf<T>(p_plugin);
    if (self == nullptr)
    {
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    try
    {
        Host host = hostOf(self);
        return self->object.shutdown(host);
    }
    catch (std::exception const& error)
    {
        report(&self->api, error.what());
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    catch (...)
    {
        report(&self->api, "plugin shutdown failed");
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
}

template <class CFunction, class CppFunction>
void assignFunction(CFunction* p_dest, CppFunction p_function)
{
    static_assert(sizeof(CFunction) == sizeof(CppFunction));
    std::memcpy(p_dest, &p_function, sizeof(p_function));
}

template <class T>
void fillTable(RobotikPluginVTable* p_api)
{
    p_api->size = static_cast<std::uint32_t>(sizeof(RobotikPluginVTable));
    p_api->abi_version = ROBOTIK_PLUGIN_ABI_VERSION;
    assignFunction(&p_api->setup, &setupThunk<T>);
    assignFunction(&p_api->start, &startThunk<T>);
    assignFunction(&p_api->update, &updateThunk<T>);
    assignFunction(&p_api->on_event, &eventThunk<T>);
    assignFunction(&p_api->pause, &pauseThunk<T>);
    assignFunction(&p_api->stop, &stopThunk<T>);
    assignFunction(&p_api->shutdown, &shutdownThunk<T>);
}

template <class T>
bool queryPlugin(RobotikPluginInfo* p_info, RobotikPluginVTable* p_api)
{
    if (p_info == nullptr || p_api == nullptr)
    {
        return false;
    }
    if (p_info->size < sizeof(RobotikPluginInfo) ||
        p_api->size < sizeof(RobotikPluginVTable))
    {
        return false;
    }
    if (p_info->abi_version != ROBOTIK_PLUGIN_ABI_VERSION ||
        p_api->abi_version != ROBOTIK_PLUGIN_ABI_VERSION)
    {
        return false;
    }
    try
    {
        PluginInfo const described = T::describe();
        if (!copyField(p_info->id, sizeof(p_info->id), described.id) ||
            !copyField(p_info->name, sizeof(p_info->name), described.name) ||
            !copyField(p_info->version, sizeof(p_info->version), described.version) ||
            !copyField(p_info->category, sizeof(p_info->category), described.category))
        {
            return false;
        }
        fillTable<T>(p_api);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

template <class T>
RobotikPlugin* createPlugin(RobotikHostAPI const* p_host)
{
    if (p_host == nullptr || p_host->size < sizeof(RobotikHostAPI) ||
        p_host->abi_version != ROBOTIK_PLUGIN_ABI_VERSION || p_host->host == nullptr)
    {
        return nullptr;
    }
    try
    {
        auto* plugin = new Box<T>();
        plugin->api = *p_host;
        return reinterpret_cast<RobotikPlugin*>(plugin);
    }
    catch (...)
    {
        report(p_host, "plugin create failed");
        return nullptr;
    }
}

template <class T>
void destroyPlugin(RobotikPlugin* p_plugin)
{
    delete boxOf<T>(p_plugin);
}

} // namespace plugin_detail

} // namespace robotik

// ****************************************************************************
//! @brief Exports the C ABI for @p Type. Use it in one translation unit.
//!
//! @p Type is default-constructible and provides @c static PluginInfo describe().
// ****************************************************************************
#define ROBOTIK_EXPORT_PLUGIN(Type)                                                  \
    extern "C"                                                                       \
    {                                                                                \
    uint32_t robotik_plugin_abi_version(void)                                        \
    {                                                                                \
        return ROBOTIK_PLUGIN_ABI_VERSION;                                           \
    }                                                                                \
    bool robotik_plugin_query(RobotikPluginInfo* info, RobotikPluginVTable* api)     \
    {                                                                                \
        return robotik::plugin_detail::queryPlugin<Type>(info, api);                \
    }                                                                                \
    RobotikPlugin* robotik_plugin_create(RobotikHostAPI const* host)                \
    {                                                                                \
        return robotik::plugin_detail::createPlugin<Type>(host);                    \
    }                                                                                \
    void robotik_plugin_destroy(RobotikPlugin* plugin)                              \
    {                                                                                \
        robotik::plugin_detail::destroyPlugin<Type>(plugin);                         \
    }                                                                                \
    }
