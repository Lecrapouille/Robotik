// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PluginAPI.hpp
/// C++ helpers for a demo plugin. The exported symbols stay C.
///
/// A plugin derives from @ref Plugin, gives a static @c describe(), and
/// exports itself with @ref ROBOTIK_EXPORT_PLUGIN. The helpers turn the C
/// calls of @c PluginABI.h into virtual calls, and C++ exceptions into error
/// statuses: nothing C++ crosses the border.
///
/// @code
/// class MyPlugin final : public robotik::Plugin
/// {
/// public:
///     static robotik::PluginInfo describe() { return { .id =
///     "robotik.my_plugin" }; }
///
///     RobotikPluginStatus setup(robotik::Host& p_host, std::string_view
///     p_scenario) override
///     {
///         if (p_scenario != "demo")
///             return ROBOTIK_PLUGIN_ERR_SETUP;
///         m_mission = std::make_unique<MyMission>();
///         p_host.bindMission(*m_mission);
///         return ROBOTIK_PLUGIN_OK;
///     }
/// private:
///     std::unique_ptr<MyMission> m_mission;
/// };
///
/// ROBOTIK_EXPORT_PLUGIN(MyPlugin)
/// @endcode
#pragma once

#include "Robotik/Package/Plugin/PluginABI.h"

#include <cstddef>
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
/// Identity published by @c robotik_plugin_query, before any instance.
///
/// @c plugin.yaml is the description of the package: the host only checks
/// that @ref id matches it. The other fields are optional.
///
/// @code
/// static robotik::PluginInfo describe()
/// {
///     return { .id = "robotik.hello_world", .name = "Hello world" };
/// }
/// @endcode
// ****************************************************************************
struct PluginInfo
{
    /// Plugin id; must equal @c plugin.id of @c plugin.yaml.
    std::string id{};
    /// Name shown to the user.
    std::string name{};
    /// Version text.
    std::string version{};
    /// Group in the plugin list.
    std::string category{};
};

// ****************************************************************************
/// Drawing surface implemented by the host. Valid only during a draw.
///
/// Every method does nothing (or returns false) when the host does not
/// provide the widget, so a panel also works with an older host.
///
/// @code
/// static void onDraw(void* p_user, RobotikCanvas const* p_canvas)
/// {
///     auto* self = static_cast<MyPlugin*>(p_user);
///     robotik::Canvas canvas(p_canvas);
///     canvas.text("Laps: " + std::to_string(self->laps));
///     if (canvas.button("Reset"))
///         self->laps = 0;
///     canvas.checkbox("Show path", self->show_path);
/// }
/// @endcode
// ****************************************************************************
class Canvas
{
public:

    /// @param p_canvas Canvas given by the host to the draw function.
    explicit Canvas(RobotikCanvas const* p_canvas = nullptr)
        : m_canvas(p_canvas)
    {
    }

    // -------------------------------------------------------------------------
    /// Writes one line of text.
    /// @param p_line Text to write.
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    /// Writes one line of text in a color.
    /// @param p_red Red in [0, 1].
    /// @param p_green Green in [0, 1].
    /// @param p_blue Blue in [0, 1].
    /// @param p_line Text to write.
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    /// Draws a button.
    /// @param p_label Text on the button.
    /// @return True when clicked.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool button(std::string_view p_label) const
    {
        if (m_canvas == nullptr || m_canvas->button == nullptr)
        {
            return false;
        }
        std::string label(p_label);
        return m_canvas->button(m_canvas, label.c_str()) != 0;
    }

    // -------------------------------------------------------------------------
    /// Draws a check box bound to a boolean.
    /// @param p_label Text next to the box.
    /// @param p_value Value shown and changed by the user.
    /// @return True when the user changed it.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool checkbox(std::string_view p_label, bool& p_value) const
    {
        if (m_canvas == nullptr || m_canvas->checkbox == nullptr)
        {
            return false;
        }
        int value = p_value ? 1 : 0;
        std::string label(p_label);
        bool const changed =
            m_canvas->checkbox(m_canvas, label.c_str(), &value) != 0;
        p_value = value != 0;
        return changed;
    }

    // -------------------------------------------------------------------------
    /// Draws an integer slider.
    /// @param p_label Text next to the slider.
    /// @param p_value Value shown and changed by the user.
    /// @param p_min Lowest value.
    /// @param p_max Highest value.
    /// @return True when the user changed it.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool sliderInt(std::string_view p_label,
                                 int& p_value,
                                 int p_min,
                                 int p_max) const
    {
        if (m_canvas == nullptr || m_canvas->slider_int == nullptr)
        {
            return false;
        }
        std::string label(p_label);
        return m_canvas->slider_int(
                   m_canvas, label.c_str(), &p_value, p_min, p_max) != 0;
    }

    // -------------------------------------------------------------------------
    /// Draws a radio button.
    /// @param p_label Text next to the button.
    /// @param p_active True to draw it selected.
    /// @return True when clicked.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool radio(std::string_view p_label, bool p_active) const
    {
        if (m_canvas == nullptr ||
            m_canvas->size <
                offsetof(RobotikCanvas, radio) + sizeof(m_canvas->radio) ||
            m_canvas->radio == nullptr)
        {
            return false;
        }
        std::string label(p_label);
        return m_canvas->radio(m_canvas, label.c_str(), p_active ? 1 : 0) != 0;
    }

    /// Draws a horizontal line.
    void separator() const
    {
        if (m_canvas != nullptr && m_canvas->separator != nullptr)
        {
            m_canvas->separator(m_canvas);
        }
    }

private:

    /// C canvas of the host, or null.
    RobotikCanvas const* m_canvas = nullptr;
};

// ****************************************************************************
/// Services of the host, seen from the plugin.
///
/// A cheap copyable view of the C API; every method does nothing when the
/// host lacks the service.
///
/// @code
/// p_host.log(ROBOTIK_LOG_INFO, "ready");
/// p_host.subscribe(ROBOTIK_EVENT_KEY_PRESSED);
/// p_host.addMenuItem("My plugin", "Home", &onHome, this);
/// p_host.addPanel("my_plugin", "My plugin", &onDraw, this);
/// @endcode
// ****************************************************************************
class Host
{
public:

    /// Host without services.
    Host() = default;

    /// @param p_api C services of the host; must outlive this view.
    Host(RobotikHostAPI const* p_api) : m_api(p_api) {}

    // -------------------------------------------------------------------------
    /// @return True when connected to a host.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool valid() const
    {
        return m_api != nullptr && m_api->host != nullptr;
    }

    // -------------------------------------------------------------------------
    /// Writes a message in the host log.
    /// @param p_level @c ROBOTIK_LOG_INFO, @c ROBOTIK_LOG_WARNING or
    ///   @c ROBOTIK_LOG_ERROR.
    /// @param p_message Text to write.
    // -------------------------------------------------------------------------
    void log(int p_level, std::string_view p_message) const
    {
        call(m_api != nullptr ? m_api->log : nullptr, p_level, p_message);
    }

    // -------------------------------------------------------------------------
    /// Reports an error to the host, shown to the user.
    /// @param p_message What went wrong.
    // -------------------------------------------------------------------------
    void error(std::string_view p_message) const
    {
        if (m_api == nullptr || m_api->report_error == nullptr ||
            m_api->host == nullptr)
        {
            return;
        }
        std::string text(p_message);
        m_api->report_error(m_api->host, text.c_str());
    }

    // -------------------------------------------------------------------------
    /// Asks for one kind of event; only those reach @ref Plugin::onEvent.
    /// @param p_kind Kind of event.
    // -------------------------------------------------------------------------
    void subscribe(RobotikEventKind p_kind) const
    {
        if (m_api != nullptr && m_api->subscribe != nullptr &&
            m_api->host != nullptr)
        {
            m_api->subscribe(m_api->host, static_cast<std::uint32_t>(p_kind));
        }
    }

    // -------------------------------------------------------------------------
    /// Adds a menu entry.
    /// @param p_menu Menu holding the entry.
    /// @param p_item Text of the entry.
    /// @param p_callback Called with @p_user when the user picks it.
    /// @param p_user Pointer given back to @p_callback (often @c this).
    /// @return Handle for @ref removeUi, or 0 on failure.
    // -------------------------------------------------------------------------
    std::uint32_t addMenuItem(std::string_view p_menu,
                              std::string_view p_item,
                              void (*p_callback)(void*),
                              void* p_user) const
    {
        if (m_api == nullptr || m_api->add_menu_item == nullptr ||
            m_api->host == nullptr)
        {
            return 0;
        }
        std::string menu(p_menu);
        std::string item(p_item);
        return m_api->add_menu_item(
            m_api->host, menu.c_str(), item.c_str(), p_callback, p_user);
    }

    // -------------------------------------------------------------------------
    /// Adds a panel, drawn every frame.
    /// @param p_id Stable key, to remember the layout.
    /// @param p_title Window title.
    /// @param p_draw Called with @p_user and a canvas (see @ref Canvas).
    /// @param p_user Pointer given back to @p_draw (often @c this).
    /// @return Handle for @ref removeUi, or 0 on failure.
    // -------------------------------------------------------------------------
    std::uint32_t addPanel(std::string_view p_id,
                           std::string_view p_title,
                           void (*p_draw)(void*, RobotikCanvas const*),
                           void* p_user) const
    {
        if (m_api == nullptr || m_api->add_panel == nullptr ||
            m_api->host == nullptr)
        {
            return 0;
        }
        std::string id(p_id);
        std::string title(p_title);
        return m_api->add_panel(
            m_api->host, id.c_str(), title.c_str(), p_draw, p_user);
    }

    // -------------------------------------------------------------------------
    /// Removes a menu entry or a panel.
    /// @param p_handle Handle returned by @ref addMenuItem or @ref addPanel.
    // -------------------------------------------------------------------------
    void removeUi(std::uint32_t p_handle) const
    {
        if (m_api != nullptr && m_api->remove_ui != nullptr &&
            m_api->host != nullptr)
        {
            m_api->remove_ui(m_api->host, p_handle);
        }
    }

    // -------------------------------------------------------------------------
    /// @return The scenario file being run, or empty.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string scenarioPath() const
    {
        if (m_api == nullptr || m_api->scenario_path == nullptr ||
            m_api->host == nullptr)
        {
            return {};
        }
        char const* path = m_api->scenario_path(m_api->host);
        return path != nullptr ? std::string(path) : std::string{};
    }

    // -------------------------------------------------------------------------
    /// @return The simulation of the run, or null (before it is built, or
    ///   after it is destroyed).
    // -------------------------------------------------------------------------
    [[nodiscard]] Simulation* simulation() const;

    // -------------------------------------------------------------------------
    /// Gives the host the mission of the plugin; the host passes it to the
    /// simulation it builds. Call it in @ref Plugin::setup.
    /// @param p_mission Mission owned by the plugin; must outlive the run.
    // -------------------------------------------------------------------------
    void bindMission(Mission& p_mission) const;

private:

    /// Calls a log function of the host with a C string.
    void
    call(RobotikLogFn p_function, int p_level, std::string_view p_message) const
    {
        if (p_function == nullptr || m_api == nullptr || m_api->host == nullptr)
        {
            return;
        }
        std::string text(p_message);
        p_function(m_api->host, p_level, text.c_str());
    }

    /// C services of the host, or null.
    RobotikHostAPI const* m_api = nullptr;
};

// ****************************************************************************
/// Base of a plugin instance. Override the steps the demo needs.
///
/// @c describe is static because the host reads it before @c create.
/// Every step returns @c ROBOTIK_PLUGIN_OK by default. An exception thrown
/// by a step is caught, reported with @ref Host::error and turned into
/// @c ROBOTIK_PLUGIN_ERR_RUNTIME.
///
/// The host calls: @ref setup, @ref start, then @ref update after each
/// simulation step (and @ref onEvent, @ref pause when they happen), then
/// @ref stop and @ref shutdown.
///
/// @code
/// RobotikPluginStatus MyPlugin::onEvent(robotik::Host&, RobotikEvent const&
/// p_event)
/// {
///     if (p_event.kind == ROBOTIK_EVENT_KEY_PRESSED && p_event.code == 'H')
///         home();
///     return ROBOTIK_PLUGIN_OK;
/// }
/// @endcode
// ****************************************************************************
class Plugin
{
public:

    virtual ~Plugin() = default;

    // -------------------------------------------------------------------------
    /// Prepares a scenario, before the simulation exists: create the mission
    /// and bind it (@ref Host::bindMission), add menus and panels.
    /// @param p_host Services of the host.
    /// @param p_scenario Scenario id, as listed in @c plugin.yaml.
    /// @return @c ROBOTIK_PLUGIN_ERR_SETUP for an unknown scenario.
    // -------------------------------------------------------------------------
    virtual RobotikPluginStatus setup(Host& /*p_host*/,
                                      std::string_view /*p_scenario*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }

    // -------------------------------------------------------------------------
    /// The run starts.
    /// @param p_host Services of the host.
    /// @return The status.
    // -------------------------------------------------------------------------
    virtual RobotikPluginStatus start(Host& /*p_host*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }

    // -------------------------------------------------------------------------
    /// Called after each simulation step.
    /// @param p_host Services of the host.
    /// @param p_dt Duration of the step (s).
    /// @return The status.
    // -------------------------------------------------------------------------
    virtual RobotikPluginStatus update(Host& /*p_host*/, double /*p_dt*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }

    // -------------------------------------------------------------------------
    /// Receives an event subscribed with @ref Host::subscribe.
    /// @param p_host Services of the host.
    /// @param p_event The event.
    /// @return The status.
    // -------------------------------------------------------------------------
    virtual RobotikPluginStatus onEvent(Host& /*p_host*/,
                                        RobotikEvent const& /*p_event*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }

    // -------------------------------------------------------------------------
    /// The run is paused or resumed.
    /// @param p_host Services of the host.
    /// @param p_paused True when paused.
    /// @return The status.
    // -------------------------------------------------------------------------
    virtual RobotikPluginStatus pause(Host& /*p_host*/, bool /*p_paused*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }

    // -------------------------------------------------------------------------
    /// The run ended.
    /// @param p_host Services of the host.
    /// @return The status.
    // -------------------------------------------------------------------------
    virtual RobotikPluginStatus stop(Host& /*p_host*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }

    // -------------------------------------------------------------------------
    /// The host destroyed the simulation: free the mission and the rest.
    /// @param p_host Services of the host.
    /// @return The status.
    // -------------------------------------------------------------------------
    virtual RobotikPluginStatus shutdown(Host& /*p_host*/)
    {
        return ROBOTIK_PLUGIN_OK;
    }
};

/// Glue behind @ref ROBOTIK_EXPORT_PLUGIN. Not for plugin code.
namespace plugin_detail
{

// ----------------------------------------------------------------------------
/// What the host sees as a @c RobotikPlugin: a copy of the host API and the
/// C++ plugin object.
/// @tparam T Plugin class.
// ----------------------------------------------------------------------------
template <class T>
struct Box
{
    RobotikHostAPI api{}; ///< Copy of the services of the host.
    T object;             ///< The plugin.
};

/// @return The box behind an opaque plugin pointer.
template <class T>
Box<T>* boxOf(RobotikPlugin* p_plugin)
{
    return reinterpret_cast<Box<T>*>(p_plugin);
}

/// @return A @ref Host view on the services stored in the box.
template <class T>
Host hostOf(Box<T>* p_box)
{
    return Host(p_box != nullptr ? &p_box->api : nullptr);
}

/// Reports @p_message to the host, if it can receive it.
inline void report(RobotikHostAPI const* p_api, char const* p_message)
{
    if (p_api != nullptr && p_api->report_error != nullptr &&
        p_api->host != nullptr && p_message != nullptr)
    {
        p_api->report_error(p_api->host, p_message);
    }
}

// ----------------------------------------------------------------------------
/// Copies a string into a fixed C buffer, final zero included.
/// @param p_dest Destination buffer.
/// @param p_capacity Size of the buffer.
/// @param p_value Text to copy.
/// @return False when the text does not fit.
// ----------------------------------------------------------------------------
inline bool
copyField(char* p_dest, std::size_t p_capacity, std::string_view p_value)
{
    if (p_dest == nullptr || p_value.size() + 1U > p_capacity)
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

/// C entry calling @ref Plugin::setup; exceptions become an error status.
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
        return self->object.setup(host,
                                  p_scenario != nullptr ? p_scenario : "");
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

/// C entry calling @ref Plugin::start; exceptions become an error status.
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

/// C entry calling @ref Plugin::update; exceptions become an error status.
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

/// C entry calling @ref Plugin::onEvent; exceptions become an error status.
template <class T>
RobotikPluginStatus eventThunk(RobotikPlugin* p_plugin,
                               RobotikEvent const* p_event)
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

/// C entry calling @ref Plugin::pause; exceptions become an error status.
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

/// C entry calling @ref Plugin::stop; exceptions become an error status.
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

/// C entry calling @ref Plugin::shutdown; exceptions become an error status.
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

/// Stores a C++ function pointer in a C function pointer of the same size.
template <class CFunction, class CppFunction>
void assignFunction(CFunction* p_dest, CppFunction p_function)
{
    static_assert(sizeof(CFunction) == sizeof(CppFunction));
    std::memcpy(p_dest, &p_function, sizeof(p_function));
}

/// Fills the vtable with the thunks of @p T.
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

// ----------------------------------------------------------------------------
/// Body of @c robotik_plugin_query: checks the sizes and ABI versions, then
/// copies @c T::describe() and the vtable.
/// @return False on a mismatch or a field too long.
// ----------------------------------------------------------------------------
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
            !copyField(
                p_info->version, sizeof(p_info->version), described.version) ||
            !copyField(
                p_info->category, sizeof(p_info->category), described.category))
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

// ----------------------------------------------------------------------------
/// Body of @c robotik_plugin_create: checks the host API, then builds a
/// @ref Box holding a default-constructed @p T.
/// @return The instance, or null.
// ----------------------------------------------------------------------------
template <class T>
RobotikPlugin* createPlugin(RobotikHostAPI const* p_host)
{
    if (p_host == nullptr || p_host->size < sizeof(RobotikHostAPI) ||
        p_host->abi_version != ROBOTIK_PLUGIN_ABI_VERSION ||
        p_host->host == nullptr)
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

/// Body of @c robotik_plugin_destroy.
template <class T>
void destroyPlugin(RobotikPlugin* p_plugin)
{
    delete boxOf<T>(p_plugin);
}

} // namespace plugin_detail

} // namespace robotik

// ****************************************************************************
/// Exports the C ABI for @p Type. Use it in one translation unit.
///
/// @p Type is default-constructible and provides @c static PluginInfo
/// describe().
///
/// @code
/// class MyPlugin final : public robotik::Plugin
/// {
/// public:
///     static robotik::PluginInfo describe() { return { .id =
///     "robotik.my_plugin" }; }
/// };
/// ROBOTIK_EXPORT_PLUGIN(MyPlugin)
/// @endcode
// ****************************************************************************
#define ROBOTIK_EXPORT_PLUGIN(Type)                                      \
    extern "C"                                                           \
    {                                                                    \
        uint32_t robotik_plugin_abi_version(void)                        \
        {                                                                \
            return ROBOTIK_PLUGIN_ABI_VERSION;                           \
        }                                                                \
        bool robotik_plugin_query(RobotikPluginInfo* info,               \
                                  RobotikPluginVTable* api)              \
        {                                                                \
            return robotik::plugin_detail::queryPlugin<Type>(info, api); \
        }                                                                \
        RobotikPlugin* robotik_plugin_create(RobotikHostAPI const* host) \
        {                                                                \
            return robotik::plugin_detail::createPlugin<Type>(host);     \
        }                                                                \
        void robotik_plugin_destroy(RobotikPlugin* plugin)               \
        {                                                                \
            robotik::plugin_detail::destroyPlugin<Type>(plugin);         \
        }                                                                \
    }
