// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PluginAPI.hpp
/// C++ helpers for a demo plugin. The exported symbols stay C.
///
/// A plugin says which scenario logic runs each scenario (@ref robotik::ScenarioPlugin),
/// gives a static @c describe(), and exports itself with
/// @ref ROBOTIK_EXPORT_PLUGIN. The helpers turn the C calls of
/// @c PluginABI.h into virtual calls, and C++ exceptions into error
/// statuses: nothing C++ crosses the border.
///
/// @code
/// class MyPlugin final: public robotik::ScenarioPlugin
/// {
/// public:
///     static robotik::PluginInfo describe() { return { .id = "robotik.my_plugin" }; }
///     MyPlugin() : ScenarioPlugin({ { "demo", robotik::logic<MyLogic>() } }) {}
/// };
/// ROBOTIK_EXPORT_PLUGIN(MyPlugin)
/// @endcode
///
/// @par Contract
/// - Promises: a C++ exception in a plugin becomes an error status at the
///   border; nothing C++ crosses it.
/// - Expects: @c describe().id equals @c plugin.id of @c plugin.yaml; the
///   scenario ids of the logic table are listed in it.
#pragma once

#include "RobotIK/Package/Plugin/BuiltinPlugins.hpp"
#include "RobotIK/Package/Plugin/PluginABI.h"
#include "RobotIK/Package/Plugin/PluginBuildId.hpp"
#include "RobotIK/Task/ScenarioLogic.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <initializer_list>
#include <list>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace robotik
{

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
/// A plugin cannot call Dear ImGui: the host and the plugin may be built
/// by two compilers, and a headless host has no window. So a panel draws
/// through this small set of widgets, which the simulator turns into ImGui
/// calls. Every method does nothing (or returns false) when the host does
/// not provide the widget, so a panel also works with an older host. The
/// canvas is an immediate-mode surface: draw everything at each call, keep
/// no pointer to it.
///
/// @code
/// p_host.addPanel("laps", "Laps", [this](robotik::Canvas const& p_canvas)
/// {
///     p_canvas.text("Laps: " + std::to_string(m_laps));
///     if (p_canvas.button("Reset"))
///         m_laps = 0;
///     (void)p_canvas.checkbox("Show path", m_show_path);
/// });
/// @endcode
// ****************************************************************************
class Canvas
{
public:

    /// @param p_canvas Canvas given by the host to the draw function.
    explicit Canvas(RobotIKCanvas const* p_canvas = nullptr)
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
            m_canvas->size < sizeof(RobotIKCanvas))
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
            m_canvas->size < sizeof(RobotIKCanvas))
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
                offsetof(RobotIKCanvas, radio) + sizeof(m_canvas->radio) ||
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
    RobotIKCanvas const* m_canvas = nullptr;
};

// ****************************************************************************
/// The C++ functions behind the menus and panels of one plugin instance.
///
/// The C border only takes a function pointer and a @c void*. This
/// keeps the lambdas alive until the instance is destroyed, so a plugin
/// writes @c [this] { home(); } instead of a static trampoline.
// ****************************************************************************
struct HostCallbacks
{
    /// A function and where to report its exception.
    template <class F>
    struct Slot
    {
        F function;                          ///< What the user wrote.
        RobotIKHostAPI const* api = nullptr; ///< To report an exception.
    };

    /// Menu actions (a list: the addresses given to the host stay valid).
    std::list<Slot<std::function<void()>>> actions;
    /// Panel drawings.
    std::list<Slot<std::function<void(Canvas const&)>>> panels;
};

// ****************************************************************************
/// Services of the host, seen from the plugin.
///
/// The plugin and the host talk through a C table of functions
/// (PluginABI.h), so that they can be built apart. This class wraps the
/// table in C++: strings instead of @c char*, lambdas instead of a function
/// pointer and a @c void*. It is a cheap copyable view; every method does
/// nothing when the host lacks the service, so a plugin runs in the
/// simulator and in Headless alike. Keep a copy to reach the simulation
/// later (from a menu action, for example): @ref simulation is null before
/// the run exists and after it ends.
///
/// @code
/// p_host.log(ROBOTIK_LOG_INFO, "ready");
/// p_host.subscribe(ROBOTIK_EVENT_KEY_PRESSED);
/// p_host.addMenuItem("My plugin", "Home", [this] { home(); });
/// p_host.addPanel("my_plugin", "My plugin",
///                 [this](robotik::Canvas const& p_canvas)
///                 { p_canvas.text("laps " + std::to_string(m_laps)); });
/// @endcode
// ****************************************************************************
class Host
{
public:

    /// Host without services.
    Host() = default;

    /// @param p_api C services of the host; must outlive this view.
    /// @param p_callbacks Where the menu and panel functions live, or null.
    Host(RobotIKHostAPI const* p_api, HostCallbacks* p_callbacks = nullptr)
        : m_api(p_api), m_callbacks(p_callbacks)
    {
    }

    /// @return True when connected to a host.
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
        if (valid() && m_api->log != nullptr)
        {
            std::string text(p_message);
            m_api->log(m_api->host, p_level, text.c_str());
        }
    }

    // -------------------------------------------------------------------------
    /// Reports an error to the host, shown to the user. A plugin step can
    /// also throw: the message reaches the user the same way.
    /// @param p_message What went wrong.
    // -------------------------------------------------------------------------
    void error(std::string_view p_message) const
    {
        if (valid() && m_api->report_error != nullptr)
        {
            std::string text(p_message);
            m_api->report_error(m_api->host, text.c_str());
        }
    }

    // -------------------------------------------------------------------------
    /// Asks for one kind of event; only those reach @ref Plugin::onEvent.
    /// @param p_kind Kind of event.
    // -------------------------------------------------------------------------
    void subscribe(RobotIKEventKind p_kind) const
    {
        if (valid() && m_api->subscribe != nullptr)
        {
            m_api->subscribe(m_api->host, static_cast<std::uint32_t>(p_kind));
        }
    }

    // -------------------------------------------------------------------------
    /// Adds a menu entry.
    /// @param p_menu Menu holding the entry.
    /// @param p_item Text of the entry.
    /// @param p_action Called when the user picks it; an exception is
    ///   reported to the user.
    /// @return Handle for @ref removeUi, or 0 on failure.
    // -------------------------------------------------------------------------
    std::uint32_t addMenuItem(std::string_view p_menu,
                              std::string_view p_item,
                              std::function<void()> p_action) const
    {
        if (!valid() || m_api->add_menu_item == nullptr || m_callbacks == nullptr)
        {
            return 0;
        }
        m_callbacks->actions.push_back({ std::move(p_action), m_api });
        std::string menu(p_menu);
        std::string item(p_item);
        return m_api->add_menu_item(m_api->host,
                                    menu.c_str(),
                                    item.c_str(),
                                    &runAction,
                                    &m_callbacks->actions.back());
    }

    // -------------------------------------------------------------------------
    /// Adds a panel, drawn every frame.
    /// @param p_id Stable key, to remember the layout.
    /// @param p_title Window title.
    /// @param p_draw Draws the panel on the canvas (see @ref Canvas).
    /// @return Handle for @ref removeUi, or 0 on failure.
    // -------------------------------------------------------------------------
    std::uint32_t addPanel(std::string_view p_id,
                           std::string_view p_title,
                           std::function<void(Canvas const&)> p_draw) const
    {
        if (!valid() || m_api->add_panel == nullptr || m_callbacks == nullptr)
        {
            return 0;
        }
        m_callbacks->panels.push_back({ std::move(p_draw), m_api });
        std::string id(p_id);
        std::string title(p_title);
        return m_api->add_panel(m_api->host,
                                id.c_str(),
                                title.c_str(),
                                &runPanel,
                                &m_callbacks->panels.back());
    }

    // -------------------------------------------------------------------------
    /// Removes a menu entry or a panel.
    /// @param p_handle Handle returned by @ref addMenuItem or @ref addPanel.
    // -------------------------------------------------------------------------
    void removeUi(std::uint32_t p_handle) const
    {
        if (valid() && m_api->remove_ui != nullptr)
        {
            m_api->remove_ui(m_api->host, p_handle);
        }
    }

    // -------------------------------------------------------------------------
    /// @return The scenario file being run, or empty.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string scenarioPath() const
    {
        if (!valid() || m_api->scenario_path == nullptr)
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
    /// Gives the host the scenario logic of the plugin; the host passes it to the
    /// simulation it builds. Call it in @ref Plugin::setup (a
    /// @ref ScenarioPlugin does it for you).
    /// @param p_logic ScenarioLogic owned by the plugin; must outlive the run.
    // -------------------------------------------------------------------------
    void bindLogic(ScenarioLogic& p_logic) const;

private:

    /// Reports a message, if the host can receive it.
    static void report(RobotIKHostAPI const* p_api, char const* p_message)
    {
        if (p_api != nullptr && p_api->report_error != nullptr &&
            p_api->host != nullptr)
        {
            p_api->report_error(p_api->host, p_message);
        }
    }

    /// C trampoline of a menu action; no exception crosses the border.
    static void runAction(void* p_slot)
    {
        auto const* call =
            static_cast<HostCallbacks::Slot<std::function<void()>> const*>(p_slot);
        try
        {
            call->function();
        }
        catch (std::exception const& error)
        {
            report(call->api, error.what());
        }
        catch (...)
        {
            report(call->api, "plugin menu action failed");
        }
    }

    /// C trampoline of a panel; no exception crosses the border.
    static void runPanel(void* p_slot, RobotIKCanvas const* p_canvas)
    {
        auto const* call = static_cast<
            HostCallbacks::Slot<std::function<void(Canvas const&)>> const*>(p_slot);
        try
        {
            call->function(Canvas(p_canvas));
        }
        catch (std::exception const& error)
        {
            report(call->api, error.what());
        }
        catch (...)
        {
            report(call->api, "plugin panel failed");
        }
    }

    /// C services of the host, or null.
    RobotIKHostAPI const* m_api = nullptr;
    /// Functions of the menus and panels, or null.
    HostCallbacks* m_callbacks = nullptr;
};

// ****************************************************************************
/// Base of a plugin instance. Override the steps the demo needs.
///
/// A plugin is a shared library that the hosts load when the user picks one
/// of its scenarios. This class is its C++ side: the macro
/// @c ROBOTIK_EXPORT_PLUGIN turns it into the C functions that the host
/// looks for, so nothing C++ crosses the border of the library.
///
/// @c describe is static because the host reads it before @c create. A step
/// that cannot go on throws: the message is reported to the user and the
/// host gets an error status (@c ROBOTIK_PLUGIN_ERR_SETUP from @ref setup,
/// @c ROBOTIK_PLUGIN_ERR_RUNTIME from the others).
///
/// The host calls: @ref setup, @ref start, then @ref update after each
/// simulation step (and @ref onEvent, @ref pause when they happen), then
/// @ref stop and @ref shutdown.
///
/// Most plugins only give a scenario logic per scenario: derive from
/// @ref ScenarioPlugin instead. A plugin keeps no world entity between calls
/// (find them by id), and does not decide if a run passes: the assertions of
/// the scenario do. Errors are exceptions rather than status codes, so the
/// code of a plugin has no error plumbing; the C border turns them into
/// codes.
///
/// @code
/// void MyPlugin::onEvent(robotik::Host&, RobotIKEvent const& p_event)
/// {
///     if (p_event.kind == ROBOTIK_EVENT_KEY_PRESSED && p_event.code == 'H')
///         home();
/// }
/// @endcode
// ****************************************************************************
class Plugin
{
public:

    virtual ~Plugin() = default;

    // -------------------------------------------------------------------------
    /// Prepares a scenario, before the simulation exists: create the scenario logic
    /// and bind it (@ref Host::bindLogic), add menus and panels.
    /// @param p_host Services of the host.
    /// @param p_scenario Scenario id, as listed in @c plugin.yaml.
    /// @throws std::exception for an unknown scenario.
    // -------------------------------------------------------------------------
    virtual void setup(Host& /*p_host*/, std::string_view /*p_scenario*/) {}

    /// The run starts.
    virtual void start(Host& /*p_host*/) {}

    /// Called after each simulation step of duration @p_dt.
    virtual void update(Host& /*p_host*/, Seconds /*p_dt*/) {}

    /// Receives an event subscribed with @ref Host::subscribe.
    virtual void onEvent(Host& /*p_host*/, RobotIKEvent const& /*p_event*/) {}

    /// The run is paused (@p_paused true) or resumed.
    virtual void pause(Host& /*p_host*/, bool /*p_paused*/) {}

    /// The run ended.
    virtual void stop(Host& /*p_host*/) {}

    /// The host destroyed the simulation: free the scenario logic and the rest.
    virtual void shutdown(Host& /*p_host*/) {}
};

/// Builds the scenario logic of one scenario.
using LogicFactory = std::function<std::unique_ptr<ScenarioLogic>()>;

// ----------------------------------------------------------------------------
/// @return A @ref LogicFactory building @c T from @p p_args (copied).
///
/// @code
/// { "painter_pddl", robotik::logic<PainterLogic>(true) }
/// @endcode
// ----------------------------------------------------------------------------
template <class T, class... Args>
[[nodiscard]] LogicFactory logic(Args... p_args)
{
    return [=]() -> std::unique_ptr<ScenarioLogic>
    { return std::make_unique<T>(p_args...); };
}

// ****************************************************************************
/// A plugin that gives one scenario logic per scenario: the common case.
///
/// A demo plugin mostly says "this scenario runs this scenario logic". The
/// table says it once; the base creates the scenario logic, binds it, refuses an
/// unknown scenario (the message lists the known ids) and frees the scenario logic
/// at the end.
///
/// @code
/// class PainterPlugin final: public robotik::ScenarioPlugin
/// {
/// public:
///     static robotik::PluginInfo describe() { return { .id = "robotik.painter" }; }
///     PainterPlugin()
///         : ScenarioPlugin({ { "painter", robotik::logic<PainterLogic>(false) },
///                           { "painter_pddl", robotik::logic<PainterLogic>(true) } })
///     {
///     }
/// };
/// ROBOTIK_EXPORT_PLUGIN(PainterPlugin)
/// @endcode
///
/// To add a menu or a panel, override @ref setup, call
/// @c ScenarioPlugin::setup first, then use the host.
///
/// The scenario logic lives from @ref setup to @ref shutdown: the host destroys the
/// simulation before @ref shutdown, so the scenario logic never outlives the cell
/// that points to it. @ref logic returns null when the current scenario
/// runs a scenario logic of another type. A plugin that chooses its scenario logic in
/// another way (from a parameter of the scenario, say) derives from
/// @ref Plugin instead. The table is a constructor argument, not a virtual
/// method, so that the ids of @c plugin.yaml appear once in the C++ code,
/// next to the class that runs them.
// ****************************************************************************
class ScenarioPlugin: public Plugin
{
public:

    /// @param p_logics Scenario id (as in @c plugin.yaml) and its scenario logic.
    ScenarioPlugin(
        std::initializer_list<std::pair<std::string_view, LogicFactory>>
            p_logics)
    {
        for (auto const& [id, factory] : p_logics)
        {
            m_factories.emplace_back(std::string(id), factory);
        }
    }

    // -------------------------------------------------------------------------
    /// Creates and binds the scenario logic of @p p_scenario.
    /// @throws std::invalid_argument for a scenario not in the table.
    // -------------------------------------------------------------------------
    void setup(Host& p_host, std::string_view p_scenario) override
    {
        m_logic.reset();
        m_scenario = std::string(p_scenario);
        for (auto const& [id, factory] : m_factories)
        {
            if (id == p_scenario)
            {
                m_logic = factory();
                p_host.bindLogic(*m_logic);
                return;
            }
        }
        std::string known;
        for (auto const& [id, factory] : m_factories)
        {
            known += (known.empty() ? "" : ", ") + id;
        }
        throw std::invalid_argument("unknown scenario '" + m_scenario +
                                    "' (this plugin runs: " + known + ")");
    }

    /// Frees the scenario logic.
    void shutdown(Host& /*p_host*/) override
    {
        m_logic.reset();
    }

protected:

    /// @return Id of the scenario being set up or run.
    [[nodiscard]] std::string const& scenario() const
    {
        return m_scenario;
    }

    /// @return The scenario logic of the scenario, as a @c T, or null.
    template <class T = ScenarioLogic>
    [[nodiscard]] T* logic() const
    {
        return dynamic_cast<T*>(m_logic.get());
    }

private:

    /// Scenario ids and their scenario logics.
    std::vector<std::pair<std::string, LogicFactory>> m_factories;
    /// Id of the current scenario.
    std::string m_scenario;
    /// ScenarioLogic of the current scenario, or null.
    std::unique_ptr<ScenarioLogic> m_logic;
};

/// Glue behind @ref ROBOTIK_EXPORT_PLUGIN. Not for plugin code.
namespace plugin_detail
{

// ----------------------------------------------------------------------------
/// What the host sees as a @c RobotIKPlugin: a copy of the host API, the
/// functions of the menus and panels, and the C++ plugin object.
/// @tparam T Plugin class.
// ----------------------------------------------------------------------------
template <class T>
struct Box
{
    RobotIKHostAPI api{};      ///< Copy of the services of the host.
    HostCallbacks callbacks{}; ///< Menu and panel functions.
    T object;                  ///< The plugin.
};

/// @return The box behind an opaque plugin pointer.
template <class T>
Box<T>* boxOf(RobotIKPlugin* p_plugin)
{
    return reinterpret_cast<Box<T>*>(p_plugin);
}

/// Reports @p_message to the host, if it can receive it.
inline void report(RobotIKHostAPI const* p_api, char const* p_message)
{
    if (p_api != nullptr && p_api->report_error != nullptr &&
        p_api->host != nullptr && p_message != nullptr)
    {
        p_api->report_error(p_api->host, p_message);
    }
}

// ----------------------------------------------------------------------------
/// Runs one step of the plugin: an exception becomes @p p_failure and a
/// message to the user; nothing C++ crosses the border.
/// @param p_plugin Opaque instance.
/// @param p_failure Status of a step that throws.
/// @param p_what Name of the step, for an exception without message.
/// @param p_step Calls the step with the object and its host view.
// ----------------------------------------------------------------------------
template <class T, class Step>
RobotIKPluginStatus guard(RobotIKPlugin* p_plugin,
                          RobotIKPluginStatus p_failure,
                          char const* p_what,
                          Step&& p_step)
{
    Box<T>* self = boxOf<T>(p_plugin);
    if (self == nullptr)
    {
        return ROBOTIK_PLUGIN_ERR_RUNTIME;
    }
    try
    {
        Host host(&self->api, &self->callbacks);
        p_step(self->object, host);
        return ROBOTIK_PLUGIN_OK;
    }
    catch (std::exception const& error)
    {
        report(&self->api, error.what());
    }
    catch (...)
    {
        report(&self->api, (std::string("plugin ") + p_what + " failed").c_str());
    }
    return p_failure;
}

// ----------------------------------------------------------------------------
/// Copies a string into a fixed C buffer, final zero included.
/// @return False when the text does not fit.
// ----------------------------------------------------------------------------
inline bool copyField(char* p_dest, std::size_t p_capacity, std::string_view p_value)
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

/// C entries calling the steps of @p T.
template <class T>
struct Thunks
{
    static RobotIKPluginStatus setup(RobotIKPlugin* p_plugin, char const* p_scenario)
    {
        std::string_view const scenario = p_scenario != nullptr ? p_scenario : "";
        return guard<T>(p_plugin, ROBOTIK_PLUGIN_ERR_SETUP, "setup",
                        [&](T& p_object, Host& p_host) { p_object.setup(p_host, scenario); });
    }
    static RobotIKPluginStatus start(RobotIKPlugin* p_plugin)
    {
        return guard<T>(p_plugin, ROBOTIK_PLUGIN_ERR_RUNTIME, "start",
                        [](T& p_object, Host& p_host) { p_object.start(p_host); });
    }
    static RobotIKPluginStatus update(RobotIKPlugin* p_plugin, double p_dt)
    {
        return guard<T>(p_plugin, ROBOTIK_PLUGIN_ERR_RUNTIME, "update",
                        [=](T& p_object, Host& p_host)
                        { p_object.update(p_host, Seconds(p_dt)); });
    }
    static RobotIKPluginStatus event(RobotIKPlugin* p_plugin, RobotIKEvent const* p_event)
    {
        if (p_event == nullptr)
        {
            return ROBOTIK_PLUGIN_ERR_RUNTIME;
        }
        return guard<T>(p_plugin, ROBOTIK_PLUGIN_ERR_RUNTIME, "event",
                        [=](T& p_object, Host& p_host) { p_object.onEvent(p_host, *p_event); });
    }
    static RobotIKPluginStatus pause(RobotIKPlugin* p_plugin, int p_paused)
    {
        return guard<T>(p_plugin, ROBOTIK_PLUGIN_ERR_RUNTIME, "pause",
                        [=](T& p_object, Host& p_host) { p_object.pause(p_host, p_paused != 0); });
    }
    static RobotIKPluginStatus stop(RobotIKPlugin* p_plugin)
    {
        return guard<T>(p_plugin, ROBOTIK_PLUGIN_ERR_RUNTIME, "stop",
                        [](T& p_object, Host& p_host) { p_object.stop(p_host); });
    }
    static RobotIKPluginStatus shutdown(RobotIKPlugin* p_plugin)
    {
        return guard<T>(p_plugin, ROBOTIK_PLUGIN_ERR_RUNTIME, "shutdown",
                        [](T& p_object, Host& p_host) { p_object.shutdown(p_host); });
    }
};

/// Stores a C++ function pointer in a C function pointer of the same size.
template <class CFunction, class CppFunction>
void assignFunction(CFunction* p_dest, CppFunction p_function)
{
    static_assert(sizeof(CFunction) == sizeof(CppFunction));
    std::memcpy(p_dest, &p_function, sizeof(p_function));
}

/// Fills the vtable with the thunks of @p T.
template <class T>
void fillTable(RobotIKPluginVTable* p_api)
{
    p_api->size = static_cast<std::uint32_t>(sizeof(RobotIKPluginVTable));
    p_api->abi_version = ROBOTIK_PLUGIN_ABI_VERSION;
    assignFunction(&p_api->setup, &Thunks<T>::setup);
    assignFunction(&p_api->start, &Thunks<T>::start);
    assignFunction(&p_api->update, &Thunks<T>::update);
    assignFunction(&p_api->on_event, &Thunks<T>::event);
    assignFunction(&p_api->pause, &Thunks<T>::pause);
    assignFunction(&p_api->stop, &Thunks<T>::stop);
    assignFunction(&p_api->shutdown, &Thunks<T>::shutdown);
}

// ----------------------------------------------------------------------------
/// Body of @c robotik_plugin_query: checks the sizes and ABI versions, then
/// copies @c T::describe() and the vtable.
/// @return False on a mismatch or a field too long.
// ----------------------------------------------------------------------------
template <class T>
bool queryPlugin(RobotIKPluginInfo* p_info, RobotIKPluginVTable* p_api)
{
    if (p_info == nullptr || p_api == nullptr)
    {
        return false;
    }
    if (p_info->size < sizeof(RobotIKPluginInfo) ||
        p_api->size < sizeof(RobotIKPluginVTable))
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
RobotIKPlugin* createPlugin(RobotIKHostAPI const* p_host)
{
    if (p_host == nullptr || p_host->size < sizeof(RobotIKHostAPI) ||
        p_host->abi_version != ROBOTIK_PLUGIN_ABI_VERSION ||
        p_host->host == nullptr)
    {
        return nullptr;
    }
    try
    {
        auto* plugin = new Box<T>();
        plugin->api = *p_host;
        return reinterpret_cast<RobotIKPlugin*>(plugin);
    }
    catch (...)
    {
        report(p_host, "plugin create failed");
        return nullptr;
    }
}

/// Body of @c robotik_plugin_destroy.
template <class T>
void destroyPlugin(RobotIKPlugin* p_plugin)
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
#if defined(ROBOTIK_PLUGIN_LIBRARY)
// The web build links the plugins into the page (plugins/plugin.mk defines
// ROBOTIK_PLUGIN_LIBRARY="librobotik_x.so"): the five functions are local
// to the plugin, and a static object adds them to the table of
// BuiltinPlugins.hpp under the name of its library.
#    define ROBOTIK_EXPORT_PLUGIN(Type)                                       \
        namespace                                                             \
        {                                                                     \
        uint32_t robotikPluginAbiVersion()                                    \
        {                                                                     \
            return ROBOTIK_PLUGIN_ABI_VERSION;                                \
        }                                                                     \
        char const* robotikPluginBuildId()                                    \
        {                                                                     \
            return ROBOTIK_PLUGIN_BUILD_ID;                                   \
        }                                                                     \
        bool robotikPluginQuery(RobotIKPluginInfo* info,                      \
                                RobotIKPluginVTable* api)                     \
        {                                                                     \
            return robotik::plugin_detail::queryPlugin<Type>(info, api);      \
        }                                                                     \
        RobotIKPlugin* robotikPluginCreate(RobotIKHostAPI const* host)        \
        {                                                                     \
            return robotik::plugin_detail::createPlugin<Type>(host);          \
        }                                                                     \
        void robotikPluginDestroy(RobotIKPlugin* plugin)                      \
        {                                                                     \
            robotik::plugin_detail::destroyPlugin<Type>(plugin);              \
        }                                                                     \
        template <class F>                                                    \
        robotik::BuiltinFunction robotikPluginErase(F p_function)             \
        {                                                                     \
            return reinterpret_cast<robotik::BuiltinFunction>(p_function);    \
        }                                                                     \
        [[maybe_unused]] bool const ROBOTIK_PLUGIN_REGISTERED =               \
            robotik::registerBuiltinPlugin(                                   \
                { ROBOTIK_PLUGIN_LIBRARY,                                     \
                  { { { "robotik_plugin_abi_version",                         \
                        robotikPluginErase(&robotikPluginAbiVersion) },       \
                      { "robotik_plugin_build_id",                            \
                        robotikPluginErase(&robotikPluginBuildId) },          \
                      { "robotik_plugin_query",                               \
                        robotikPluginErase(&robotikPluginQuery) },            \
                      { "robotik_plugin_create",                              \
                        robotikPluginErase(&robotikPluginCreate) },           \
                      { "robotik_plugin_destroy",                             \
                        robotikPluginErase(&robotikPluginDestroy) } } } });   \
        }
#else
#    define ROBOTIK_EXPORT_PLUGIN(Type)                                      \
    extern "C"                                                           \
    {                                                                    \
        uint32_t robotik_plugin_abi_version(void)                        \
        {                                                                \
            return ROBOTIK_PLUGIN_ABI_VERSION;                           \
        }                                                                \
        char const* robotik_plugin_build_id(void)                        \
        {                                                                \
            return ROBOTIK_PLUGIN_BUILD_ID;                              \
        }                                                                \
        bool robotik_plugin_query(RobotIKPluginInfo* info,               \
                                  RobotIKPluginVTable* api)              \
        {                                                                \
            return robotik::plugin_detail::queryPlugin<Type>(info, api); \
        }                                                                \
        RobotIKPlugin* robotik_plugin_create(RobotIKHostAPI const* host) \
        {                                                                \
            return robotik::plugin_detail::createPlugin<Type>(host);     \
        }                                                                \
        void robotik_plugin_destroy(RobotIKPlugin* plugin)               \
        {                                                                \
            robotik::plugin_detail::destroyPlugin<Type>(plugin);         \
        }                                                                \
    }
#endif
