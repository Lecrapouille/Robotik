// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PluginABI.h
/// C boundary of a Robotik demo plugin.
///
/// One shared library implements a family of demonstrations. Each execution
/// is a scenario file shipped beside that library. The host selects a
/// scenario id and passes it to @c setup. No C++ type, STL container or
/// exception crosses this boundary.
///
/// Every structure starts with its @c size: a newer host or plugin appends
/// fields at the end, and the other side checks the size before reading
/// them. Plugins do not use this file directly: @c PluginAPI.hpp wraps it.
///
/// @code
/// // Host side, simplified:
/// void* lib = dlopen("librobotik_hello_world.so", RTLD_NOW);
/// auto abi = (uint32_t (*)(void))dlsym(lib, "robotik_plugin_abi_version");
/// if (abi() != ROBOTIK_PLUGIN_ABI_VERSION) { ... }
/// RobotikPlugin* plugin = robotik_plugin_create(&host_api);
/// vtable.setup(plugin, "wave");
/// vtable.start(plugin);
/// @endcode

#ifndef ROBOTIK_PLUGIN_ABI_H
#define ROBOTIK_PLUGIN_ABI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/// Version of this boundary; host and plugin must agree.
#define ROBOTIK_PLUGIN_ABI_VERSION 1u

/// Size of @ref RobotikPluginInfo::id, final zero included.
#define ROBOTIK_PLUGIN_ID_CAP 64u
/// Size of @ref RobotikPluginInfo::name.
#define ROBOTIK_PLUGIN_NAME_CAP 96u
/// Size of @ref RobotikPluginInfo::version.
#define ROBOTIK_PLUGIN_VERSION_CAP 32u
/// Size of @ref RobotikPluginInfo::category.
#define ROBOTIK_PLUGIN_CATEGORY_CAP 48u
/// Size of @ref RobotikEvent::name.
#define ROBOTIK_PLUGIN_EVENT_NAME_CAP 96u

/// Log level: information.
#define ROBOTIK_LOG_INFO 0
/// Log level: something looks wrong.
#define ROBOTIK_LOG_WARNING 1
/// Log level: something failed.
#define ROBOTIK_LOG_ERROR 2

    /// Instance of a plugin; opaque to the host.
    typedef struct RobotikPlugin RobotikPlugin;
    /// Host object; opaque to the plugin.
    typedef struct RobotikHost RobotikHost;
    typedef struct RobotikCanvas RobotikCanvas;

    /// Result of every call across the boundary.
    typedef enum RobotikPluginStatus
    {
        ROBOTIK_PLUGIN_OK = 0,        ///< Done.
        ROBOTIK_PLUGIN_ERR_ABI = 1,   ///< Host and plugin ABI versions differ.
        ROBOTIK_PLUGIN_ERR_STATE = 2, ///< Call not allowed in this state.
        ROBOTIK_PLUGIN_ERR_SETUP = 3, ///< The scenario could not be prepared.
        ROBOTIK_PLUGIN_ERR_RUNTIME = 4, ///< The plugin failed while running.
        ROBOTIK_PLUGIN_ERR_NOT_FOUND =
            5, ///< Unknown library, symbol or scenario.
    } RobotikPluginStatus;

    /// What happened, in @ref RobotikEvent::kind.
    typedef enum RobotikEventKind
    {
        ROBOTIK_EVENT_NONE = 0,             ///< No event.
        ROBOTIK_EVENT_KEY_PRESSED = 1,      ///< A key; its code in @c code.
        ROBOTIK_EVENT_SCENARIO_STARTED = 2, ///< The run started.
        ROBOTIK_EVENT_SCENARIO_STOPPED = 3, ///< The run ended.
        ROBOTIK_EVENT_SKILL_COMPLETED =
            4, ///< A skill succeeded; its name in @c name.
        ROBOTIK_EVENT_SKILL_FAILED =
            5, ///< A skill failed; its name in @c name.
        ROBOTIK_EVENT_ASSERTION_CHANGED =
            6, ///< An assertion changed; @c name and @c flag.
    } RobotikEventKind;

    /// Who the plugin is, filled by @ref robotik_plugin_query.
    typedef struct RobotikPluginInfo
    {
        uint32_t size;                  ///< sizeof, set by the host.
        uint32_t abi_version;           ///< ABI version, set by the host.
        char id[ROBOTIK_PLUGIN_ID_CAP]; ///< Plugin id, as in @c plugin.yaml.
        char name[ROBOTIK_PLUGIN_NAME_CAP];         ///< Name shown to the user.
        char version[ROBOTIK_PLUGIN_VERSION_CAP];   ///< Plugin version text.
        char category[ROBOTIK_PLUGIN_CATEGORY_CAP]; ///< Group in the plugin
                                                    ///< list.
    } RobotikPluginInfo;

    /// One event sent by the host to the plugin (see @c subscribe).
    typedef struct RobotikEvent
    {
        uint32_t size; ///< sizeof the structure.
        uint32_t kind; ///< A @ref RobotikEventKind.
        int32_t code;  ///< Key code of a key event.
        double number; ///< Spare value; 0 for the current events.
        char name[ROBOTIK_PLUGIN_EVENT_NAME_CAP]; ///< Skill or assertion name.
        /// Assertion result, 1 when the check passes.
        int32_t flag;
    } RobotikEvent;

    /// Small widget toolkit the host gives to a panel draw function. The plugin
    /// draws without linking to a UI library.
    typedef struct RobotikCanvas
    {
        uint32_t size; ///< sizeof the structure, to check the appended fields.
        void* impl;    ///< Host data; do not touch.
        /// Writes one line of text.
        void (*text)(RobotikCanvas const* canvas, char const* line);
        /// Writes one line of text in a color (each channel in [0, 1]).
        void (*text_colored)(RobotikCanvas const* canvas,
                             float red,
                             float green,
                             float blue,
                             char const* line);
        /// Draws a button; returns 1 when clicked.
        int (*button)(RobotikCanvas const* canvas, char const* label);
        /// Draws a check box bound to @c *value; returns 1 when changed.
        int (*checkbox)(RobotikCanvas const* canvas,
                        char const* label,
                        int* value);
        /// Draws a slider bound to @c *value; returns 1 when changed.
        int (*slider_int)(RobotikCanvas const* canvas,
                          char const* label,
                          int* value,
                          int min_value,
                          int max_value);
        /// Draws a horizontal line.
        void (*separator)(RobotikCanvas const* canvas);
        /* Appended: check size before use. Returns 1 when clicked. */
        int (*radio)(RobotikCanvas const* canvas,
                     char const* label,
                     int active);
    } RobotikCanvas;

    /// Life cycle functions of a plugin, filled by @ref robotik_plugin_query.
    typedef struct RobotikPluginVTable
    {
        uint32_t size;        ///< sizeof, set by the host.
        uint32_t abi_version; ///< ABI version, set by the host.
        /// Prepares the scenario @c scenario_id (an id of @c plugin.yaml).
        RobotikPluginStatus (*setup)(RobotikPlugin* plugin,
                                     char const* scenario_id);
        /// Starts the prepared scenario.
        RobotikPluginStatus (*start)(RobotikPlugin* plugin);
        /// Advances by @c dt seconds.
        RobotikPluginStatus (*update)(RobotikPlugin* plugin, double dt);
        /// Receives an event the plugin subscribed to.
        RobotikPluginStatus (*on_event)(RobotikPlugin* plugin,
                                        RobotikEvent const* event);
        /// Pauses (1) or resumes (0).
        RobotikPluginStatus (*pause)(RobotikPlugin* plugin, int paused);
        /// Ends the run.
        RobotikPluginStatus (*stop)(RobotikPlugin* plugin);
        /// Releases the resources before destruction.
        RobotikPluginStatus (*shutdown)(RobotikPlugin* plugin);
    } RobotikPluginVTable;

    /// Writes a message in the host log (@c ROBOTIK_LOG_* level).
    typedef void (*RobotikLogFn)(RobotikHost* host,
                                 int level,
                                 char const* message);
    /// Reports an error to the host (shown to the user).
    typedef void (*RobotikErrorFn)(RobotikHost* host, char const* message);
    /// Asks for the events of a @ref RobotikEventKind; returns 1 on success.
    typedef int (*RobotikSubscribeFn)(RobotikHost* host, uint32_t kind);
    /// Adds a menu entry calling @c callback(user); returns its handle.
    typedef uint32_t (*RobotikAddMenuFn)(RobotikHost* host,
                                         char const* menu,
                                         char const* item,
                                         void (*callback)(void* user),
                                         void* user);
    /// Adds a panel drawn by @c draw(user, canvas) every frame; returns its
    /// handle.
    typedef uint32_t (*RobotikAddPanelFn)(
        RobotikHost* host,
        char const* id,
        char const* title,
        void (*draw)(void* user, RobotikCanvas const* canvas),
        void* user);
    /// Removes a menu entry or a panel by its handle.
    typedef void (*RobotikRemoveUiFn)(RobotikHost* host, uint32_t handle);
    /// Path of the scenario file being run.
    typedef char const* (*RobotikScenarioPathFn)(RobotikHost* host);
    /// The @c robotik::Simulation of the run, as an opaque pointer (or null).
    typedef void* (*RobotikSimulationFn)(RobotikHost* host);
    /// Gives the host the @c robotik::Mission of the plugin.
    typedef void (*RobotikBindMissionFn)(RobotikHost* host, void* mission);

    /// Services of the host, given to @ref robotik_plugin_create.
    typedef struct RobotikHostAPI
    {
        uint32_t size;                  ///< sizeof the structure.
        uint32_t abi_version;           ///< ABI version of the host.
        RobotikHost* host;              ///< First argument of every function.
        RobotikLogFn log;               ///< Writes in the log.
        RobotikErrorFn report_error;    ///< Reports an error.
        RobotikSubscribeFn subscribe;   ///< Asks for events.
        RobotikAddMenuFn add_menu_item; ///< Adds a menu entry.
        RobotikAddPanelFn add_panel;    ///< Adds a panel.
        RobotikRemoveUiFn remove_ui;    ///< Removes an entry or a panel.
        RobotikScenarioPathFn scenario_path; ///< Gives the scenario file.
        RobotikSimulationFn simulation;      ///< Gives the simulation.
        RobotikBindMissionFn bind_mission;   ///< Binds the mission.
    } RobotikHostAPI;

    /// ABI version implemented by the library. Compared before @c query.
    /// @return @ref ROBOTIK_PLUGIN_ABI_VERSION of the library build.
    uint32_t robotik_plugin_abi_version(void);

    /// Describes the plugin. Both structures arrive with their size and
    /// abi_version set by the host.
    /// @param info Receives the id, name, version and category.
    /// @param api Receives the life cycle functions.
    /// @return False when the plugin cannot fill them (ABI mismatch).
    bool robotik_plugin_query(RobotikPluginInfo* info,
                              RobotikPluginVTable* api);

    /// Creates one instance.
    /// @param host Services of the host; the host pointer inside must outlive
    ///   the instance.
    /// @return The instance, or null on failure.
    RobotikPlugin* robotik_plugin_create(RobotikHostAPI const* host);

    /// Destroys an instance created by @ref robotik_plugin_create.
    /// @param plugin Instance to destroy.
    void robotik_plugin_destroy(RobotikPlugin* plugin);

#ifdef __cplusplus
}
#endif

#endif
