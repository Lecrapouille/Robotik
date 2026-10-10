// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PluginABI.h
//! @brief C boundary of a Robotik demo plugin.
//!
//! One shared library implements a family of demonstrations. Each execution
//! is a scenario file shipped beside that library. The host selects a
//! scenario id and passes it to @c setup. No C++ type, STL container or
//! exception crosses this boundary.

#ifndef ROBOTIK_PLUGIN_ABI_H
#define ROBOTIK_PLUGIN_ABI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOTIK_PLUGIN_ABI_VERSION 1u

#define ROBOTIK_PLUGIN_ID_CAP 64u
#define ROBOTIK_PLUGIN_NAME_CAP 96u
#define ROBOTIK_PLUGIN_VERSION_CAP 32u
#define ROBOTIK_PLUGIN_CATEGORY_CAP 48u
#define ROBOTIK_PLUGIN_EVENT_NAME_CAP 96u

#define ROBOTIK_LOG_INFO 0
#define ROBOTIK_LOG_WARNING 1
#define ROBOTIK_LOG_ERROR 2

typedef struct RobotikPlugin RobotikPlugin;
typedef struct RobotikHost RobotikHost;
typedef struct RobotikCanvas RobotikCanvas;

typedef enum RobotikPluginStatus
{
    ROBOTIK_PLUGIN_OK = 0,
    ROBOTIK_PLUGIN_ERR_ABI = 1,
    ROBOTIK_PLUGIN_ERR_STATE = 2,
    ROBOTIK_PLUGIN_ERR_SETUP = 3,
    ROBOTIK_PLUGIN_ERR_RUNTIME = 4,
    ROBOTIK_PLUGIN_ERR_NOT_FOUND = 5,
} RobotikPluginStatus;

typedef enum RobotikEventKind
{
    ROBOTIK_EVENT_NONE = 0,
    ROBOTIK_EVENT_KEY_PRESSED = 1,
    ROBOTIK_EVENT_SCENARIO_STARTED = 2,
    ROBOTIK_EVENT_SCENARIO_STOPPED = 3,
    ROBOTIK_EVENT_SKILL_COMPLETED = 4,
    ROBOTIK_EVENT_SKILL_FAILED = 5,
    ROBOTIK_EVENT_ASSERTION_CHANGED = 6,
} RobotikEventKind;

typedef struct RobotikPluginInfo
{
    uint32_t size;
    uint32_t abi_version;
    char id[ROBOTIK_PLUGIN_ID_CAP];
    char name[ROBOTIK_PLUGIN_NAME_CAP];
    char version[ROBOTIK_PLUGIN_VERSION_CAP];
    char category[ROBOTIK_PLUGIN_CATEGORY_CAP];
} RobotikPluginInfo;

typedef struct RobotikEvent
{
    uint32_t size;
    uint32_t kind;
    int32_t code;
    double number;
    char name[ROBOTIK_PLUGIN_EVENT_NAME_CAP];
    //! @brief Assertion result, 1 when the check passes.
    int32_t flag;
} RobotikEvent;

typedef struct RobotikCanvas
{
    uint32_t size;
    void* impl;
    void (*text)(RobotikCanvas const* canvas, char const* line);
    void (*text_colored)(RobotikCanvas const* canvas,
                         float red,
                         float green,
                         float blue,
                         char const* line);
    int (*button)(RobotikCanvas const* canvas, char const* label);
    int (*checkbox)(RobotikCanvas const* canvas, char const* label, int* value);
    int (*slider_int)(RobotikCanvas const* canvas,
                      char const* label,
                      int* value,
                      int min_value,
                      int max_value);
    void (*separator)(RobotikCanvas const* canvas);
} RobotikCanvas;

typedef struct RobotikPluginVTable
{
    uint32_t size;
    uint32_t abi_version;
    RobotikPluginStatus (*setup)(RobotikPlugin* plugin, char const* scenario_id);
    RobotikPluginStatus (*start)(RobotikPlugin* plugin);
    RobotikPluginStatus (*update)(RobotikPlugin* plugin, double dt);
    RobotikPluginStatus (*on_event)(RobotikPlugin* plugin, RobotikEvent const* event);
    RobotikPluginStatus (*pause)(RobotikPlugin* plugin, int paused);
    RobotikPluginStatus (*stop)(RobotikPlugin* plugin);
    RobotikPluginStatus (*shutdown)(RobotikPlugin* plugin);
} RobotikPluginVTable;

typedef void (*RobotikLogFn)(RobotikHost* host, int level, char const* message);
typedef void (*RobotikErrorFn)(RobotikHost* host, char const* message);
typedef int (*RobotikSubscribeFn)(RobotikHost* host, uint32_t kind);
typedef uint32_t (*RobotikAddMenuFn)(RobotikHost* host,
                                     char const* menu,
                                     char const* item,
                                     void (*callback)(void* user),
                                     void* user);
typedef uint32_t (*RobotikAddPanelFn)(RobotikHost* host,
                                      char const* id,
                                      char const* title,
                                      void (*draw)(void* user, RobotikCanvas const* canvas),
                                      void* user);
typedef void (*RobotikRemoveUiFn)(RobotikHost* host, uint32_t handle);
typedef char const* (*RobotikScenarioPathFn)(RobotikHost* host);
typedef void* (*RobotikSimulationFn)(RobotikHost* host);
typedef void (*RobotikBindMissionFn)(RobotikHost* host, void* mission);

typedef struct RobotikHostAPI
{
    uint32_t size;
    uint32_t abi_version;
    RobotikHost* host;
    RobotikLogFn log;
    RobotikErrorFn report_error;
    RobotikSubscribeFn subscribe;
    RobotikAddMenuFn add_menu_item;
    RobotikAddPanelFn add_panel;
    RobotikRemoveUiFn remove_ui;
    RobotikScenarioPathFn scenario_path;
    RobotikSimulationFn simulation;
    RobotikBindMissionFn bind_mission;
} RobotikHostAPI;

//! @brief ABI version implemented by the library. Compared before @c query.
uint32_t robotik_plugin_abi_version(void);

//! @brief Fills @p info and @p api. Both structures arrive with their size
//! and abi_version set by the host.
bool robotik_plugin_query(RobotikPluginInfo* info, RobotikPluginVTable* api);

//! @brief Creates one instance. The host pointer inside @p host must outlive it.
RobotikPlugin* robotik_plugin_create(RobotikHostAPI const* host);

//! @brief Destroys an instance created by @ref robotik_plugin_create.
void robotik_plugin_destroy(RobotikPlugin* plugin);

#ifdef __cplusplus
}
#endif

#endif
