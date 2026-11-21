// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PluginBuildId.hpp
/// The text that says which build of RobotIK a plugin was made with.
///
/// The C table of PluginABI.h carries two C++ objects as @c void*: the
/// @c robotik::Simulation of the run and the @c robotik::ScenarioLogic of
/// the plugin. Both sides read them as C++ classes, so the plugin and the
/// host must agree on their layout: same RobotIK version, same compiler,
/// same standard library, same debug containers. The plugin exports this
/// text (@c robotik_plugin_build_id, see @c ROBOTIK_EXPORT_PLUGIN); the host
/// refuses a plugin whose text is not its own, before it calls anything.
///
/// @code
/// // Host side (PluginManager.cpp):
/// if (std::strcmp(build_id(), ROBOTIK_PLUGIN_BUILD_ID) != 0) { refuse(); }
/// @endcode
///
/// @par Contract
/// - Promises: two builds with the same RobotIK version, compiler version,
///   standard library release and _GLIBCXX_DEBUG choice give the same text.
/// - Limits: it does not see a change of compile options that keeps these
///   (another -std, packing pragmas): build the plugins with the SDK build
///   of the host, as the Makefiles of the repository do.
#pragma once

#include "Robotik/Package/Plugin/PluginABI.h"
#include "robotik-core/project_info.hpp"

/// Text of a macro value.
#define ROBOTIK_BUILD_ID_TEXT(p_value) ROBOTIK_BUILD_ID_TEXT2(p_value)
/// Helper of ROBOTIK_BUILD_ID_TEXT.
#define ROBOTIK_BUILD_ID_TEXT2(p_value) #p_value

#if defined(__GLIBCXX__)
#    define ROBOTIK_BUILD_ID_STDLIB \
        " libstdc++ " ROBOTIK_BUILD_ID_TEXT(__GLIBCXX__)
#elif defined(_LIBCPP_VERSION)
#    define ROBOTIK_BUILD_ID_STDLIB \
        " libc++ " ROBOTIK_BUILD_ID_TEXT(_LIBCPP_VERSION)
#else
#    define ROBOTIK_BUILD_ID_STDLIB " stdlib ?"
#endif

#if defined(_GLIBCXX_DEBUG)
#    define ROBOTIK_BUILD_ID_DEBUG " debug-containers"
#else
#    define ROBOTIK_BUILD_ID_DEBUG ""
#endif

/// The build of RobotIK this code is compiled with.
#define ROBOTIK_PLUGIN_BUILD_ID                                    \
    "robotik " ROBOTIK_CORE_VERSION " abi " ROBOTIK_BUILD_ID_TEXT( \
        ROBOTIK_PLUGIN_ABI_VERSION) " compiler " __VERSION__       \
        ROBOTIK_BUILD_ID_STDLIB ROBOTIK_BUILD_ID_DEBUG
