// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file BuiltinPlugins.hpp
/// Plugins linked into the program instead of loaded from a @c .so file.
///
/// The web page (Emscripten) cannot open a shared library: its plugins are
/// compiled into the page. Each one adds its five C functions to this table
/// when the program starts (@ref ROBOTIK_EXPORT_PLUGIN does it when
/// @c ROBOTIK_PLUGIN_LIBRARY is defined, see @c plugins/plugin.mk), under
/// the file name of its library (@c library.entrypoint of @c plugin.yaml).
/// @ref PluginManager::load looks in this table first, then calls @c dlopen.
///
/// @par Contract
/// - Promises: @ref findBuiltinPlugin gives the plugin registered under this
///   file name, or null. The table is built before @c main and never
///   changes after.
/// - Expects: one registration per library name; a second one is refused
///   (@ref registerBuiltinPlugin returns false).
#pragma once

#include <array>
#include <string_view>

namespace robotik
{

/// A function of the plugin ABI, stored without its type (cast back by
/// @ref PluginManager, which knows the type of each name).
using BuiltinFunction = void (*)();

// ****************************************************************************
/// One function of the C ABI of a plugin: its symbol name and its address.
// ****************************************************************************
struct BuiltinSymbol
{
    char const* name = nullptr;         ///< As @c dlsym would ask it.
    BuiltinFunction function = nullptr; ///< Address, cast to its type.
};

// ****************************************************************************
/// A plugin compiled into the program.
///
/// @code
/// // What ROBOTIK_EXPORT_PLUGIN(HelloWorldPlugin) makes in the web build:
/// robotik::registerBuiltinPlugin({ "librobotik_hello_world.so",
///     { { { "robotik_plugin_abi_version", ... }, ... } } });
/// @endcode
// ****************************************************************************
struct BuiltinPlugin
{
    /// File name of the library it replaces: "librobotik_hello_world.so".
    std::string_view library;
    /// The five functions of @c PluginABI.h.
    std::array<BuiltinSymbol, 5> symbols;
};

// ----------------------------------------------------------------------------
/// Adds a plugin to the table. Called by a static object of the plugin,
/// before @c main.
/// @param p_plugin The plugin; its texts must live as long as the program.
/// @return False when a plugin of this library name is already there.
// ----------------------------------------------------------------------------
bool registerBuiltinPlugin(BuiltinPlugin const& p_plugin);

// ----------------------------------------------------------------------------
/// @param p_library File name of the library ("librobotik_fly.so"), without
///   its folder.
/// @return The plugin compiled under this name, or null.
// ----------------------------------------------------------------------------
[[nodiscard]] BuiltinPlugin const*
findBuiltinPlugin(std::string_view p_library);

// ----------------------------------------------------------------------------
/// @param p_plugin A plugin of the table.
/// @param p_name A symbol name of @c PluginABI.h.
/// @return Its function, or null when the plugin has no such symbol.
// ----------------------------------------------------------------------------
[[nodiscard]] BuiltinFunction builtinSymbol(BuiltinPlugin const& p_plugin,
                                            std::string_view p_name);

} // namespace robotik
