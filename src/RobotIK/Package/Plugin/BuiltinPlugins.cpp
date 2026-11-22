// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Package/Plugin/BuiltinPlugins.hpp"

#include <vector>

namespace robotik
{

/// The table. A function-local static: the plugins register from their own
/// static objects, whose order against the ones of this file is not known.
static std::vector<BuiltinPlugin>& table()
{
    static std::vector<BuiltinPlugin> plugins;
    return plugins;
}

bool registerBuiltinPlugin(BuiltinPlugin const& p_plugin)
{
    if (findBuiltinPlugin(p_plugin.library) != nullptr)
    {
        return false;
    }
    table().push_back(p_plugin);
    return true;
}

BuiltinPlugin const* findBuiltinPlugin(std::string_view p_library)
{
    for (BuiltinPlugin const& plugin : table())
    {
        if (plugin.library == p_library)
        {
            return &plugin;
        }
    }
    return nullptr;
}

BuiltinFunction builtinSymbol(BuiltinPlugin const& p_plugin,
                              std::string_view p_name)
{
    for (BuiltinSymbol const& symbol : p_plugin.symbols)
    {
        if (symbol.name != nullptr && p_name == symbol.name)
        {
            return symbol.function;
        }
    }
    return nullptr;
}

} // namespace robotik
