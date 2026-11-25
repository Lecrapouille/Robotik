// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// A plugin compiled into the test program, as the web page does with its
// plugins (demos/plugin.mk defines ROBOTIK_PLUGIN_LIBRARY there).
#define ROBOTIK_PLUGIN_LIBRARY "librobotik_builtin_test.so"

#include "main.hpp"

#include "RobotIK/Package/Plugin/BuiltinPlugins.hpp"
#include "RobotIK/Package/Plugin/PluginAPI.hpp"
#include "RobotIK/Package/Plugin/PluginManager.hpp"

/// The smallest plugin: an id, no scenario.
class BuiltinTestPlugin final: public robotik::Plugin
{
public:

    static robotik::PluginInfo describe()
    {
        return { .id = "robotik.builtin_test" };
    }
};

ROBOTIK_EXPORT_PLUGIN(BuiltinTestPlugin)

TEST(BuiltinPlugins, TheExportMacroRegistersThePluginUnderItsLibrary)
{
    robotik::BuiltinPlugin const* plugin =
        robotik::findBuiltinPlugin("librobotik_builtin_test.so");
    ASSERT_NE(plugin, nullptr);
    EXPECT_NE(robotik::builtinSymbol(*plugin, "robotik_plugin_query"), nullptr);
    EXPECT_EQ(robotik::builtinSymbol(*plugin, "robotik_plugin_other"), nullptr);
    EXPECT_EQ(robotik::findBuiltinPlugin("librobotik_absent.so"), nullptr);

    // A second plugin of the same library name is refused.
    EXPECT_FALSE(robotik::registerBuiltinPlugin(*plugin));
}

TEST(BuiltinPlugins, TheManagerLoadsABuiltinPluginWithoutAFile)
{
    // The folder does not exist: only the file name is read.
    robotik::PluginManager manager;
    ASSERT_EQ(manager.load("/nowhere/librobotik_builtin_test.so"),
              ROBOTIK_PLUGIN_OK)
        << manager.error();
    EXPECT_EQ(manager.state(), robotik::PluginState::Loaded);
    EXPECT_EQ(manager.id(), "robotik.builtin_test");
    EXPECT_EQ(manager.unload(), ROBOTIK_PLUGIN_OK);
    EXPECT_EQ(manager.state(), robotik::PluginState::Unloaded);
}
