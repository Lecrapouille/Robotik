// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Ui.hpp"

#include <imgui_internal.h>

#include <imgui.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

/// Canvas callback: one line of text.
static void canvasText(RobotIKCanvas const* /*p_canvas*/, char const* p_line)
{
    ImGui::TextUnformatted(p_line != nullptr ? p_line : "");
}

/// Canvas callback: one line of colored text.
static void canvasColor(RobotIKCanvas const* /*p_canvas*/,
                        float p_red,
                        float p_green,
                        float p_blue,
                        char const* p_line)
{
    ImGui::TextColored(ImVec4(p_red, p_green, p_blue, 1.0F),
                       "%s",
                       p_line != nullptr ? p_line : "");
}

/// Canvas callback: a button, 1 when it was clicked.
static int canvasButton(RobotIKCanvas const* /*p_canvas*/, char const* p_label)
{
    return ImGui::Button(p_label != nullptr ? p_label : "") ? 1 : 0;
}

/// Canvas callback: a checkbox bound to an int.
static int canvasCheckbox(RobotIKCanvas const* /*p_canvas*/,
                          char const* p_label,
                          int* p_value)
{
    if (p_value == nullptr)
    {
        return 0;
    }
    bool flag = *p_value != 0;
    bool const changed =
        ImGui::Checkbox(p_label != nullptr ? p_label : "", &flag);
    *p_value = flag ? 1 : 0;
    return changed ? 1 : 0;
}

/// Canvas callback: an integer slider.
static int canvasSlider(RobotIKCanvas const* /*p_canvas*/,
                        char const* p_label,
                        int* p_value,
                        int p_min,
                        int p_max)
{
    if (p_value == nullptr)
    {
        return 0;
    }
    return ImGui::SliderInt(
               p_label != nullptr ? p_label : "", p_value, p_min, p_max)
               ? 1
               : 0;
}

/// Canvas callback: a radio button.
static int canvasRadio(RobotIKCanvas const* /*p_canvas*/,
                       char const* p_label,
                       int p_active)
{
    return ImGui::RadioButton(p_label != nullptr ? p_label : "", p_active != 0)
               ? 1
               : 0;
}

/// Canvas callback: a separator.
static void canvasSeparator(RobotIKCanvas const* /*p_canvas*/)
{
    ImGui::Separator();
}

/// Fills the C canvas with the ImGui callbacks.
static RobotIKCanvas makeCanvas()
{
    RobotIKCanvas canvas{};
    canvas.size = static_cast<std::uint32_t>(sizeof(canvas));
    canvas.text = &canvasText;
    canvas.text_colored = &canvasColor;
    canvas.button = &canvasButton;
    canvas.checkbox = &canvasCheckbox;
    canvas.slider_int = &canvasSlider;
    canvas.separator = &canvasSeparator;
    canvas.radio = &canvasRadio;
    return canvas;
}

void requestScenario(App& p_app, std::filesystem::path const& p_file)
{
    p_app.ui.pending_scenario = p_file;
}

/// Category of the packages shown in the Tutorials menu.
static constexpr std::string_view TUTORIAL = "Tutorial";

/// One menu of packages: the tutorials, or every other package (the demos).
static void drawPackagesMenu(App& p_app, char const* p_title, bool p_tutorials)
{
    if (!ImGui::BeginMenu(p_title))
    {
        return;
    }
    bool empty = true;
    for (robotik::PluginPackage const& package :
         p_app.plugins.catalog().packages())
    {
        if ((package.category == TUTORIAL) != p_tutorials)
        {
            continue;
        }
        empty = false;
        if (!ImGui::BeginMenu(package.name.c_str()))
        {
            continue;
        }
        // Entries with a menu are grouped in a submenu, in order of first use.
        std::vector<std::string> submenus;
        for (robotik::PluginScenario const& scenario : package.scenarios)
        {
            if (!scenario.menu.empty() &&
                std::ranges::find(submenus, scenario.menu) == submenus.end())
            {
                submenus.push_back(scenario.menu);
            }
        }
        auto items = [&](std::string const& p_menu)
        {
            for (robotik::PluginScenario const& scenario : package.scenarios)
            {
                bool const current = scenario.file == p_app.scenario_path;
                if (scenario.menu == p_menu &&
                    ImGui::MenuItem(scenario.name.c_str(), nullptr, current))
                {
                    requestScenario(p_app, scenario.file);
                }
            }
        };
        for (std::string const& submenu : submenus)
        {
            if (ImGui::BeginMenu(submenu.c_str()))
            {
                items(submenu);
                ImGui::EndMenu();
            }
        }
        items({});
        ImGui::EndMenu();
    }
    if (empty)
    {
        ImGui::TextDisabled("No plugin package found (ROBOTIK_PLUGINS, "
                            "lib/robotik/plugins)");
    }
    ImGui::EndMenu();
}

void drawDemosMenu(App& p_app)
{
    drawPackagesMenu(p_app, "Tutorials", true);
    drawPackagesMenu(p_app, "Demos", false);
}

void drawPluginMenus(App& p_app)
{
    std::vector<std::string> menus;
    for (robotik::PluginMenuItem const& item : p_app.plugins.host().menus())
    {
        if (std::ranges::find(menus, item.menu) == menus.end())
        {
            menus.push_back(item.menu);
        }
    }
    for (std::string const& menu : menus)
    {
        if (!ImGui::BeginMenu(menu.c_str()))
        {
            continue;
        }
        for (robotik::PluginMenuItem const& item : p_app.plugins.host().menus())
        {
            if (item.menu == menu && ImGui::MenuItem(item.label.c_str()) &&
                item.callback != nullptr)
            {
                item.callback(item.user);
            }
        }
        ImGui::EndMenu();
    }
}

std::string pluginEditorKey(App const& p_app,
                            robotik::PluginPanel const& p_panel)
{
    auto const* package =
        p_app.plugins.catalog().matchScenario(p_app.scenario_path).package;
    return (package == nullptr ? std::string() : package->id + "/") +
           p_panel.key;
}

std::string pluginWindowTitle(App const& p_app,
                              robotik::PluginPanel const& p_panel)
{
    return p_panel.title + "###plugin_" + pluginEditorKey(p_app, p_panel);
}

void drawPluginPanels(App& p_app, ImGuiID p_dock, bool p_focus)
{
    bool first = true;
    for (robotik::PluginPanel const& panel : p_app.plugins.host().panels())
    {
        std::string const title = pluginWindowTitle(p_app, panel);
        bool& open = p_app.ui.plugin_panels
                         .try_emplace(pluginEditorKey(p_app, panel), true)
                         .first->second;
        if (p_dock != 0)
            ImGui::DockBuilderDockWindow(title.c_str(), p_dock);
        if (!open || (!p_app.ui.maximized_window.empty() &&
                      p_app.ui.maximized_window != title))
            continue;
        if (p_focus && first)
            ImGui::SetNextWindowFocus();
        first = false;
        if (!beginEditor(p_app, title.c_str(), open))
        {
            ImGui::End();
            continue;
        }
        if (panel.draw != nullptr)
        {
            RobotIKCanvas const canvas = makeCanvas();
            panel.draw(panel.user, &canvas);
        }
        ImGui::End();
    }
}
