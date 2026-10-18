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
#include <vector>

/// Canvas callback: one line of text.
static void canvasText(RobotikCanvas const* /*p_canvas*/, char const* p_line)
{
    ImGui::TextUnformatted(p_line != nullptr ? p_line : "");
}

/// Canvas callback: one line of colored text.
static void canvasColor(RobotikCanvas const* /*p_canvas*/,
                 float p_red,
                 float p_green,
                 float p_blue,
                 char const* p_line)
{
    ImGui::TextColored(ImVec4(p_red, p_green, p_blue, 1.0f),
                       "%s",
                       p_line != nullptr ? p_line : "");
}

/// Canvas callback: a button, 1 when it was clicked.
static int canvasButton(RobotikCanvas const* /*p_canvas*/, char const* p_label)
{
    return ImGui::Button(p_label != nullptr ? p_label : "") ? 1 : 0;
}

/// Canvas callback: a checkbox bound to an int.
static int canvasCheckbox(RobotikCanvas const* /*p_canvas*/, char const* p_label, int* p_value)
{
    if (p_value == nullptr)
    {
        return 0;
    }
    bool flag = *p_value != 0;
    bool const changed = ImGui::Checkbox(p_label != nullptr ? p_label : "", &flag);
    *p_value = flag ? 1 : 0;
    return changed ? 1 : 0;
}

/// Canvas callback: an integer slider.
static int canvasSlider(RobotikCanvas const* /*p_canvas*/,
                 char const* p_label,
                 int* p_value,
                 int p_min,
                 int p_max)
{
    if (p_value == nullptr)
    {
        return 0;
    }
    return ImGui::SliderInt(p_label != nullptr ? p_label : "", p_value, p_min, p_max) ? 1 : 0;
}

/// Canvas callback: a radio button.
static int canvasRadio(RobotikCanvas const* /*p_canvas*/, char const* p_label, int p_active)
{
    return ImGui::RadioButton(p_label != nullptr ? p_label : "", p_active != 0) ? 1 : 0;
}

/// Canvas callback: a separator.
static void canvasSeparator(RobotikCanvas const* /*p_canvas*/)
{
    ImGui::Separator();
}

/// Fills the C canvas with the ImGui callbacks.
static RobotikCanvas makeCanvas()
{
    RobotikCanvas canvas{};
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

void drawDemosMenu(App& p_app)
{
    if (!ImGui::BeginMenu("Demos"))
    {
        return;
    }
    if (p_app.plugins.catalog().packages().empty())
    {
        ImGui::TextDisabled("No plugin package found in build/plugins");
    }
    for (robotik::PluginPackage const& package : p_app.plugins.catalog().packages())
    {
        if (!ImGui::BeginMenu(package.name.c_str()))
        {
            continue;
        }
        // Entries with a menu are grouped in a submenu, in order of first use.
        std::vector<std::string> submenus;
        for (robotik::PluginScenario const& scenario : package.scenarios)
        {
            if (!scenario.menu.empty() &&
                std::find(submenus.begin(), submenus.end(), scenario.menu) == submenus.end())
            {
                submenus.push_back(scenario.menu);
            }
        }
        auto items = [&](std::string const& p_menu) {
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
    ImGui::EndMenu();
}

void drawPluginMenus(App& p_app)
{
    std::vector<std::string> menus;
    for (robotik::PluginMenuItem const& item : p_app.plugins.host().menus())
    {
        if (std::find(menus.begin(), menus.end(), item.menu) == menus.end())
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
            if (item.menu == menu && ImGui::MenuItem(item.label.c_str()) && item.callback != nullptr)
            {
                item.callback(item.user);
            }
        }
        ImGui::EndMenu();
    }
}

void drawPluginPanels(App& p_app, ImGuiID p_dock, bool p_focus)
{
    bool first = true;
    for (robotik::PluginPanel const& panel : p_app.plugins.host().panels())
    {
        if (p_dock != 0)
        {
            ImGui::DockBuilderDockWindow(panel.title.c_str(), p_dock);
        }
        if (p_focus && first)
        {
            ImGui::SetNextWindowFocus();
        }
        first = false;
        if (!ImGui::Begin(panel.title.c_str()))
        {
            ImGui::End();
            continue;
        }
        if (panel.draw != nullptr)
        {
            RobotikCanvas const canvas = makeCanvas();
            panel.draw(panel.user, &canvas);
        }
        ImGui::End();
    }
}
