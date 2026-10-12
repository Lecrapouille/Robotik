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

namespace
{

void canvasText(RobotikCanvas const* /*p_canvas*/, char const* p_line)
{
    ImGui::TextUnformatted(p_line != nullptr ? p_line : "");
}

void canvasColor(RobotikCanvas const* /*p_canvas*/,
                 float p_red,
                 float p_green,
                 float p_blue,
                 char const* p_line)
{
    ImGui::TextColored(ImVec4(p_red, p_green, p_blue, 1.0f),
                       "%s",
                       p_line != nullptr ? p_line : "");
}

int canvasButton(RobotikCanvas const* /*p_canvas*/, char const* p_label)
{
    return ImGui::Button(p_label != nullptr ? p_label : "") ? 1 : 0;
}

int canvasCheckbox(RobotikCanvas const* /*p_canvas*/, char const* p_label, int* p_value)
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

int canvasSlider(RobotikCanvas const* /*p_canvas*/,
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

int canvasRadio(RobotikCanvas const* /*p_canvas*/, char const* p_label, int p_active)
{
    return ImGui::RadioButton(p_label != nullptr ? p_label : "", p_active != 0) ? 1 : 0;
}

void canvasSeparator(RobotikCanvas const* /*p_canvas*/)
{
    ImGui::Separator();
}

RobotikCanvas makeCanvas()
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

} // namespace

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
        for (robotik::PluginScenario const& scenario : package.scenarios)
        {
            bool const current = scenario.file == p_app.scenario_path;
            if (ImGui::MenuItem(scenario.name.c_str(), nullptr, current))
            {
                requestScenario(p_app, scenario.file);
            }
        }
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
