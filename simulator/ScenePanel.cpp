// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.
#include "Ui.hpp"

#include <imgui.h>

#include <algorithm>
#include <cstdint>

static constexpr char const* STOP = "Stop";

/// @brief Run controls drawn over the top-left corner of the 3D view.
static void runControls(App& p_app, ImVec2 p_at)
{
    ImGui::SetCursorScreenPos(ImVec2(p_at.x + 10.0f, p_at.y + 8.0f));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImDrawListSplitter layers;
    layers.Split(draw, 2);
    layers.SetCurrentChannel(draw, 1);
    ImGui::BeginGroup();

    bool const owns_clock = p_app.owns_clock;
    ImGui::PushStyleColor(ImGuiCol_Button,
                          p_app.playing ? ImVec4(0.55f, 0.38f, 0.10f, 1.0f)
                                        : ImVec4(0.16f, 0.45f, 0.22f, 1.0f));
    if (ImGui::Button(p_app.playing ? "Pause" : "Play", ImVec2(64.0f, 0.0f)))
    {
        p_app.playing = !p_app.playing;
    }
    ImGui::PopStyleColor();
    ImGui::SetItemTooltip("Space");
    ImGui::SameLine();
    ImGui::BeginDisabled(p_app.playing);
    if (ImGui::Button("Step"))
    {
        p_app.step_once = true;
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("One step of 10 ms  (.)");
    ImGui::SameLine(0.0f, 16.0f);
    if (ImGui::Button("Replay"))
    {
        p_app.reset(p_app.seed);
    }
    ImGui::SetItemTooltip("Start again with this seed and play  (Ctrl+R)");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(96.0f);
    ImGui::InputScalar("##seed", ImGuiDataType_U64, &p_app.seed);
    ImGui::SetItemTooltip(
        "Seed: object layout, sensor noise and random faults derive from it.\n"
        "Press Replay to use it.");
    ImGui::SameLine();
    if (ImGui::Button("Next seed"))
    {
        p_app.reset(p_app.seed + 1u);
    }
    ImGui::SetItemTooltip("Ctrl+N");
    ImGui::SameLine(0.0f, 16.0f);
    ImGui::SetNextItemWidth(120.0f);
    ImGui::SliderFloat("##speed", &p_app.speed, 0.1f, 4.0f, "speed x%.1f");
    if (owns_clock)
    {
        ImGui::SameLine();
        ImGui::TextDisabled("(the plugin drives the clock)");
    }
    ImGui::EndGroup();

    ImVec2 const min = ImGui::GetItemRectMin();
    ImVec2 const max = ImGui::GetItemRectMax();
    layers.SetCurrentChannel(draw, 0);
    draw->AddRectFilled(ImVec2(min.x - 6.0f, min.y - 5.0f),
                        ImVec2(max.x + 6.0f, max.y + 5.0f),
                        IM_COL32(20, 22, 27, 200),
                        6.0f);
    layers.Merge(draw);
}

/// Emergency-stop button, drawn over the view when a Stop skill exists.
static void stopButton(App& p_app, ImVec2 p_at, float p_width)
{
    if (p_app.simulation->skills().find(STOP) == robotik::NO_SKILL)
    {
        return;
    }
    bool const engaged = emergencyEngaged(p_app);
    char const* label = engaged ? "Release stop" : "EMERGENCY STOP";
    ImVec2 const size(ImGui::CalcTextSize(label).x + 28.0f,
                      ImGui::GetFrameHeight() + 8.0f);
    ImGui::SetCursorScreenPos(
        ImVec2(p_at.x + p_width - size.x - 10.0f, p_at.y + 6.0f));
    ImGui::PushStyleColor(ImGuiCol_Button,
                          engaged ? ImVec4(0.30f, 0.30f, 0.32f, 1.0f)
                                  : ImVec4(0.75f, 0.12f, 0.12f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                          engaged ? ImVec4(0.40f, 0.40f, 0.42f, 1.0f)
                                  : ImVec4(0.90f, 0.18f, 0.18f, 1.0f));
    if (ImGui::Button(label, size))
    {
        toggleEmergency(p_app);
    }
    ImGui::PopStyleColor(2);
    ImGui::SetItemTooltip("The Stop skill takes every actuator  (Ctrl+E)");
}

/// @brief Colored frame around the view: the state of the run at a glance.
static void runFrame(App const& p_app, ImVec2 p_at, ImVec2 p_size)
{
    ImVec4 color;
    if (!runColor(p_app, color))
    {
        return;
    }
    constexpr float THICKNESS = 3.0f;
    float const half = THICKNESS * 0.5f;
    ImGui::GetWindowDrawList()->AddRect(
        ImVec2(p_at.x + half, p_at.y + half),
        ImVec2(p_at.x + p_size.x - half, p_at.y + p_size.y - half),
        ImGui::GetColorU32(color),
        0.0f,
        0,
        THICKNESS);
}

bool runColor(App const& p_app, ImVec4& p_color)
{
    if (!p_app.simulation)
    {
        return false;
    }
    robotik::Simulation const& simulation = *p_app.simulation;
    switch (simulation.outcome())
    {
        case robotik::Simulation::Outcome::Running:
            if (simulation.time().value() <= 0.0)
            {
                return false;
            }
            p_color = ORANGE;
            return true;
        case robotik::Simulation::Outcome::ControllerSucceeded:
        case robotik::Simulation::Outcome::StopCondition:
            p_color = simulation.passed() ? GREEN : RED;
            return true;
        case robotik::Simulation::Outcome::ControllerFailed:
        case robotik::Simulation::Outcome::Timeout:
        case robotik::Simulation::Outcome::AssertionFailed:
            p_color = RED;
            return true;
        case robotik::Simulation::Outcome::Interrupted:
            return false;
    }
    return false;
}

bool emergencyEngaged(App const& p_app)
{
    if (!p_app.simulation)
    {
        return false;
    }
    robotik::SkillScheduler const& skills = p_app.simulation->skills();
    robotik::SkillId const stop = skills.find(STOP);
    if (stop == robotik::NO_SKILL)
    {
        return false;
    }
    robotik::SkillState const state = skills.state(stop);
    return state == robotik::SkillState::Running ||
           state == robotik::SkillState::Waiting;
}

void toggleEmergency(App& p_app)
{
    if (!p_app.simulation)
    {
        return;
    }
    robotik::SkillScheduler& skills = p_app.simulation->skills();
    robotik::SkillId const stop = skills.find(STOP);
    if (stop == robotik::NO_SKILL)
    {
        return;
    }
    if (emergencyEngaged(p_app))
    {
        skills.cancel(stop);
    }
    else
    {
        skills.request(stop);
    }
}

void scenePanel(App& p_app, char const* p_key)
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    std::string const title = panelTitle("Scene", p_key);
    bool const visible = ImGui::Begin(title.c_str(),
                                      nullptr,
                                      ImGuiWindowFlags_NoScrollbar |
                                          ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    p_app.view_hovered = false;
    if (!visible)
    {
        ImGui::End();
        return;
    }
    if (!p_app.simulation)
    {
        emptyState(p_app.error.empty()
                       ? "No scenario loaded.\nChoose one in the Demos menu, "
                         "or File > Open scenario."
                       : p_app.error.c_str());
        ImGui::End();
        return;
    }
    ImVec2 const size(std::max(ImGui::GetContentRegionAvail().x, 1.0f),
                      std::max(ImGui::GetContentRegionAvail().y, 1.0f));
    ImVec2 const scale = ImGui::GetIO().DisplayFramebufferScale;
    p_app.view.resize(static_cast<std::uint32_t>(size.x * scale.x),
                      static_cast<std::uint32_t>(size.y * scale.y));
    ImVec2 const at = ImGui::GetCursorScreenPos();
    p_app.view_x = at.x;
    p_app.view_y = at.y;
    p_app.view_w = size.x;
    p_app.view_h = size.y;
    // The controls drawn on top must win the mouse over the view.
    ImGui::SetNextItemAllowOverlap();
    ImGui::InvisibleButton("##view",
                           size,
                           ImGuiButtonFlags_MouseButtonLeft |
                               ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    p_app.view_hovered = ImGui::IsItemHovered() || ImGui::IsItemActive();
    if (p_app.view.width > 0)
    {
        // The device puts the first row at the bottom.
        ImGui::GetWindowDrawList()->AddImage(
            ImTextureRef(static_cast<ImTextureID>(p_app.view.color.nativeId())),
            at,
            ImVec2(at.x + size.x, at.y + size.y),
            ImVec2(0.0f, 1.0f),
            ImVec2(1.0f, 0.0f));
    }
    runFrame(p_app, at, size);
    runControls(p_app, at);
    stopButton(p_app, at, size.x);
    char const* hint =
        "Left drag: move   Right drag: turn   Wheel: zoom   WASD QE RF";
    ImVec2 const text = ImGui::CalcTextSize(hint);
    ImVec2 const corner(at.x + 10.0f, at.y + size.y - text.y - 12.0f);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(
        corner,
        ImVec2(corner.x + text.x + 12.0f, corner.y + text.y + 6.0f),
        IM_COL32(20, 22, 27, 170),
        4.0f);
    draw->AddText(ImVec2(corner.x + 6.0f, corner.y + 3.0f),
                  ImGui::GetColorU32(ImGuiCol_TextDisabled),
                  hint);
    ImGui::End();
}
