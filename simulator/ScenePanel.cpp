// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.
#include "Ui.hpp"

#include <imgui.h>

#include <algorithm>
#include <iostream>
#include <cstdint>

static constexpr char const* STOP = "Stop";

/// Colored frame around the view: the state of the run at a glance.
static void runFrame(App const& p_app, ImVec2 p_at, ImVec2 p_size)
{
    ImVec4 color;
    if (!runColor(p_app, color))
    {
        return;
    }
    constexpr float THICKNESS = 3.0F;
    float const half = THICKNESS * 0.5F;
    ImGui::GetWindowDrawList()->AddRect(
        ImVec2(p_at.x + half, p_at.y + half),
        ImVec2(p_at.x + p_size.x - half, p_at.y + p_size.y - half),
        ImGui::GetColorU32(color),
        0.0F,
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

bool emergencyAvailable(App const& p_app)
{
    return p_app.simulation &&
           p_app.simulation->cell().skills().find(STOP) != robotik::NO_SKILL;
}

bool emergencyEngaged(App const& p_app)
{
    if (!p_app.simulation)
    {
        return false;
    }
    robotik::SkillScheduler const& skills = p_app.simulation->cell().skills();
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
    robotik::SkillScheduler& skills = p_app.simulation->cell().skills();
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
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
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
    ImVec2 const size(std::max(ImGui::GetContentRegionAvail().x, 1.0F),
                      std::max(ImGui::GetContentRegionAvail().y, 1.0F));
    ImVec2 const scale = ImGui::GetIO().DisplayFramebufferScale;
    if (auto sized = p_app.view.resize(static_cast<std::uint32_t>(size.x * scale.x),
                                       static_cast<std::uint32_t>(size.y * scale.y));
        !sized)
    {
        std::cerr << "3D view: " << sized.error() << '\n';
    }
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
    if (p_app.view.valid())
    {
        // The device puts the first row at the bottom.
        ImGui::GetWindowDrawList()->AddImage(
            ImTextureRef(static_cast<ImTextureID>(p_app.view.color().nativeId())),
            at,
            ImVec2(at.x + size.x, at.y + size.y),
            ImVec2(0.0F, 1.0F),
            ImVec2(1.0F, 0.0F));
    }
    runFrame(p_app, at, size);
    cameraInset(p_app, at, size);
    char const* hint =
        "Left drag: move   Right drag: turn   Wheel: zoom   WASD QE RF";
    ImVec2 const text = ImGui::CalcTextSize(hint);
    ImVec2 const corner(at.x + 10.0F, at.y + size.y - text.y - 12.0F);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(
        corner,
        ImVec2(corner.x + text.x + 12.0F, corner.y + text.y + 6.0F),
        IM_COL32(20, 22, 27, 170),
        4.0F);
    draw->AddText(ImVec2(corner.x + 6.0F, corner.y + 3.0F),
                  ImGui::GetColorU32(ImGuiCol_TextDisabled),
                  hint);
    ImGui::End();
}
