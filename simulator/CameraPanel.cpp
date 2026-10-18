// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.
#include "Ui.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

/// @brief One camera: a title line and its picture, fitted in @p_cell
/// without changing the aspect ratio. Fitting both sides keeps the panel
/// free of scrollbars, so the picture does not shake.
static void cameraCell(App& p_app, robotik::Camera& p_camera, std::size_t p_index, ImVec2 p_cell)
{
    robotik::Simulation& simulation = *p_app.simulation;
    SimulatorView& view = *p_app.scene_view;
    bool const available = simulation.robot().resources().available(p_camera.name());
    ImGui::TextUnformatted(p_camera.name().c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("%ux%u  fov %.0f°  #%llu", p_camera.intrinsics().width,
                        p_camera.intrinsics().height,
                        p_camera.intrinsics().fov().value() * 180.0 / std::numbers::pi,
                        static_cast<unsigned long long>(p_camera.frame().sequence));
    if (!available)
    {
        ImGui::SameLine();
        badge("FAULT", RED);
    }

    RenderTarget const* picture = nullptr;
    if (view.showFrame(p_index, p_camera.frame()))
    {
        picture = &view.frameTarget(p_index);
    }
    else if (p_index < view.cameras().size() && view.cameras()[p_index]->target().width > 0)
    {
        picture = &view.cameras()[p_index]->target();
    }
    float const room = std::max(p_cell.y - ImGui::GetFrameHeightWithSpacing(), 16.0f);
    if (picture == nullptr || picture->width == 0)
    {
        ImGui::Dummy(ImVec2(p_cell.x, room));
        ImVec2 const min = ImGui::GetItemRectMin();
        ImVec2 const max = ImGui::GetItemRectMax();
        ImGui::GetWindowDrawList()->AddRect(min, max, ImGui::GetColorU32(ImGuiCol_Border), 4.0f);
        char const* text = "No frame yet: press Play or Step.";
        ImVec2 const size = ImGui::CalcTextSize(text);
        ImGui::GetWindowDrawList()->AddText(
            ImVec2((min.x + max.x - size.x) * 0.5f, (min.y + max.y - size.y) * 0.5f),
            ImGui::GetColorU32(ImGuiCol_TextDisabled), text);
        return;
    }
    float const aspect = static_cast<float>(picture->height) / static_cast<float>(picture->width);
    float const width = std::min(p_cell.x, room / aspect);
    ImVec2 const size(std::floor(width), std::floor(width * aspect));
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, (p_cell.x - size.x) * 0.5f));
    ImVec2 const at = ImGui::GetCursorScreenPos();
    ImGui::Image(ImTextureRef(static_cast<ImTextureID>(picture->color.nativeId())), size,
                 ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    if (!available)
    {
        draw->AddRectFilled(at, ImVec2(at.x + size.x, at.y + size.y), IM_COL32(0, 0, 0, 160));
        return;
    }
    // Perception runs on the first camera: draw what it found.
    if (&p_camera != simulation.camera())
    {
        return;
    }
    float const zoom = size.x / static_cast<float>(p_camera.intrinsics().width);
    for (robotik::Detection const& item : simulation.perception().detections().items)
    {
        ImVec2 const top(at.x + static_cast<float>(item.box[0]) * zoom,
                         at.y + static_cast<float>(item.box[1]) * zoom);
        ImVec2 const end(at.x + static_cast<float>(item.box[2] + 1) * zoom,
                         at.y + static_cast<float>(item.box[3] + 1) * zoom);
        draw->AddRect(top, end, IM_COL32(80, 255, 120, 255), 0.0f, 0, 2.0f);
        draw->AddText(ImVec2(top.x, std::max(at.y, top.y - 16.0f)), IM_COL32(80, 255, 120, 255),
                      item.label.c_str());
    }
}

void camerasPanel(App& p_app, char const* p_key)
{
    std::string const title = panelTitle("Cameras", p_key);
    if (!ImGui::Begin(title.c_str(), nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse))
    {
        ImGui::End();
        return;
    }
    if (!p_app.simulation || !p_app.scene_view)
    {
        emptyState("No scenario loaded.");
        ImGui::End();
        return;
    }
    robotik::SensorSet const& sensors = p_app.simulation->robot().sensors();
    std::vector<robotik::Camera*> cameras;
    for (std::size_t i = 0; i < sensors.size(); ++i)
    {
        if (auto* camera = dynamic_cast<robotik::Camera*>(&sensors[i]))
        {
            cameras.push_back(camera);
        }
    }
    if (cameras.empty())
    {
        emptyState("The robot has no camera.");
        ImGui::End();
        return;
    }
    // Side by side when the panel is wider than tall, stacked otherwise.
    ImVec2 const avail = ImGui::GetContentRegionAvail();
    std::size_t const count = cameras.size();
    std::size_t const columns = avail.x > avail.y ? std::min<std::size_t>(count, 3u) : 1u;
    std::size_t const rows = (count + columns - 1u) / columns;
    float const spacing = ImGui::GetStyle().ItemSpacing.x;
    ImVec2 const cell((avail.x - spacing * static_cast<float>(columns - 1u)) / static_cast<float>(columns),
                      avail.y / static_cast<float>(rows) - ImGui::GetStyle().ItemSpacing.y);
    for (std::size_t i = 0; i < count; ++i)
    {
        if (i % columns != 0u)
        {
            ImGui::SameLine();
        }
        ImGui::PushID(static_cast<int>(i));
        ImGui::BeginGroup();
        // Keeps every column as wide as the cell, whatever the picture.
        ImGui::Dummy(ImVec2(cell.x, 0.0f));
        ImGui::PushItemWidth(cell.x);
        cameraCell(p_app, *cameras[i], i, cell);
        ImGui::PopItemWidth();
        ImGui::EndGroup();
        ImGui::PopID();
    }
    ImGui::End();
}
