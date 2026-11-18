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

std::vector<robotik::Camera*> robotCameras(App& p_app)
{
    std::vector<robotik::Camera*> cameras;
    if (!p_app.simulation)
    {
        return cameras;
    }
    robotik::SensorSet const& sensors =
        p_app.simulation->cell().robot().sensors();
    for (std::size_t i = 0; i < sensors.size(); ++i)
    {
        if (auto* camera = dynamic_cast<robotik::Camera*>(&sensors[i]))
        {
            cameras.push_back(camera);
        }
    }
    return cameras;
}

/// The picture of camera @p_index: the frame the robot got, else what its
/// camera last drew; a null texture when there is none yet.
struct Picture
{
    compages::gpu::Texture const* texture = nullptr;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
};

/// The picture to show for camera @p_index.
static Picture pictureOf(App& p_app,
                         robotik::Camera const& p_camera,
                         std::size_t p_index)
{
    Picture picture;
    if (!p_app.scene_view)
    {
        return picture;
    }
    SimulatorView& view = *p_app.scene_view;
    if (view.showFrame(p_index, p_camera.frame()))
    {
        ShownPicture const& shown = view.frameTarget(p_index);
        picture = { &shown.color, shown.width, shown.height };
    }
    else if (p_index < view.cameras().size() &&
             view.cameras()[p_index]->target().valid())
    {
        RenderTarget const& drawn = view.cameras()[p_index]->target();
        picture = { &drawn.color(), drawn.width(), drawn.height() };
    }
    return picture;
}

/// The detections of the perception, over the picture of the first camera
/// drawn at @p_at with the width @p_width; a dark veil on a camera in fault.
static void overlay(App& p_app,
                    robotik::Camera const& p_camera,
                    ImVec2 p_at,
                    ImVec2 p_size)
{
    robotik::Simulation& simulation = *p_app.simulation;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    if (!simulation.cell().robot().resources().available(p_camera.name()))
    {
        draw->AddRectFilled(p_at,
                            ImVec2(p_at.x + p_size.x, p_at.y + p_size.y),
                            IM_COL32(0, 0, 0, 160));
        return;
    }
    // Perception runs on the first camera: draw what it found.
    if (&p_camera != simulation.cell().camera())
    {
        return;
    }
    float const zoom =
        p_size.x / static_cast<float>(p_camera.intrinsics().width);
    for (robotik::Detection const& item :
         simulation.cell().perception().detections().items)
    {
        ImVec2 const top(p_at.x + static_cast<float>(item.box[0]) * zoom,
                         p_at.y + static_cast<float>(item.box[1]) * zoom);
        ImVec2 const end(p_at.x + static_cast<float>(item.box[2] + 1) * zoom,
                         p_at.y + static_cast<float>(item.box[3] + 1) * zoom);
        draw->AddRect(top, end, IM_COL32(80, 255, 120, 255), 0.0F, 0, 2.0F);
        draw->AddText(ImVec2(top.x, std::max(p_at.y, top.y - 16.0F)),
                      IM_COL32(80, 255, 120, 255),
                      item.label.c_str());
    }
}

/// One camera: a title line and its picture, fitted in @p_cell
/// without changing the aspect ratio. Fitting both sides keeps the panel
/// free of scrollbars, so the picture does not shake.
static void cameraCell(App& p_app,
                       robotik::Camera& p_camera,
                       std::size_t p_index,
                       ImVec2 p_cell)
{
    robotik::Simulation& simulation = *p_app.simulation;
    bool const available =
        simulation.cell().robot().resources().available(p_camera.name());
    ImGui::TextUnformatted(p_camera.name().c_str());
    ImGui::SameLine();
    ImGui::TextDisabled(
        "%ux%u  fov %.0f°  #%llu",
        p_camera.intrinsics().width,
        p_camera.intrinsics().height,
        p_camera.intrinsics().fov().value() * 180.0 / std::numbers::pi,
        static_cast<unsigned long long>(p_camera.frame().sequence));
    if (!available)
    {
        ImGui::SameLine();
        badge("FAULT", RED);
    }

    Picture const shown = pictureOf(p_app, p_camera, p_index);
    compages::gpu::Texture const* picture = shown.texture;
    std::uint32_t const picture_width = shown.width;
    std::uint32_t const picture_height = shown.height;
    float const room =
        std::max(p_cell.y - ImGui::GetFrameHeightWithSpacing(), 16.0F);
    if (picture == nullptr || picture_width == 0)
    {
        ImGui::Dummy(ImVec2(p_cell.x, room));
        ImVec2 const min = ImGui::GetItemRectMin();
        ImVec2 const max = ImGui::GetItemRectMax();
        ImGui::GetWindowDrawList()->AddRect(
            min, max, ImGui::GetColorU32(ImGuiCol_Border), 4.0F);
        char const* text = "No frame yet: press Play or Step.";
        ImVec2 const size = ImGui::CalcTextSize(text);
        ImGui::GetWindowDrawList()->AddText(
            ImVec2((min.x + max.x - size.x) * 0.5F,
                   (min.y + max.y - size.y) * 0.5F),
            ImGui::GetColorU32(ImGuiCol_TextDisabled),
            text);
        return;
    }
    float const aspect = static_cast<float>(picture_height) /
                         static_cast<float>(picture_width);
    float const width = std::min(p_cell.x, room / aspect);
    ImVec2 const size(std::floor(width), std::floor(width * aspect));
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() +
                         std::max(0.0F, (p_cell.x - size.x) * 0.5F));
    ImVec2 const at = ImGui::GetCursorScreenPos();
    ImGui::Image(
        ImTextureRef(static_cast<ImTextureID>(picture->nativeId())),
        size,
        ImVec2(0.0F, 1.0F),
        ImVec2(1.0F, 0.0F));
    overlay(p_app, p_camera, at, size);
}

void camerasPanel(App& p_app, char const* p_key)
{
    std::string const title = panelTitle("Cameras", p_key);
    if (!ImGui::Begin(title.c_str(),
                      nullptr,
                      ImGuiWindowFlags_NoScrollbar |
                          ImGuiWindowFlags_NoScrollWithMouse))
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
    std::vector<robotik::Camera*> const cameras = robotCameras(p_app);
    if (cameras.empty())
    {
        emptyState("The robot has no camera.");
        ImGui::End();
        return;
    }
    // Side by side when the panel is wider than tall, stacked otherwise.
    ImVec2 const avail = ImGui::GetContentRegionAvail();
    std::size_t const count = cameras.size();
    std::size_t const columns =
        avail.x > avail.y ? std::min<std::size_t>(count, 3U) : 1U;
    std::size_t const rows = (count + columns - 1U) / columns;
    float const spacing = ImGui::GetStyle().ItemSpacing.x;
    ImVec2 const cell((avail.x - spacing * static_cast<float>(columns - 1U)) /
                          static_cast<float>(columns),
                      avail.y / static_cast<float>(rows) -
                          ImGui::GetStyle().ItemSpacing.y);
    for (std::size_t i = 0; i < count; ++i)
    {
        if (i % columns != 0U)
        {
            ImGui::SameLine();
        }
        ImGui::PushID(static_cast<int>(i));
        ImGui::BeginGroup();
        // Keeps every column as wide as the cell, whatever the picture.
        ImGui::Dummy(ImVec2(cell.x, 0.0F));
        ImGui::PushItemWidth(cell.x);
        cameraCell(p_app, *cameras[i], i, cell);
        ImGui::PopItemWidth();
        ImGui::EndGroup();
        ImGui::PopID();
    }
    ImGui::End();
}

void cameraInset(App& p_app, ImVec2 p_view_at, ImVec2 p_view_size)
{
    std::vector<robotik::Camera*> const cameras = robotCameras(p_app);
    if (cameras.empty() || !p_app.scene_view)
    {
        return;
    }
    UiState& ui = p_app.ui;
    float const margin = 12.0F;
    ImVec2 const corner(p_view_at.x + p_view_size.x - margin,
                        p_view_at.y + p_view_size.y - margin);
    if (!ui.inset_camera)
    {
        char const* label = "Camera";
        ImVec2 const size(ImGui::CalcTextSize(label).x + 18.0F,
                          ImGui::GetFrameHeight());
        ImGui::SetCursorScreenPos(ImVec2(corner.x - size.x, corner.y - size.y));
        if (ImGui::Button(label, size))
        {
            ui.inset_camera = true;
        }
        ImGui::SetItemTooltip("Show a robot camera over the scene.");
        return;
    }
    ui.inset_index = std::min(ui.inset_index, cameras.size() - 1U);
    robotik::Camera const& camera = *cameras[ui.inset_index];
    Picture const picture = pictureOf(p_app, camera, ui.inset_index);
    float const aspect =
        picture.width > 0U ? static_cast<float>(picture.height) /
                                 static_cast<float>(picture.width)
                           : 0.75F;
    float const width =
        std::clamp(p_view_size.x * 0.28F, 180.0F, 380.0F);
    float const head = ImGui::GetFrameHeight();
    ImVec2 const image(std::floor(width), std::floor(width * aspect));
    if (image.y + head + 2.0F * margin > p_view_size.y)
    {
        return;
    }
    ImVec2 const min(corner.x - image.x, corner.y - image.y - head);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(ImVec2(min.x - 1.0F, min.y - 1.0F),
                        ImVec2(corner.x + 1.0F, corner.y + 1.0F),
                        ImGui::GetColorU32(ImGuiCol_WindowBg),
                        5.0F);
    draw->AddRect(ImVec2(min.x - 1.0F, min.y - 1.0F),
                  ImVec2(corner.x + 1.0F, corner.y + 1.0F),
                  ImGui::GetColorU32(ImGuiCol_Border),
                  5.0F);

    // Title line: name, previous and next camera, hide.
    ImGui::PushID("inset");
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5.0F, 1.0F));
    ImGui::SetCursorScreenPos(ImVec2(min.x + 6.0F, min.y + 3.0F));
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(camera.name().c_str());
    float const buttons = cameras.size() > 1U ? 3.0F : 1.0F;
    ImGui::SameLine();
    ImGui::SetCursorScreenPos(ImVec2(
        corner.x - buttons * (ImGui::GetFrameHeight() + 4.0F) - 2.0F,
        min.y + 3.0F));
    if (cameras.size() > 1U)
    {
        if (ImGui::SmallButton("<"))
        {
            ui.inset_index =
                (ui.inset_index + cameras.size() - 1U) % cameras.size();
        }
        ImGui::SetItemTooltip("Previous camera");
        ImGui::SameLine(0.0F, 4.0F);
        if (ImGui::SmallButton(">"))
        {
            ui.inset_index = (ui.inset_index + 1U) % cameras.size();
        }
        ImGui::SetItemTooltip("Next camera");
        ImGui::SameLine(0.0F, 4.0F);
    }
    if (ImGui::SmallButton("x"))
    {
        ui.inset_camera = false;
    }
    ImGui::SetItemTooltip("Hide the camera");
    ImGui::PopStyleVar();

    // The picture; the mouse on it does not move the 3D view.
    ImVec2 const at(min.x, min.y + head);
    ImGui::SetCursorScreenPos(at);
    ImGui::InvisibleButton("picture", image);
    if (picture.texture == nullptr || picture.width == 0U)
    {
        char const* text = "No frame yet";
        ImVec2 const size = ImGui::CalcTextSize(text);
        draw->AddText(ImVec2(at.x + (image.x - size.x) * 0.5F,
                             at.y + (image.y - size.y) * 0.5F),
                      ImGui::GetColorU32(ImGuiCol_TextDisabled),
                      text);
    }
    else
    {
        draw->AddImage(
            ImTextureRef(static_cast<ImTextureID>(picture.texture->nativeId())),
            at,
            ImVec2(at.x + image.x, at.y + image.y),
            ImVec2(0.0F, 1.0F),
            ImVec2(1.0F, 0.0F));
        overlay(p_app, camera, at, image);
    }
    ImGui::PopID();
}
