// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.
#include "Ui.hpp"

#include "Robotik/Robot/Components/Objects.hpp"

#include "Compages/Core/AABB.hpp"

#include <imgui.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <stdexcept>

static constexpr char const* STOP = "Stop";

/// The robot or the object that entity @p_id belongs to: the first of its
/// ancestors that is an object (a shape, not a decoration) or the root of
/// a robot. Nothing for the floor or a decoration.
static Selection ownerOf(App& p_app, compages::world::EntityId p_id)
{
    compages::world::Entity entity = p_app.world->entity(p_id);
    while (entity)
    {
        robotik::ecs::Shape const* shape = entity.find<robotik::ecs::Shape>();
        if (shape != nullptr &&
            entity.find<robotik::ecs::Decoration>() == nullptr)
        {
            return { SelectionKind::Object, shape->name };
        }
        for (auto const& robot : p_app.simulation->cell().robots())
        {
            if (robot->root().id() == entity.id())
            {
                return { SelectionKind::Robot, robot->id() };
            }
        }
        entity = entity.parent();
    }
    return {};
}

/// A click without drag in the view selects what is under the mouse, or
/// nothing. @p_at and @p_size: the view on the screen.
static void pickInView(App& p_app, ImVec2 p_at, ImVec2 p_size)
{
    ImGuiIO const& io = ImGui::GetIO();
    bool const click = ImGui::IsItemDeactivated() &&
                       ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
                       io.MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] < 9.0F;
    if (!click || !p_app.scene)
    {
        return;
    }
    // The picture counts its pixels from the bottom left.
    ImVec2 const scale = io.DisplayFramebufferScale;
    compages::Vector2f const pixel((io.MousePos.x - p_at.x) * scale.x,
                                   (p_size.y - (io.MousePos.y - p_at.y)) *
                                       scale.y);
    std::optional<compages::renderer::RayHit> const hit =
        p_app.scene->pick(pixel);
    p_app.ui.selection = hit ? ownerOf(p_app, hit->entity) : Selection{};
}

/// The selected object in the 3D view: a light box where it is, an amber
/// one where the robot believes it is. Drawn at the next render only.
static void markSelection(App& p_app)
{
    Selection const& chosen = p_app.ui.selection;
    if (chosen.kind != SelectionKind::Object || !p_app.scene)
    {
        return;
    }
    robotik::Cell const& cell = p_app.simulation->cell();
    compages::renderer::DebugDraw& debug = p_app.scene->debug();
    compages::Vector3f half(0.02F, 0.02F, 0.02F);
    p_app.world->each<robotik::ecs::Shape>(
        [&](compages::world::Entity p_entity,
            robotik::ecs::Shape const& p_shape)
        {
            if (p_shape.name != chosen.name)
            {
                return;
            }
            half = compages::Vector3f(
                0.5F * static_cast<float>(p_shape.size[0].value()),
                0.5F * static_cast<float>(p_shape.size[1].value()),
                0.5F * static_cast<float>(p_shape.size[2].value()));
            // A bit larger than the object, so the lines are not hidden.
            compages::Vector3f const pad = half * 1.08F;
            debug.box(compages::AABB::fromCorners(-pad, pad),
                      p_entity.worldMatrix(),
                      compages::Vector3f(0.92F, 0.93F, 0.95F));
        });
    robotik::WorldObject const* belief = cell.worldModel().find(chosen.name);
    if (belief == nullptr)
    {
        return;
    }
    robotik::Vector3 const at = cell.beliefFrame() * belief->position;
    compages::Vector3f const center(static_cast<float>(at.x),
                                    static_cast<float>(at.y),
                                    static_cast<float>(at.z));
    compages::Vector3f const size =
        belief->size.x > 0.0
            ? compages::Vector3f(0.5F * static_cast<float>(belief->size.x),
                                 0.5F * static_cast<float>(belief->size.y),
                                 0.5F * static_cast<float>(belief->size.z))
            : half;
    debug.box(compages::AABB::fromCorners(center - size, center + size),
              compages::Matrix44f(compages::matrix::Identity),
              compages::Vector3f(0.93F, 0.71F, 0.39F));
}

/// Colored frame around the view: the state of the run at a glance.
static void runFrame(App const& p_app, ImVec2 p_at, ImVec2 p_size)
{
    ImVec4 color;
    if (!runColor(p_app, color))
    {
        return;
    }
    // The run bar carries the status; keep the running outline unobtrusive.
    if (p_app.simulation && !p_app.simulation->finished())
        color.w = 0.25F;
    constexpr float THICKNESS = 2.0F;
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

void sceneView(App& p_app, bool p_compact)
{
    if (!p_app.simulation)
    {
        emptyState(p_app.error.empty()
                       ? "No scenario loaded.\nChoose one in the Demos menu, "
                         "or File > Open scenario."
                       : p_app.error.c_str());
        return;
    }
    ImVec2 const size(std::max(ImGui::GetContentRegionAvail().x, 1.0F),
                      std::max(ImGui::GetContentRegionAvail().y, 1.0F));
    ImVec2 const scale = ImGui::GetIO().DisplayFramebufferScale;
    if (auto sized =
            p_app.view.resize(static_cast<std::uint32_t>(size.x * scale.x),
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
    pickInView(p_app, at, size);
    markSelection(p_app);
    if (p_app.view.valid())
    {
        // The device puts the first row at the bottom.
        ImGui::GetWindowDrawList()->AddImage(
            ImTextureRef(
                static_cast<ImTextureID>(p_app.view.color().nativeId())),
            at,
            ImVec2(at.x + size.x, at.y + size.y),
            ImVec2(0.0F, 1.0F),
            ImVec2(1.0F, 0.0F));
    }
    runFrame(p_app, at, size);
    if (p_compact)
        return;
    cameraInset(p_app, at, size);

    char const* hint =
        "Click: select   Left drag: move   Right drag: turn   Wheel: zoom   "
        "WASD QE RF";
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
}

void sceneInset(App& p_app)
{
    UiState& ui = p_app.ui;
    if (!p_app.simulation || ui.maximized_window.empty() ||
        ui.maximized_window == "3D scene")
        return;
    ImGuiViewport const* viewport = ImGui::GetMainViewport();
    float const width = std::min(320.0F, viewport->WorkSize.x * 0.30F);
    float const height = std::min(220.0F, viewport->WorkSize.y * 0.30F);
    ImVec2 const corner(viewport->WorkPos.x + viewport->WorkSize.x - 12.0F,
                        viewport->WorkPos.y + viewport->WorkSize.y - 12.0F);
    ImGui::SetNextWindowPos(corner, ImGuiCond_Always, ImVec2(1.0F, 1.0F));
    ImGui::SetNextWindowSize(ui.inset_scene ? ImVec2(width, height)
                                            : ImVec2(110.0F, 38.0F));
    ImGui::SetNextWindowBgAlpha(0.95F);
    ImGuiWindowFlags const flags =
        ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoScrollWithMouse |
        ImGuiWindowFlags_NoFocusOnAppearing;
    if (ImGui::Begin("##scene inset", nullptr, flags))
    {
        if (!ui.inset_scene)
        {
            if (ImGui::SmallButton("3D scene"))
                ui.inset_scene = true;
        }
        else
        {
            ImGui::TextUnformatted("3D scene");
            ImGui::SameLine();
            if (iconButton("restore", Icon::Restore, "Back to the layout"))
                ui.pending_maximized = std::string();
            ImGui::SameLine();
            if (iconButton("hide", Icon::Close, "Hide the 3D preview"))
                ui.inset_scene = false;
            if (ImGui::BeginChild("scene",
                                  ImVec2(0.0F, 0.0F),
                                  ImGuiChildFlags_None,
                                  ImGuiWindowFlags_NoScrollbar |
                                      ImGuiWindowFlags_NoScrollWithMouse))
                sceneView(p_app, true);
            ImGui::EndChild();
        }
    }
    ImGui::End();
}

void scenePanel(App& p_app)
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
    bool const visible = beginPanel(p_app,
                                    PanelKind::Scene,
                                    "3D scene",
                                    ImGuiWindowFlags_NoScrollbar |
                                        ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    if (visible)
        sceneView(p_app);
    ImGui::End();
}

void graphPanel(App& p_app)
{
    if (beginPanel(p_app, PanelKind::Graph, "Execution graph"))
        graphView(p_app);
    ImGui::End();
}
