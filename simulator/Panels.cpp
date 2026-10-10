// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "App.hpp"
#include "FlyHost.hpp"
#include "PluginUi.hpp"
#include "SkillView.hpp"

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Robot/Actuators.hpp"

#include <imgui.h>
#include <imgui_internal.h>
#include "imgui_stdlib.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <string>
#include <vector>

static char const* const WORLD = "World";
static char const* const CAMERA = "Robot camera";
static char const* const TREE = "Behavior tree";
static char const* const RESOURCES = "Resources";
static char const* const ROBOT = "Robot";
static char const* const SCENARIO = "Scenario";
static char const* const SCENARIO_FILE = "Scenario file";
static char const* const FLY_BRAIN = "Fly network";

static char const* const STOP = "Stop";

static ImVec4 colorOf(bt::Status p_status)
{
    switch (p_status)
    {
        case bt::Status::INVALID:
            return GREY;
        case bt::Status::RUNNING:
            return ORANGE;
        case bt::Status::SUCCESS:
            return GREEN;
        case bt::Status::FAILURE:
            return RED;
    }
    return GREY;
}

static char const* textOf(bt::Status p_status)
{
    switch (p_status)
    {
        case bt::Status::INVALID:
            return "IDLE";
        case bt::Status::RUNNING:
            return "RUNNING";
        case bt::Status::SUCCESS:
            return "SUCCESS";
        case bt::Status::FAILURE:
            return "FAILURE";
    }
    return "IDLE";
}

//! @brief Layout the docking nodes.
//! @param p_dock The dock ID.
static void layout(ImGuiID p_dock)
{
    ImGui::DockBuilderRemoveNode(p_dock);
    ImGui::DockBuilderAddNode(p_dock, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(p_dock, ImGui::GetMainViewport()->WorkSize);
    ImGuiID main = p_dock;
    ImGuiID const tools = ImGui::DockBuilderSplitNode(
        main, ImGuiDir_Down, 0.18f, nullptr, &main);
    ImGuiID const timeline = ImGui::DockBuilderSplitNode(
        main, ImGuiDir_Down, 0.30f, nullptr, &main);
    ImGuiID const reference = ImGui::DockBuilderSplitNode(
        main, ImGuiDir_Left, 0.22f, nullptr, &main);
    ImGuiID world = 0;
    ImGuiID const live = ImGui::DockBuilderSplitNode(
        main, ImGuiDir_Right, 0.30f, nullptr, &world);
    ImGuiID camera = 0;
    ImGuiID const skills = ImGui::DockBuilderSplitNode(
        live, ImGuiDir_Down, 0.58f, nullptr, &camera);
    ImGui::DockBuilderDockWindow(SCENARIO, reference);
    ImGui::DockBuilderDockWindow(SCENARIO_FILE, reference);
    ImGui::DockBuilderDockWindow(TREE, reference);
    ImGui::DockBuilderDockWindow(ROBOT, reference);
    ImGui::DockBuilderDockWindow(TEACH_PANEL, reference);
    ImGui::DockBuilderDockWindow(FLY_BRAIN, reference);
    ImGui::DockBuilderDockWindow(WORLD, world);
    ImGui::DockBuilderDockWindow(CAMERA, camera);
    ImGui::DockBuilderDockWindow(SKILLS_PANEL, skills);
    ImGui::DockBuilderDockWindow(TIMELINE_PANEL, timeline);
    ImGui::DockBuilderDockWindow(SKILL_LIST_PANEL, tools);
    ImGui::DockBuilderDockWindow(RESOURCES, tools);
    ImGui::DockBuilderFinish(p_dock);
}

//! @brief Emergency stop: the Stop skill preempts every actuator.
static void emergencyStop(App& p_app)
{
    robotik::SkillScheduler& skills = p_app.simulation->skills();
    robotik::SkillId const stop = skills.find(STOP);
    if (stop == robotik::NO_SKILL)
    {
        return;
    }
    robotik::SkillState const state = skills.state(stop);
    bool const engaged = state == robotik::SkillState::Running ||
                         state == robotik::SkillState::Waiting;
    ImGui::PushStyleColor(ImGuiCol_Button,
                          engaged ? ImVec4(0.30f, 0.30f, 0.32f, 1.0f)
                                  : ImVec4(0.75f, 0.12f, 0.12f, 1.0f));
    if (ImGui::Button(engaged ? "Release stop" : "EMERGENCY STOP"))
    {
        if (engaged)
        {
            skills.cancel(stop);
        }
        else
        {
            skills.request(stop);
        }
    }
    ImGui::PopStyleColor();
}

//! @brief Draw the toolbar.
//! @param p_app The application.
static void toolbar(App& p_app)
{
    if (!ImGui::BeginMainMenuBar())
    {
        return;
    }
    static std::string path;
    static std::filesystem::path shown;
    if (shown != p_app.scenario_path)
    {
        shown = p_app.scenario_path;
        path = shown.string();
    }
    drawPluginMenus(p_app);
    if (!p_app.scenario_title.empty())
    {
        ImGui::TextUnformatted(p_app.scenario_title.c_str());
    }
    ImGui::SetNextItemWidth(280.0f);
    ImGui::InputText("##scenario", &path);
    if (ImGui::Button("Load"))
    {
        p_app.scenario_path = std::filesystem::path(path);
        p_app.load();
    }
    ImGui::Separator();
    if (ImGui::Button(p_app.playing ? "Pause" : "Play"))
    {
        p_app.playing = !p_app.playing;
    }
    ImGui::BeginDisabled(p_app.playing);
    if (ImGui::Button("Step"))
    {
        p_app.step_once = true;
    }
    ImGui::EndDisabled();
    ImGui::SetNextItemWidth(90.0f);
    ImGui::InputScalar("seed", ImGuiDataType_U64, &p_app.seed);
    if (ImGui::Button("Replay"))
    {
        p_app.reset(p_app.seed);
    }
    if (ImGui::Button("New seed"))
    {
        p_app.reset(p_app.seed + 1u);
    }
    ImGui::SetNextItemWidth(110.0f);
    ImGui::SliderFloat("##speed", &p_app.speed, 0.1f, 3.0f, "speed x%.1f");
    ImGui::Separator();
    if (p_app.simulation)
    {
        emergencyStop(p_app);
        ImGui::Text("t = %6.2f s", p_app.simulation->time().value());
        ImGui::TextColored(colorOf(p_app.simulation->status()),
                           "Mission %s",
                           textOf(p_app.simulation->status()));
    }
    else if (p_app.fly.environment)
    {
        FlySnapshot const& snapshot = p_app.fly.environment->snapshot();
        ImGui::Text("t = %6.2f s",
                    static_cast<double>(snapshot.steps) *
                        p_app.fly.environment->dt());
        ImGui::TextColored(
            p_app.fly.success ? GREEN : (p_app.fly.done ? RED : ORANGE),
            "Fly %s",
            p_app.fly.success ? "FOOD" : (p_app.fly.done ? "TIME" : "FLY"));
        ImGui::Text("L %.2f  C %.2f  R %.2f",
                    static_cast<double>(snapshot.vision.left),
                    static_cast<double>(snapshot.vision.center),
                    static_cast<double>(snapshot.vision.right));
        ImGui::Text("F %.2f  T %+.2f  Z %+.2f",
                    static_cast<double>(p_app.fly.action[0]),
                    static_cast<double>(p_app.fly.action[1]),
                    static_cast<double>(p_app.fly.action[2]));
    }
    if (!p_app.error.empty())
    {
        ImGui::TextColored(RED, "%s", p_app.error.c_str());
    }
    ImGui::EndMainMenuBar();
}

//! @brief Draw the world panel.
//! @param p_app The application.
static void worldPanel(App& p_app)
{
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    bool const visible = ImGui::Begin(WORLD,
                                      nullptr,
                                      ImGuiWindowFlags_NoScrollbar |
                                          ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    p_app.view_hovered = false;
    if (visible)
    {
        ImVec2 const size(std::max(ImGui::GetContentRegionAvail().x, 1.0f),
                          std::max(ImGui::GetContentRegionAvail().y, 1.0f));
        ImVec2 const scale = ImGui::GetIO().DisplayFramebufferScale;
        p_app.view.resize(static_cast<std::uint32_t>(size.x * scale.x),
                          static_cast<std::uint32_t>(size.y * scale.y));
        ImVec2 const at = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("##world",
                               size,
                               ImGuiButtonFlags_MouseButtonLeft |
                                   ImGuiButtonFlags_MouseButtonRight);
        p_app.view_hovered = ImGui::IsItemHovered() || ImGui::IsItemActive();
        if (p_app.view.width > 0)
        {
            // The device puts the first row at the bottom.
            ImGui::GetWindowDrawList()->AddImage(
                ImTextureRef(
                    static_cast<ImTextureID>(p_app.view.color.nativeId())),
                at,
                ImVec2(at.x + size.x, at.y + size.y),
                ImVec2(0.0f, 1.0f),
                ImVec2(1.0f, 0.0f));
        }
        ImGui::SetCursorScreenPos(ImVec2(at.x + 8.0f, at.y + 6.0f));
        ImGui::TextDisabled("Right drag: orbit   Wheel: zoom");
        if (p_app.fly.environment)
        {
            ImGui::SetCursorScreenPos(ImVec2(at.x + 8.0f, at.y + 24.0f));
            ImGui::TextColored(ImVec4(0.95f, 0.75f, 0.20f, 1.0f), "Yellow");
            ImGui::SameLine();
            ImGui::TextDisabled("left eye");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.95f, 0.25f, 0.20f, 1.0f), "Red");
            ImGui::SameLine();
            ImGui::TextDisabled("ahead");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.25f, 0.45f, 1.00f, 1.0f), "Blue");
            ImGui::SameLine();
            ImGui::TextDisabled("right eye. Sphere = start, tip = end.");
        }
    }
    ImGui::End();
}

//! @brief Draw the camera panel.
//! @param p_app The application.
static void cameraPanel(App& p_app)
{
    if (!ImGui::Begin(CAMERA))
    {
        ImGui::End();
        return;
    }
    if (p_app.fly.environment)
    {
        char const* const titles[2] = { "Oeil gauche", "Oeil droit" };
        for (int eye = 0; eye < 2; ++eye)
        {
            ImGui::SeparatorText(titles[eye]);
            RenderTarget const& picture = p_app.fly.eye_picture[eye];
            if (!p_app.fly.eyes[eye] || picture.width == 0)
            {
                ImGui::TextDisabled("Pas d'image.");
                continue;
            }
            float const avail = ImGui::GetContentRegionAvail().x;
            float const zoom = avail / static_cast<float>(picture.width);
            ImGui::Image(
                ImTextureRef(
                    static_cast<ImTextureID>(picture.color.nativeId())),
                ImVec2(avail, static_cast<float>(picture.height) * zoom),
                ImVec2(0.0f, 1.0f),
                ImVec2(1.0f, 0.0f));
        }
        ImGui::TextDisabled("Axe optique +X de eye_L et eye_R, champ 70 deg.");
        ImGui::End();
        return;
    }
    robotik::Camera* camera =
        p_app.simulation ? p_app.simulation->camera() : nullptr;
    if (camera == nullptr || !p_app.scene_view)
    {
        ImGui::TextDisabled("The scenario mounts no camera.");
        ImGui::End();
        return;
    }
    bool const has_cpu = p_app.scene_view->showFrame(camera->frame());
    RenderTarget const& picture =
        has_cpu ? p_app.scene_view->frameTarget()
                : (p_app.scene_view->cameras().empty()
                       ? p_app.scene_view->frameTarget()
                       : p_app.scene_view->cameras().front()->target());
    bool const available =
        p_app.simulation->robot().resources().available(camera->name());
    ImGui::Text("%s  %ux%u  fov %.0f deg  frame %llu",
                camera->name().c_str(),
                camera->intrinsics().width,
                camera->intrinsics().height,
                camera->intrinsics().fov().value() * 180.0 / std::numbers::pi,
                static_cast<unsigned long long>(camera->frame().sequence));
    if (!available)
    {
        ImGui::SameLine();
        ImGui::TextColored(RED, "FAILED (resource)");
        ImGui::TextDisabled(
            "Sensor offline; overlays cleared. Mission may continue from the "
            "world model after Detect succeeded.");
    }
    if (picture.width == 0)
    {
        ImGui::TextDisabled("No frame yet.");
        ImGui::End();
        return;
    }
    float const avail = ImGui::GetContentRegionAvail().x;
    float const zoom = avail / static_cast<float>(picture.width);
    ImVec2 const size(avail, static_cast<float>(picture.height) * zoom);
    ImVec2 const at = ImGui::GetCursorScreenPos();
    ImGui::Image(
        ImTextureRef(static_cast<ImTextureID>(picture.color.nativeId())),
        size,
        ImVec2(0.0f, 1.0f),
        ImVec2(1.0f, 0.0f));
    if (!available)
    {
        ImGui::GetWindowDrawList()->AddRectFilled(
            at, ImVec2(at.x + size.x, at.y + size.y), IM_COL32(0, 0, 0, 160));
    }

    robotik::Detections const& detections =
        p_app.simulation->perception().detections();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    for (robotik::Detection const& item : detections.items)
    {
        ImVec2 const top(at.x + static_cast<float>(item.box[0]) * zoom,
                         at.y + static_cast<float>(item.box[1]) * zoom);
        ImVec2 const end(at.x + static_cast<float>(item.box[2] + 1) * zoom,
                         at.y + static_cast<float>(item.box[3] + 1) * zoom);
        draw->AddRect(top, end, IM_COL32(80, 255, 120, 255), 0.0f, 0, 2.0f);
        draw->AddText(ImVec2(top.x, top.y - 16.0f),
                      IM_COL32(80, 255, 120, 255),
                      item.label.c_str());
    }
    if (ImGui::BeginTable(
            "detections", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Detection");
        ImGui::TableSetupColumn("Fill");
        ImGui::TableSetupColumn("Box (px)");
        ImGui::TableHeadersRow();
        for (robotik::Detection const& item : detections.items)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(item.label.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%.0f %%",
                        static_cast<double>(item.confidence) * 100.0);
            ImGui::TableNextColumn();
            ImGui::Text("%d,%d - %d,%d",
                        item.box[0],
                        item.box[1],
                        item.box[2],
                        item.box[3]);
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

//! @brief Draw a tree node.
//! @param p_node The node.
static void treeNode(bt::Node const& p_node)
{
    ImGui::PushID(&p_node);
    auto const* composite = dynamic_cast<bt::Composite const*>(&p_node);
    auto const* decorator = dynamic_cast<bt::Decorator const*>(&p_node);
    bool const leaf = composite == nullptr &&
                      (decorator == nullptr || !decorator->hasChild());
    ImGuiTreeNodeFlags flags =
        ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_SpanAvailWidth;
    if (leaf)
    {
        flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen |
                 ImGuiTreeNodeFlags_Bullet;
    }
    ImGui::PushStyleColor(ImGuiCol_Text, colorOf(p_node.status()));
    bool const open = ImGui::TreeNodeEx(
        "##node", flags, "%s  %s", p_node.typeName(), p_node.name.c_str());
    ImGui::PopStyleColor();
    if (!leaf && open)
    {
        if (composite != nullptr)
        {
            for (std::size_t i = 0; i < composite->childIndices().size(); ++i)
            {
                treeNode(composite->childAt(i));
            }
        }
        else
        {
            treeNode(decorator->childNode());
        }
        ImGui::TreePop();
    }
    ImGui::PopID();
}

//! @brief Draw the tree panel.
//! @param p_app The application.
static void treePanel(App const& p_app)
{
    if (ImGui::Begin(TREE))
    {
        bt::Tree const* tree =
            p_app.simulation ? p_app.simulation->tree() : nullptr;
        if (tree == nullptr)
        {
            ImGui::TextDisabled("No behavior tree.");
        }
        else
        {
            ImGui::TextDisabled(
                "%s",
                p_app.simulation->scenario().behavior_tree.string().c_str());
            ImGui::TextColored(GREY, "idle");
            ImGui::SameLine();
            ImGui::TextColored(ORANGE, "running");
            ImGui::SameLine();
            ImGui::TextColored(GREEN, "success");
            ImGui::SameLine();
            ImGui::TextColored(RED, "failure");
            ImGui::Separator();
            treeNode(tree->getRoot());
        }
    }
    ImGui::End();
}

struct ControlFailure
{
    std::string label;
    double time = 0.0;
};

//! @brief Failed composites and decorators. Skill leaves already have a row.
static void collectControlFailures(bt::Node const& p_node,
                                   std::vector<std::string>& p_labels)
{
    bool const control =
        dynamic_cast<bt::Composite const*>(&p_node) != nullptr ||
        dynamic_cast<bt::Decorator const*>(&p_node) != nullptr;
    if (control && p_node.status() == bt::Status::FAILURE)
    {
        std::string label = p_node.typeName();
        if (!p_node.name.empty())
        {
            label += ' ';
            label += p_node.name;
        }
        p_labels.push_back(std::move(label));
    }
    if (auto const* composite = dynamic_cast<bt::Composite const*>(&p_node))
    {
        for (std::size_t i = 0; i < composite->childIndices().size(); ++i)
        {
            collectControlFailures(composite->childAt(i), p_labels);
        }
    }
    else if (auto const* decorator =
                 dynamic_cast<bt::Decorator const*>(&p_node);
             decorator != nullptr && decorator->hasChild())
    {
        collectControlFailures(decorator->childNode(), p_labels);
    }
}

//! @brief Remember when a control node first failed, so the mark stays put.
static std::vector<ControlFailure> const&
controlFailures(robotik::Simulation const& p_simulation)
{
    static std::vector<ControlFailure> marks;
    static bt::Tree const* tree = nullptr;
    static double last_time = 0.0;
    double const now = p_simulation.time().value();
    bt::Tree const* current = p_simulation.tree();
    if (current != tree || now + 1e-6 < last_time)
    {
        marks.clear();
        tree = current;
    }
    last_time = now;
    if (current == nullptr || !current->hasRoot())
    {
        marks.clear();
        return marks;
    }

    std::vector<std::string> labels;
    collectControlFailures(current->getRoot(), labels);
    std::vector<ControlFailure> next;
    next.reserve(labels.size());
    for (std::string const& label : labels)
    {
        auto const known = std::find_if(marks.begin(),
                                        marks.end(),
                                        [&](ControlFailure const& p_mark)
                                        { return p_mark.label == label; });
        next.push_back(known == marks.end() ? ControlFailure{ label, now }
                                            : *known);
    }
    marks = std::move(next);
    return marks;
}

//! @brief Draw the timeline of the skill runs on a common time axis.
static void timeline(robotik::Simulation const& p_simulation)
{
    robotik::SkillScheduler const& skills = p_simulation.skills();
    std::span<robotik::SkillRun const> const runs = skills.trace();
    std::vector<ControlFailure> const& failures = controlFailures(p_simulation);
    double const span = std::max(p_simulation.time().value(), 1.0);
    ImVec2 const at = ImGui::GetCursorScreenPos();
    float const width = ImGui::GetContentRegionAvail().x;
    float label = 120.0f;
    for (ControlFailure const& failure : failures)
    {
        label = std::max(label,
                         ImGui::CalcTextSize(failure.label.c_str()).x + 12.0f);
    }
    float const row = 16.0f;
    float const scale = std::max(width - label - 10.0f, 10.0f);
    float const head = static_cast<float>(failures.size()) * row;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImU32 const red = ImGui::GetColorU32(RED);
    for (std::size_t i = 0; i < failures.size(); ++i)
    {
        float const y = at.y + static_cast<float>(i) * row;
        draw->AddText(ImVec2(at.x, y), red, failures[i].label.c_str());
        float const x0 =
            at.x + label + static_cast<float>(failures[i].time / span) * scale;
        float const x1 =
            at.x + label +
            static_cast<float>(p_simulation.time().value() / span) * scale;
        draw->AddRectFilled(ImVec2(x0, y + 3.0f),
                            ImVec2(std::max(x1, x0 + 6.0f), y + row - 3.0f),
                            red,
                            2.0f);
    }
    for (std::size_t id = 0; id < skills.size(); ++id)
    {
        float const y = at.y + head + static_cast<float>(id) * row;
        draw->AddText(ImVec2(at.x, y),
                      ImGui::GetColorU32(colorOf(skills.state(id))),
                      skills.name(id).c_str());
    }
    for (robotik::SkillRun const& run : runs)
    {
        float const y = at.y + head + static_cast<float>(run.skill) * row;
        float const x0 =
            at.x + label + static_cast<float>(run.start.value() / span) * scale;
        float const x1 =
            at.x + label + static_cast<float>(run.end.value() / span) * scale;
        draw->AddRectFilled(ImVec2(x0, y + 3.0f),
                            ImVec2(std::max(x1, x0 + 2.0f), y + row - 3.0f),
                            ImGui::GetColorU32(colorOf(run.state)),
                            2.0f);
    }
    ImGui::Dummy(
        ImVec2(width, head + static_cast<float>(skills.size()) * row + 4.0f));
}

//! @brief Draw the timeline panel.
//! @param p_app The application.
static void skillsPanel(App const& p_app)
{
    if (!ImGui::Begin(TIMELINE_PANEL) || !p_app.simulation)
    {
        ImGui::End();
        return;
    }
    timeline(*p_app.simulation);
    ImGui::End();
}

//! @brief Skill table: priority, resources, live state and manual requests.
//! @param p_app The application.
static void skillListPanel(App& p_app)
{
    if (!ImGui::Begin(SKILL_LIST_PANEL) || !p_app.simulation)
    {
        ImGui::End();
        return;
    }
    robotik::Simulation& simulation = *p_app.simulation;
    robotik::SkillScheduler& skills = simulation.skills();
    robotik::ResourceManager const& resources = simulation.robot().resources();
    if (ImGui::BeginTable("skills",
                          6,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
                              ImGuiTableFlags_ScrollY))
    {
        ImGui::TableSetupColumn("Skill");
        ImGui::TableSetupColumn("Priority");
        ImGui::TableSetupColumn("Resources");
        ImGui::TableSetupColumn("State");
        ImGui::TableSetupColumn("Why", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("");
        ImGui::TableHeadersRow();
        for (robotik::SkillId id = 0; id < skills.size(); ++id)
        {
            robotik::SkillDescription const& description =
                skills.description(id);
            robotik::SkillState const state = skills.state(id);
            ImGui::PushID(static_cast<int>(id));
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(description.name.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%d%s",
                        description.priority,
                        description.cancellable ? "" : " (locked)");
            ImGui::TableNextColumn();
            std::string needs;
            for (robotik::ResourceRequirement const& need :
                 description.resources)
            {
                needs += needs.empty() ? "" : " ";
                needs += resources.name(need.id);
            }
            ImGui::TextUnformatted(needs.c_str());
            ImGui::TableNextColumn();
            ImGui::TextColored(colorOf(state), "%s", robotik::toString(state));
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(skillWhy(skills, resources, id).c_str());
            ImGui::TableNextColumn();
            if (state == robotik::SkillState::Running ||
                state == robotik::SkillState::Waiting)
            {
                if (ImGui::SmallButton("Cancel"))
                {
                    skills.cancel(id);
                }
            }
            else if (ImGui::SmallButton("Request"))
            {
                skills.request(id);
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

//! @brief Names of the skills currently holding @p_id.
static std::string holdersOf(robotik::SkillScheduler const& p_skills,
                             robotik::ResourceManager const& p_resources,
                             robotik::ResourceId p_id)
{
    std::uint16_t const shared = p_resources.users(p_id);
    if (shared > 0u)
    {
        std::string names;
        for (robotik::SkillId id = 0; id < p_skills.size(); ++id)
        {
            if (p_skills.state(id) != robotik::SkillState::Running)
            {
                continue;
            }
            for (robotik::ResourceRequirement const& need :
                 p_skills.description(id).resources)
            {
                if (need.id != p_id || need.access != robotik::Access::Shared)
                {
                    continue;
                }
                if (!names.empty())
                {
                    names += ", ";
                }
                names += p_skills.name(id);
            }
        }
        if (names.empty())
        {
            names = "?";
        }
        names += " (";
        names += std::to_string(shared);
        names += ")";
        return names;
    }
    if (p_resources.owner(p_id) != robotik::NO_OWNER)
    {
        return skillOwnerName(p_skills, p_resources.owner(p_id));
    }
    return "-";
}

//! @brief Draw the resources panel: lease mode, holders and failure buttons.
//! @param p_app The application.
static void resourcesPanel(App& p_app)
{
    if (!ImGui::Begin(RESOURCES) || !p_app.simulation)
    {
        ImGui::End();
        return;
    }
    robotik::Simulation& simulation = *p_app.simulation;
    robotik::ResourceManager& resources = simulation.robot().resources();
    robotik::SkillScheduler const& skills = simulation.skills();
    if (ImGui::BeginTable(
            "resources", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Resource");
        ImGui::TableSetupColumn("Type");
        ImGui::TableSetupColumn("Status");
        ImGui::TableSetupColumn("Holders", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("");
        ImGui::TableHeadersRow();
        for (robotik::ResourceId id = 0; id < resources.size(); ++id)
        {
            bool const available = resources.available(id);
            bool const shared = resources.users(id) > 0u;
            bool const exclusive = resources.owner(id) != robotik::NO_OWNER;
            ImGui::PushID(static_cast<int>(id));
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(resources.name(id).c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(shared      ? "shared"
                                   : exclusive ? "exclusive"
                                               : "-");
            ImGui::TableNextColumn();
            ImGui::TextColored(
                available ? GREEN : RED, "%s", available ? "ok" : "FAILED");
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(holdersOf(skills, resources, id).c_str());
            ImGui::TableNextColumn();
            if (ImGui::SmallButton(available ? "Disable" : "Restore"))
            {
                if (available)
                {
                    resources.fail(id);
                }
                else
                {
                    resources.restore(id);
                }
            }
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

//! @brief Draw the robot panel.
//! @param p_app The application.
static void robotPanel(App const& p_app)
{
    if (!ImGui::Begin(ROBOT) || !p_app.simulation)
    {
        ImGui::End();
        return;
    }
    robotik::RobotSession const& robot = p_app.simulation->robot();
    robotik::JointSet const& joints = robot.joints();
    double const degrees = 180.0 / std::numbers::pi;

    ImGui::SeparatorText("Joints");
    if (ImGui::BeginTable(
            "joints", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Joint");
        ImGui::TableSetupColumn("Position", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Goal");
        ImGui::TableSetupColumn("Torque");
        ImGui::TableHeadersRow();
        for (robotik::JointId id = 0; id < joints.size(); ++id)
        {
            bool const linear = joints.isPrismatic(id);
            double lower = 0.0;
            double upper = 0.0;
            double effort_limit = 0.0;
            bool bounded = false;
            if (linear)
            {
                auto const limits = joints.limits(joints.prismatic(id));
                lower = limits.lower.value();
                upper = limits.upper.value();
                effort_limit = limits.effort.value();
                bounded = limits.bounded();
            }
            else
            {
                auto const limits = joints.limits(joints.revolute(id));
                lower = limits.lower.value();
                upper = limits.upper.value();
                effort_limit = limits.effort.value();
                bounded = limits.bounded();
            }
            double const unit = linear ? 1.0 : degrees;
            double const position = joints.position(id);
            double const range = upper - lower;
            float const ratio =
                bounded && range > 0.0
                    ? static_cast<float>((position - lower) / range)
                    : 0.5f;
            char const* label = nullptr;
            char const* label_end = nullptr;
            ImFormatStringToTempBuffer(&label,
                                       &label_end,
                                       linear ? "%.3f m" : "%.1f deg",
                                       position * unit);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(joints.name(id).c_str());
            ImGui::TableNextColumn();
            ImGui::ProgressBar(ratio, ImVec2(-1.0f, 0.0f), label);
            ImGui::TableNextColumn();
            if (joints.mode(id) == robotik::JointMode::Disabled)
            {
                ImGui::TextColored(RED, "off");
            }
            else
            {
                ImGui::Text(linear ? "%.3f" : "%.1f", joints.target(id) * unit);
            }
            ImGui::TableNextColumn();
            double const effort = joints.effort(id);
            bool const saturated =
                effort_limit > 0.0 && std::abs(effort) >= effort_limit;
            ImGui::TextColored(
                saturated ? RED : ImGui::GetStyle().Colors[ImGuiCol_Text],
                "%.1f",
                effort);
        }
        ImGui::EndTable();
    }

    auto const* gripper = robot.actuators().first<robotik::VacuumGripper>();
    if (gripper != nullptr)
    {
        ImGui::SeparatorText("Tool");
        robotik::Pose const flange = gripper->flange(robot);
        robotik::Vector3 const tip = gripper->tip(robot);
        ImGui::Text("Flange %s", robot.tool().c_str());
        ImGui::Text("  xyz  %.3f %.3f %.3f m",
                    flange.position.x,
                    flange.position.y,
                    flange.position.z);
        ImGui::Text("Suction cup  %.3f %.3f %.3f m", tip.x, tip.y, tip.z);
        if (gripper->holding())
        {
            ImGui::TextColored(
                ORANGE,
                "Holding %s",
                compages::world::Entity(robot.world(), gripper->held())
                    .name()
                    .c_str());
        }
        else
        {
            ImGui::TextDisabled(gripper->suction() ? "Suction on, empty"
                                                   : "Gripper empty");
        }
    }
    ImGui::End();
}

//! @brief Draw the scenario panel: ground truth against the world model.
//! @param p_app The application.
static void scenarioPanel(App const& p_app)
{
    if (!ImGui::Begin(SCENARIO) || !p_app.simulation)
    {
        ImGui::End();
        return;
    }
    robotik::Simulation& simulation = *p_app.simulation;
    robotik::Scenario const& scenario = simulation.scenario();
    ImGui::Text("%s", scenario.name.c_str());
    ImGui::TextDisabled("%s", scenario.robot_model.string().c_str());
    ImGui::TextDisabled(
        "seed %llu", static_cast<unsigned long long>(simulation.seed().value));
    ImGui::SeparatorText("Task");
    ImGui::TextWrapped("%s", scenario.task.c_str());

    ImGui::SeparatorText("Objects: truth / belief");
    if (ImGui::BeginTable(
            "objects", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Name");
        ImGui::TableSetupColumn("Truth (m)");
        ImGui::TableSetupColumn("Error (mm)");
        ImGui::TableSetupColumn("Seen (s)");
        ImGui::TableHeadersRow();
        robotik::WorldModel const& beliefs = simulation.worldModel();
        p_app.world->each<robotik::ecs::SceneObject>(
            [&](compages::world::Entity p_entity,
                robotik::ecs::SceneObject const& p_object)
            {
                auto const at = p_entity.position();
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::ColorButton("##color",
                                   ImVec4(p_object.color[0],
                                          p_object.color[1],
                                          p_object.color[2],
                                          1.0f),
                                   ImGuiColorEditFlags_NoTooltip,
                                   ImVec2(12.0f, 12.0f));
                ImGui::SameLine();
                ImGui::TextUnformatted(p_object.name.c_str());
                ImGui::TableNextColumn();
                ImGui::Text("%.3f %.3f %.3f",
                            static_cast<double>(at.x),
                            static_cast<double>(at.y),
                            static_cast<double>(at.z));
                robotik::WorldObject const* belief =
                    beliefs.find(p_object.name);
                ImGui::TableNextColumn();
                if (belief != nullptr)
                {
                    double const dx =
                        belief->position.x - static_cast<double>(at.x);
                    double const dy =
                        belief->position.y - static_cast<double>(at.y);
                    double const dz =
                        belief->position.z - static_cast<double>(at.z);
                    ImGui::Text("%.1f",
                                1000.0 *
                                    std::sqrt(dx * dx + dy * dy + dz * dz));
                }
                ImGui::TableNextColumn();
                if (belief != nullptr && belief->observed())
                {
                    ImGui::Text("%.2f", belief->seen.value());
                }
                else
                {
                    ImGui::TextDisabled("prior");
                }
            });
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Assertions");
    for (auto const& check : simulation.checks())
    {
        ImVec4 const color = check.passed ? GREEN : RED;
        ImGui::TextColored(color,
                           "%s  %s",
                           check.passed ? "[ok]" : "[ko]",
                           check.text.c_str());
        if (!check.passed && !check.detail.empty())
        {
            ImGui::SameLine();
            ImGui::TextDisabled("%s", check.detail.c_str());
        }
    }
    ImGui::TextDisabled("Contacts seen: %d", simulation.maxContacts());
    ImGui::End();
}

//! @brief Raw YAML of the loaded scenario file.
static void scenarioFilePanel(App& p_app)
{
    if (!ImGui::Begin(SCENARIO_FILE))
    {
        ImGui::End();
        return;
    }
    ImGui::TextUnformatted(p_app.scenario_path.string().c_str());
    ImGui::Separator();
    if (p_app.scenario_text.empty())
    {
        ImGui::TextDisabled("No scenario file loaded.");
    }
    else
    {
        ImGui::InputTextMultiline("##yaml",
                                  &p_app.scenario_text,
                                  ImVec2(-1.0f, -1.0f),
                                  ImGuiInputTextFlags_ReadOnly);
    }
    ImGui::End();
}

//! @brief Thirteen-neuron circuit, or the extracted FlyWire connectome.
static void flyBrainPanel(App& p_app)
{
    if (!ImGui::Begin(FLY_BRAIN))
    {
        ImGui::End();
        return;
    }
    if (!p_app.fly.brain)
    {
        ImGui::TextWrapped(
            "Open Fly brain from the Demos menu. This panel switches the "
            "network.");
        ImGui::End();
        return;
    }

    bool const connectome = p_app.fly.connectome;
    if (ImGui::RadioButton("13 neurons", !connectome))
    {
        if (connectome)
        {
            selectFlyBrain(p_app, false);
        }
    }
    if (ImGui::RadioButton("Connectome", connectome))
    {
        if (!connectome)
        {
            selectFlyBrain(p_app, true);
        }
    }
    ImGui::Text("%u neurons, %llu synapses",
                p_app.fly.brain->neurons(),
                static_cast<unsigned long long>(p_app.fly.brain->synapses()));
    if (p_app.fly.connectome)
    {
        ImGui::TextDisabled("%s", FLYWIRE_EDGES);
        ImGui::TextWrapped(
            "binding.txt: each output is a real target of its input, "
            "heavy enough to pass a spike. Not the eye neurons.");
    }
    else
    {
        ImGui::TextWrapped(
            "Synapses written in the code. The connectome is "
            "data/flywire/edges.csv, extracted by make compile-external-libs.");
    }
    ImGui::End();
}

//! @brief Draw the panels.
//! @param p_app The application.
void drawPanels(App& p_app)
{
    ImGuiID const dock = ImGui::GetID("RobotikDockV5");
    // A layout saved in imgui.ini wins over the default one.
    if (ImGui::DockBuilderGetNode(dock) == nullptr)
    {
        layout(dock);
    }
    toolbar(p_app);
    ImGui::DockSpaceOverViewport(dock, ImGui::GetMainViewport());
    worldPanel(p_app);
    cameraPanel(p_app);
    treePanel(p_app);
    skillsPanel(p_app);
    skillListPanel(p_app);
    skillsBoard(p_app);
    resourcesPanel(p_app);
    scenarioPanel(p_app);
    scenarioFilePanel(p_app);
    flyBrainPanel(p_app);
    robotPanel(p_app);
    teachPanel(p_app);
    drawPluginPanels(p_app);
}
