// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "App.hpp"

#include "Robotik/Actuators/Actuator.hpp"
#include "Robotik/ECS/ObjectComponents.hpp"

#include <imgui.h>
#include <imgui_internal.h>
#include "imgui_stdlib.h" // after imgui.h (IMGUI_API)

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <numbers>
#include <string>

static char const* const WORLD = "World";
static char const* const CAMERA = "Robot camera";
static char const* const TREE = "Behavior tree";
static char const* const SKILLS = "Skills";
static char const* const RESOURCES = "Resources";
static char const* const ROBOT = "Robot";
static char const* const SCENARIO = "Scenario";

static char const* const STOP = "Stop";

static ImVec4 const GREY(0.55f, 0.55f, 0.58f, 1.0f);
static ImVec4 const ORANGE(1.00f, 0.70f, 0.20f, 1.0f);
static ImVec4 const GREEN(0.35f, 0.85f, 0.40f, 1.0f);
static ImVec4 const RED(0.95f, 0.35f, 0.35f, 1.0f);
static ImVec4 const BLUE(0.45f, 0.65f, 1.00f, 1.0f);

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

static ImVec4 colorOf(robotik::SkillState p_state)
{
    switch (p_state)
    {
        case robotik::SkillState::Idle:
            return GREY;
        case robotik::SkillState::Waiting:
            return BLUE;
        case robotik::SkillState::Running:
            return ORANGE;
        case robotik::SkillState::Succeeded:
            return GREEN;
        case robotik::SkillState::Failed:
            return RED;
        case robotik::SkillState::Cancelled:
            return GREY;
    }
    return GREY;
}

//! @brief Name of the skill holding a lease, "-" when none.
static std::string ownerName(robotik::SkillScheduler const& p_skills,
                             robotik::OwnerId p_owner)
{
    if (p_owner == robotik::NO_OWNER)
    {
        return "-";
    }
    if (p_owner == robotik::ANONYMOUS || p_owner >= p_skills.size())
    {
        return "(external)";
    }
    return p_skills.name(p_owner);
}

//! @brief Why a skill waits or stopped, in operator words.
static std::string whyOf(robotik::Simulation const& p_simulation,
                         robotik::SkillId p_id)
{
    robotik::SkillScheduler const& skills = p_simulation.skills();
    robotik::SkillReason const reason = skills.reason(p_id);
    if (reason == robotik::SkillReason::None)
    {
        return {};
    }
    std::string text = robotik::toString(reason);
    if (reason == robotik::SkillReason::Precondition)
    {
        if (robotik::Precondition const* failed = skills.precondition(p_id))
        {
            text += ": " + failed->text;
        }
        return text;
    }
    robotik::ResourceId const blocker = skills.blocker(p_id);
    if (blocker != robotik::NO_RESOURCE)
    {
        robotik::ResourceManager const& resources =
            p_simulation.robot().resources();
        text += ": " + resources.name(blocker);
        if (reason == robotik::SkillReason::Busy ||
            reason == robotik::SkillReason::Preempted)
        {
            text += " (" + ownerName(skills, resources.owner(blocker)) + ")";
        }
    }
    return text;
}

//! @brief Layout the docking nodes.
//! @param p_dock The dock ID.
static void layout(ImGuiID p_dock)
{
    ImGui::DockBuilderRemoveNode(p_dock);
    ImGui::DockBuilderAddNode(p_dock, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(p_dock, ImGui::GetMainViewport()->WorkSize);
    ImGuiID center = p_dock;
    ImGuiID const left = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Left, 0.24f, nullptr, &center);
    ImGuiID const right = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Right, 0.30f, nullptr, &center);
    ImGuiID const bottom = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Down, 0.32f, nullptr, &center);
    ImGuiID right_bottom = 0;
    ImGuiID const right_top = ImGui::DockBuilderSplitNode(
        right, ImGuiDir_Up, 0.45f, nullptr, &right_bottom);
    ImGui::DockBuilderDockWindow(SCENARIO, left);
    ImGui::DockBuilderDockWindow(ROBOT, left);
    ImGui::DockBuilderDockWindow(WORLD, center);
    ImGui::DockBuilderDockWindow(SKILLS, bottom);
    ImGui::DockBuilderDockWindow(RESOURCES, bottom);
    ImGui::DockBuilderDockWindow(CAMERA, right_top);
    ImGui::DockBuilderDockWindow(TREE, right_bottom);
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
    if (path.empty())
    {
        path = p_app.scenario_path.string();
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
    bool const visible =
        ImGui::Begin(WORLD, nullptr, ImGuiWindowFlags_NoScrollbar);
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
    robotik::Camera* camera =
        p_app.simulation ? p_app.simulation->camera() : nullptr;
    if (camera == nullptr || !p_app.scene_view ||
        p_app.scene_view->cameras().empty())
    {
        ImGui::TextDisabled("The scenario mounts no camera.");
        ImGui::End();
        return;
    }
    RenderTarget const& picture = p_app.scene_view->cameras().front()->target();
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
        ImGui::TextColored(RED, "FAILED");
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

//! @brief Draw the timeline of the skill runs on a common time axis.
static void timeline(robotik::Simulation const& p_simulation)
{
    robotik::SkillScheduler const& skills = p_simulation.skills();
    std::span<robotik::SkillRun const> const runs = skills.trace();
    double const span = std::max(p_simulation.time().value(), 1.0);
    ImVec2 const at = ImGui::GetCursorScreenPos();
    float const width = ImGui::GetContentRegionAvail().x;
    float const label = 120.0f;
    float const row = 16.0f;
    float const scale = std::max(width - label - 10.0f, 10.0f);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    for (std::size_t id = 0; id < skills.size(); ++id)
    {
        float const y = at.y + static_cast<float>(id) * row;
        draw->AddText(ImVec2(at.x, y),
                      ImGui::GetColorU32(colorOf(skills.state(id))),
                      skills.name(id).c_str());
    }
    for (robotik::SkillRun const& run : runs)
    {
        float const y = at.y + static_cast<float>(run.skill) * row;
        float const x0 = at.x + label +
                         static_cast<float>(run.start.value() / span) * scale;
        float const x1 =
            at.x + label + static_cast<float>(run.end.value() / span) * scale;
        draw->AddRectFilled(ImVec2(x0, y + 3.0f),
                            ImVec2(std::max(x1, x0 + 2.0f), y + row - 3.0f),
                            ImGui::GetColorU32(colorOf(run.state)),
                            2.0f);
    }
    ImGui::Dummy(
        ImVec2(width, static_cast<float>(skills.size()) * row + 4.0f));
}

//! @brief Draw the skills panel: metadata, live state and manual requests.
//! @param p_app The application.
static void skillsPanel(App& p_app)
{
    if (!ImGui::Begin(SKILLS) || !p_app.simulation)
    {
        ImGui::End();
        return;
    }
    robotik::Simulation& simulation = *p_app.simulation;
    robotik::SkillScheduler& skills = simulation.skills();
    robotik::ResourceManager const& resources = simulation.robot().resources();
    timeline(simulation);

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
            robotik::SkillDescription const& description = skills.description(id);
            robotik::SkillState const state = skills.state(id);
            ImGui::PushID(static_cast<int>(id));
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(description.name.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%d%s", description.priority, description.cancellable ? "" : " (locked)");
            ImGui::TableNextColumn();
            std::string needs;
            for (robotik::ResourceRequirement const& need : description.resources)
            {
                needs += needs.empty() ? "" : " ";
                needs += resources.name(need.id);
                if (need.access == robotik::Access::Shared)
                {
                    needs += "(s)";
                }
            }
            ImGui::TextUnformatted(needs.c_str());
            ImGui::TableNextColumn();
            ImGui::TextColored(colorOf(state), "%s", robotik::toString(state));
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(whyOf(simulation, id).c_str());
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

//! @brief Draw the resources panel: availability, holders, failure buttons
//! and the fault plan of the scenario.
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
        ImGui::TableSetupColumn("Status");
        ImGui::TableSetupColumn("Holder");
        ImGui::TableSetupColumn("Users");
        ImGui::TableSetupColumn("");
        ImGui::TableHeadersRow();
        for (robotik::ResourceId id = 0; id < resources.size(); ++id)
        {
            bool const available = resources.available(id);
            ImGui::PushID(static_cast<int>(id));
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(resources.name(id).c_str());
            ImGui::TableNextColumn();
            ImGui::TextColored(available ? GREEN : RED,
                               "%s",
                               available ? "ok" : "FAILED");
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(ownerName(skills, resources.owner(id)).c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%u", static_cast<unsigned>(resources.users(id)));
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

    robotik::FaultInjector const& faults = simulation.faults();
    ImGui::SeparatorText("Scenario faults");
    if (faults.scheduled().empty() && faults.random().empty())
    {
        ImGui::TextDisabled("None.");
    }
    double const now = simulation.time().value();
    for (robotik::Fault const& fault : faults.scheduled())
    {
        bool const past = fault.at.value() <= now;
        ImGui::TextColored(past ? GREY : ImGui::GetStyle().Colors[ImGuiCol_Text],
                           "t=%5.2f s  %s %s",
                           fault.at.value(),
                           fault.disable ? "disable" : "restore",
                           fault.resource.c_str());
    }
    for (robotik::RandomFault const& fault : faults.random())
    {
        ImGui::Text("random  %s  %.3f /s", fault.resource.c_str(), fault.rate);
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
            robotik::JointLimits const& limits = joints.limits(id);
            bool const linear = joints.type(id) == robotik::JointType::Prismatic;
            double const unit = linear ? 1.0 : degrees;
            double const position = joints.position(id);
            double const range = limits.upper - limits.lower;
            float const ratio =
                limits.bounded() && range > 0.0
                    ? static_cast<float>((position - limits.lower) / range)
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
                limits.effort > 0.0 && std::abs(effort) >= limits.effort;
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
                compages::world::Entity(robot.world(), gripper->held()).name().c_str());
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
    ImGui::TextDisabled("seed %llu",
                        static_cast<unsigned long long>(simulation.seed().value));
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
                robotik::WorldObject const* belief = beliefs.find(p_object.name);
                ImGui::TableNextColumn();
                if (belief != nullptr)
                {
                    double const dx = belief->position.x - static_cast<double>(at.x);
                    double const dy = belief->position.y - static_cast<double>(at.y);
                    double const dz = belief->position.z - static_cast<double>(at.z);
                    ImGui::Text("%.1f", 1000.0 * std::sqrt(dx * dx + dy * dy + dz * dz));
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
    bool const done = simulation.finished();
    for (auto const& check : simulation.checks())
    {
        ImVec4 const color = check.passed ? GREEN : (done ? RED : GREY);
        ImGui::TextColored(color,
                           "%s  %s",
                           check.passed ? "[ok]" : "[--]",
                           check.text.c_str());
    }
    ImGui::TextDisabled("Contacts seen: %d", simulation.maxContacts());
    ImGui::End();
}

//! @brief Draw the panels.
//! @param p_app The application.
void drawPanels(App& p_app)
{
    ImGuiID const dock = ImGui::GetID("RobotikDock");
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
    resourcesPanel(p_app);
    scenarioPanel(p_app);
    robotPanel(p_app);
}
