// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Ui.hpp"

#include "Robotik/ECS/RobotComponents.hpp"
#include "Robotik/Robot/Actuators.hpp"

#include "Compages/Renderer/Scene.hpp"

#include <imgui.h>
#include "imgui_stdlib.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <span>
#include <string>
#include <utility>
#include <vector>

void takeArm(App& p_app)
{
    if (p_app.teach.manual || p_app.owns_clock || !p_app.simulation)
    {
        return;
    }
    p_app.teach.manual = true;
    p_app.simulation->suspend(true);
    p_app.simulation->robot().joints().hold();
}

namespace
{

void syncMarkers(App& p_app)
{
    if (!p_app.simulation || !p_app.scene || !p_app.world)
    {
        return;
    }
    std::span<robotik::TeachWaypoint const> const points =
        p_app.teach.pendant.waypoints();
    std::vector<compages::world::Entity>& markers = p_app.teach.markers;
    compages::world::Entity const root = p_app.simulation->robot().root();
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        if (i == markers.size() || !p_app.world->alive(markers[i].id()))
        {
            compages::world::Entity marker = p_app.scene->box(
                "teach_" + std::to_string(++p_app.teach.marker_serial),
                compages::renderer::color(1.0f, 0.72f, 0.15f));
            marker.parent(root).scale(0.02f);
            if (i == markers.size())
            {
                markers.push_back(marker);
            }
            else
            {
                markers[i] = marker;
            }
        }
        robotik::Vector3 const at = points[i].pose.position;
        markers[i]
            .position(static_cast<float>(at.x),
                      static_cast<float>(at.y),
                      static_cast<float>(at.z))
            .set(robotik::ecs::TeachMarker{ i });
    }
    while (markers.size() > points.size())
    {
        if (p_app.world->alive(markers.back().id()))
        {
            markers.back().destroy();
        }
        markers.pop_back();
    }
}

//! @brief Mounted tool. A change reloads the scenario with that URDF.
//! @return True when the simulation was reloaded. The caller must stop using
//! the previous robot.
bool changeTool(App& p_app)
{
    robotik::WorldDescription const& world = p_app.simulation->scenario().world;
    robotik::EntitySpec const* mounted = world.mountedTool();
    std::string const current = mounted != nullptr ? mounted->id : std::string{};
    std::string const label = current.empty() ? std::string("None") : current;

    bool changed = false;
    std::string chosen;
    ImGui::SetNextItemWidth(-1.0f);
    if (!ImGui::BeginCombo("##tool", label.c_str()))
    {
        return false;
    }
    if (ImGui::Selectable("None", current.empty()))
    {
        changed = !current.empty();
    }
    for (robotik::EntitySpec const& tool : world.entities)
    {
        if (tool.type != robotik::EntitySpec::Type::Tool)
        {
            continue;
        }
        bool const selected = tool.id == current;
        if (ImGui::Selectable(tool.id.c_str(), selected) && !selected)
        {
            changed = true;
            chosen = tool.id;
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s", tool.model_ref.c_str());
        }
    }
    ImGui::EndCombo();
    if (!changed)
    {
        return false;
    }
    p_app.tool_override = true;
    p_app.tool_override_name = std::move(chosen);
    p_app.load(true);
    return true;
}

//! @brief Actuates the mounted tool: suction, finger close, or spindle.
void triggerTool(App& p_app, robotik::RobotSession& p_robot)
{
    // Kind of tool: the model key of its asset id (robotik.tools.vacuum).
    robotik::EntitySpec const* tool = p_app.simulation->scenario().world.mountedTool();
    std::string mounted;
    if (tool != nullptr)
    {
        mounted = tool->model_ref.substr(tool->model_ref.find_last_of('.') + 1u);
    }
    robotik::JointSet& joints = p_robot.joints();
    if (mounted == "vacuum")
    {
        robotik::VacuumGripper* gripper =
            p_robot.actuators().first<robotik::VacuumGripper>();
        if (gripper == nullptr)
        {
            ImGui::TextDisabled("Trigger: no suction cup");
            return;
        }
        bool on = gripper->suction();
        if (ImGui::Checkbox("Trigger", &on))
        {
            takeArm(p_app);
            gripper->suction(on);
        }
        return;
    }
    if (mounted == "gripper")
    {
        bool closed = true;
        bool any = false;
        for (robotik::JointId id = 0; id < joints.size(); ++id)
        {
            if (!joints.isPrismatic(id))
            {
                continue;
            }
            any = true;
            auto const limits = joints.limits(joints.prismatic(id));
            double const mid =
                0.5 * (limits.lower.value() + limits.upper.value());
            double const commanded = joints.mode(id) == robotik::JointMode::Position
                                         ? joints.target(id)
                                         : joints.position(id);
            closed = closed && commanded <= mid;
        }
        if (!any)
        {
            ImGui::BeginDisabled();
        }
        if (ImGui::Checkbox("Trigger", &closed))
        {
            takeArm(p_app);
            for (robotik::JointId id = 0; id < joints.size(); ++id)
            {
                if (!joints.isPrismatic(id))
                {
                    continue;
                }
                auto const limits = joints.limits(joints.prismatic(id));
                joints.moveTo(id,
                              closed ? limits.lower.value()
                                     : limits.upper.value());
            }
        }
        if (!any)
        {
            ImGui::EndDisabled();
        }
        ImGui::SameLine();
        ImGui::TextDisabled(any ? (closed ? "closed" : "open")
                                : "the gripper has no finger joint");
        return;
    }
    if (mounted == "drill")
    {
        robotik::JointId const spindle = joints.find("drill_spindle_joint");
        if (spindle == robotik::NO_JOINT)
        {
            ImGui::TextDisabled("Trigger: no spindle");
            return;
        }
        bool spinning = joints.mode(spindle) == robotik::JointMode::Velocity &&
                        std::abs(joints.target(spindle)) > 1e-3;
        if (ImGui::Checkbox("Trigger", &spinning))
        {
            takeArm(p_app);
            if (!spinning)
            {
                joints.hold(spindle);
            }
            else
            {
                double speed =
                    joints.limits(joints.revolute(spindle)).velocity.value();
                if (!(speed > 0.0))
                {
                    speed = 20.0;
                }
                joints.spin(spindle, std::min(speed, 20.0));
            }
        }
        ImGui::SameLine();
        ImGui::TextDisabled(spinning ? "spinning" : "stopped");
        return;
    }
    ImGui::TextDisabled("Trigger: this tool has none");
}


//! @brief One jog row: "-" value "+" for a translation or a rotation axis.
void jogRow(App& p_app,
            char const* p_axis,
            robotik::Vector3 const& p_shift,
            robotik::Vector3 const& p_turn)
{
    robotik::RobotSession& robot = p_app.simulation->robot();
    ImGui::PushID(p_axis);
    ImGui::TableNextColumn();
    if (ImGui::Button("-", ImVec2(-1.0f, 0.0f)))
    {
        takeArm(p_app);
        p_app.teach.pendant.jogTool(robot,
                                    robotik::Vector3(-p_shift.x, -p_shift.y, -p_shift.z),
                                    robotik::Vector3(-p_turn.x, -p_turn.y, -p_turn.z));
    }
    ImGui::TableNextColumn();
    ImGui::AlignTextToFramePadding();
    float const width = ImGui::CalcTextSize(p_axis).x;
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (ImGui::GetContentRegionAvail().x - width) * 0.5f);
    ImGui::TextUnformatted(p_axis);
    ImGui::TableNextColumn();
    if (ImGui::Button("+", ImVec2(-1.0f, 0.0f)))
    {
        takeArm(p_app);
        p_app.teach.pendant.jogTool(robot, p_shift, p_turn);
    }
    ImGui::PopID();
}

void cartesianJog(App& p_app)
{
    robotik::RobotSession& robot = p_app.simulation->robot();
    ImGui::SeparatorText("Move the tool");
    ImGui::SameLine();
    helpMarker("Each press moves the tool frame by one step in the robot base frame. "
               "Inverse kinematics (Pinocchio) finds the joint angles. "
               "Moving takes the robot from the mission (Manual mode).");
    if (!robot.tool().empty())
    {
        robotik::Pose const tool = robot.framePose(robot.tool());
        ImGui::TextDisabled("%s at  x %.3f  y %.3f  z %.3f m", robot.tool().c_str(), tool.position.x,
                            tool.position.y, tool.position.z);
    }
    float const linear = p_app.teach.linear_step;
    float const angular = p_app.teach.angular_step;
    robotik::Vector3 const none = robotik::zero3();
    if (beginTable("jog", 7, ImGuiTableFlags_SizingStretchSame))
    {
        // Two blocks of three columns: translation, then rotation.
        for (int column = 0; column < 7; ++column)
        {
            ImGui::TableSetupColumn(column == 3 ? "##gap" : nullptr,
                                    column == 3 ? ImGuiTableColumnFlags_WidthFixed
                                                : ImGuiTableColumnFlags_WidthStretch,
                                    column == 3 ? 12.0f : 0.0f);
        }
        jogRow(p_app, "X", { linear, 0.0, 0.0 }, none);
        ImGui::TableNextColumn();
        jogRow(p_app, "Rx", none, { angular, 0.0, 0.0 });
        jogRow(p_app, "Y", { 0.0, linear, 0.0 }, none);
        ImGui::TableNextColumn();
        jogRow(p_app, "Ry", none, { 0.0, angular, 0.0 });
        jogRow(p_app, "Z", { 0.0, 0.0, linear }, none);
        ImGui::TableNextColumn();
        jogRow(p_app, "Rz", none, { 0.0, 0.0, angular });
        endTable();
    }
    float const half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    ImGui::SetNextItemWidth(half);
    ImGui::SliderFloat("##linear", &p_app.teach.linear_step, 0.001f, 0.05f, "step %.3f m");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(half);
    ImGui::SliderFloat("##angular", &p_app.teach.angular_step, 0.01f, 0.30f, "step %.2f rad");
    ImGui::SetNextItemWidth(half);
    ImGui::SliderFloat("##joint", &p_app.teach.joint_step, 0.01f, 0.40f, "joint step %.2f rad");
    ImGui::SetItemTooltip("Step of the - and + buttons of the Robot panel.");
}

void waypoints(App& p_app)
{
    robotik::RobotSession& robot = p_app.simulation->robot();
    robotik::TeachPendant& pendant = p_app.teach.pendant;
    ImGui::SeparatorText("Waypoints");
    ImGui::SameLine();
    helpMarker("Record the current pose as a waypoint, then play them in order. "
               "The duration is the time to reach the waypoint from the previous one.");
    float const width = ImGui::GetContentRegionAvail().x;
    ImGui::SetNextItemWidth(width * 0.45f);
    ImGui::InputTextWithHint("##label", "label (optional)", &p_app.teach.label);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(width * 0.30f);
    ImGui::SliderFloat("##duration", &p_app.teach.duration, 0.2f, 8.0f, "%.1f s");
    ImGui::SameLine();
    if (ImGui::Button("Record", ImVec2(-1.0f, 0.0f)))
    {
        pendant.record(robot, p_app.teach.label, static_cast<double>(p_app.teach.duration));
    }

    std::span<robotik::TeachWaypoint const> const points = pendant.waypoints();
    if (points.empty())
    {
        ImGui::TextDisabled("No waypoint yet.");
    }
    else if (beginTable("waypoints", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV))
    {
        ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Duration", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();
        int erase = -1;
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            ImGui::PushID(static_cast<int>(i));
            ImGui::TableNextRow();
            if (pendant.segment() == static_cast<int>(i))
            {
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, IM_COL32(255, 184, 38, 50));
            }
            ImGui::TableNextColumn();
            ImGui::Text("%zu", i);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(points[i].label.empty() ? "-" : points[i].label.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%.1f s", points[i].duration);
            ImGui::TableNextColumn();
            if (ImGui::SmallButton("Go"))
            {
                takeArm(p_app);
                pendant.goTo(robot, i);
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Delete"))
            {
                erase = static_cast<int>(i);
            }
            ImGui::PopID();
        }
        endTable();
        if (erase >= 0)
        {
            pendant.erase(static_cast<std::size_t>(erase));
        }
    }
    ImGui::BeginDisabled(points.empty());
    if (ImGui::Button(pendant.playing() ? "Playing..." : "Play waypoints"))
    {
        takeArm(p_app);
        pendant.play(robot, p_app.teach.loop);
    }
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &p_app.teach.loop);
    ImGui::SameLine();
    if (ImGui::Button("Stop"))
    {
        pendant.stop(robot);
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear all"))
    {
        pendant.clear();
    }
    ImGui::EndDisabled();
    if (pendant.playing() && !p_app.playing)
    {
        ImGui::TextColored(ORANGE, "Press Play (Space) to run the motion.");
    }
    if (!pendant.error().empty())
    {
        ImGui::TextColored(RED, "%s", pendant.error().c_str());
    }
}

} // namespace

void teachPanel(App& p_app)
{
    if (!ImGui::Begin("Teach"))
    {
        ImGui::End();
        return;
    }
    if (!p_app.simulation)
    {
        emptyState("No scenario loaded.");
        ImGui::End();
        return;
    }
    if (p_app.owns_clock)
    {
        emptyState("Manual moves are not available while the RL plugin drives the robot.");
        ImGui::End();
        return;
    }

    ImGui::SeparatorText("Tool");
    if (changeTool(p_app))
    {
        ImGui::End();
        return;
    }
    triggerTool(p_app, p_app.simulation->robot());

    cartesianJog(p_app);
    waypoints(p_app);
    syncMarkers(p_app);
    ImGui::End();
}
