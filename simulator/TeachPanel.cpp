// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "App.hpp"

#include "Robotik/ECS/RobotComponents.hpp"
#include "Robotik/Robot/Actuators.hpp"

#include "Compages/Renderer/Scene.hpp"

#include <imgui.h>
#include "imgui_stdlib.h"

#include <span>
#include <string>
#include <vector>

namespace
{

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

} // namespace

void teachPanel(App& p_app)
{
    if (!ImGui::Begin(TEACH_PANEL))
    {
        ImGui::End();
        return;
    }
    if (!p_app.simulation)
    {
        ImGui::TextDisabled("Load a scenario first.");
        ImGui::End();
        return;
    }

    robotik::RobotSession& robot = p_app.simulation->robot();
    robotik::TeachPendant& pendant = p_app.teach.pendant;
    bool const rl = p_app.owns_clock;

    ImGui::TextDisabled("Cartesian jogs use Pinocchio IK. Play on the toolbar "
                        "runs the motion.");
    if (rl)
    {
        ImGui::TextDisabled("Unavailable during the RL episode.");
        ImGui::End();
        return;
    }

    if (ImGui::Checkbox("Manual", &p_app.teach.manual))
    {
        p_app.simulation->suspend(p_app.teach.manual);
        if (p_app.teach.manual)
        {
            robot.joints().hold();
        }
        else
        {
            pendant.stop(robot);
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled(p_app.teach.manual ? "mission suspended"
                                           : "mission running");

    if (robotik::VacuumGripper* gripper =
            robot.actuators().first<robotik::VacuumGripper>())
    {
        bool suction = gripper->suction();
        if (ImGui::Checkbox("Suction", &suction))
        {
            takeArm(p_app);
            gripper->suction(suction);
        }
    }

    ImGui::SeparatorText("Tool");
    if (!robot.tool().empty())
    {
        robotik::Pose const tool = robot.framePose(robot.tool());
        ImGui::Text("%s  %.3f  %.3f  %.3f m",
                    robot.tool().c_str(),
                    tool.position.x,
                    tool.position.y,
                    tool.position.z);
    }
    ImGui::SliderFloat("linear step (m)", &p_app.teach.linear_step, 0.001f, 0.05f, "%.3f");
    ImGui::SliderFloat("angular step (rad)", &p_app.teach.angular_step, 0.01f, 0.30f, "%.2f");

    float const linear = p_app.teach.linear_step;
    float const angular = p_app.teach.angular_step;
    auto nudge = [&](char const* p_name,
                     robotik::Vector3 const& p_shift,
                     robotik::Vector3 const& p_turn)
    {
        if (ImGui::Button(p_name))
        {
            takeArm(p_app);
            pendant.jogTool(robot, p_shift, p_turn);
        }
        ImGui::SameLine();
    };
    nudge("X-", { -linear, 0.0, 0.0 }, robotik::zero3());
    nudge("X+", { linear, 0.0, 0.0 }, robotik::zero3());
    nudge("Y-", { 0.0, -linear, 0.0 }, robotik::zero3());
    nudge("Y+", { 0.0, linear, 0.0 }, robotik::zero3());
    nudge("Z-", { 0.0, 0.0, -linear }, robotik::zero3());
    nudge("Z+", { 0.0, 0.0, linear }, robotik::zero3());
    ImGui::NewLine();
    nudge("Rx-", robotik::zero3(), { -angular, 0.0, 0.0 });
    nudge("Rx+", robotik::zero3(), { angular, 0.0, 0.0 });
    nudge("Ry-", robotik::zero3(), { 0.0, -angular, 0.0 });
    nudge("Ry+", robotik::zero3(), { 0.0, angular, 0.0 });
    nudge("Rz-", robotik::zero3(), { 0.0, 0.0, -angular });
    nudge("Rz+", robotik::zero3(), { 0.0, 0.0, angular });
    ImGui::NewLine();

    ImGui::SeparatorText("Joints");
    ImGui::SliderFloat("joint step (rad)", &p_app.teach.joint_step, 0.01f, 0.40f, "%.2f");
    robotik::JointSet& joints = robot.joints();
    for (robotik::JointId id = 0; id < joints.size(); ++id)
    {
        ImGui::PushID(static_cast<int>(id));
        double const step = joints.isPrismatic(id)
                                ? static_cast<double>(p_app.teach.linear_step)
                                : static_cast<double>(p_app.teach.joint_step);
        if (ImGui::Button("-"))
        {
            takeArm(p_app);
            pendant.jogJoint(robot, id, -step);
        }
        ImGui::SameLine();
        if (ImGui::Button("+"))
        {
            takeArm(p_app);
            pendant.jogJoint(robot, id, step);
        }
        ImGui::SameLine();
        ImGui::Text("%s  %.3f", joints.name(id).c_str(), joints.position(id));
        ImGui::PopID();
    }

    ImGui::SeparatorText("Trajectory");
    ImGui::InputText("Label", &p_app.teach.label);
    ImGui::SliderFloat("duration (s)", &p_app.teach.duration, 0.2f, 8.0f, "%.1f");
    if (ImGui::Button("Record"))
    {
        pendant.record(robot,
                       p_app.teach.label,
                       static_cast<double>(p_app.teach.duration));
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear"))
    {
        pendant.clear();
    }

    std::span<robotik::TeachWaypoint const> const points = pendant.waypoints();
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        ImGui::PushID(static_cast<int>(i));
        bool const current = pendant.segment() == static_cast<int>(i);
        if (current)
        {
            ImGui::TextColored(ImVec4(1.0f, 0.72f, 0.15f, 1.0f),
                               "%zu  %s  %.1f s",
                               i,
                               points[i].label.c_str(),
                               points[i].duration);
        }
        else
        {
            ImGui::Text("%zu  %s  %.1f s",
                        i,
                        points[i].label.c_str(),
                        points[i].duration);
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Go"))
        {
            takeArm(p_app);
            pendant.goTo(robot, i);
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Delete"))
        {
            pendant.erase(i);
            ImGui::PopID();
            break;
        }
        ImGui::PopID();
    }

    ImGui::Checkbox("Loop", &p_app.teach.loop);
    ImGui::SameLine();
    if (ImGui::Button("Play"))
    {
        takeArm(p_app);
        pendant.play(robot, p_app.teach.loop);
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop"))
    {
        pendant.stop(robot);
    }
    if (!pendant.error().empty())
    {
        ImGui::TextColored(ImVec4(0.95f, 0.35f, 0.35f, 1.0f),
                           "%s",
                           pendant.error().c_str());
    }

    syncMarkers(p_app);
    ImGui::End();
}
