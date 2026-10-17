// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.
#include "Ui.hpp"

#include "Robotik/ECS/ObjectComponents.hpp"

#include <imgui.h>

#include <string>
#include <unordered_set>

namespace
{

void oracleLine(robotik::Simulation const& p_simulation)
{
    if (p_simulation.oracle())
    {
        badge("ORACLE", ORANGE);
        ImGui::SameLine();
        ImGui::TextDisabled("the ground truth feeds the beliefs (no camera image)");
    }
    else
    {
        badge("SENSORS", GREEN);
        ImGui::SameLine();
        ImGui::TextDisabled("beliefs come from perception only");
    }
}

} // namespace

void worldModelPanel(App& p_app)
{
    if (!ImGui::Begin("World model"))
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
    robotik::Simulation& simulation = *p_app.simulation;
    oracleLine(simulation);
    ImGui::SameLine();
    helpMarker("Truth is the state of the simulated world. Belief is what the robot thinks, "
               "built by perception. Skills only see beliefs. The error is the distance "
               "between both; 'prior' means the robot has not seen the object yet.");

    robotik::WorldModel const& beliefs = simulation.worldModel();
    std::unordered_set<std::string> shown;
    ImGuiTableFlags const flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY;
    // With a planner, the symbolic beliefs share the panel with the objects.
    bool const symbolic = simulation.planner() != nullptr;
    float const objects_height = symbolic ? ImGui::GetContentRegionAvail().y * 0.5f
                                          : -ImGui::GetFrameHeightWithSpacing();
    if (beginTable("objects", 6, flags, ImVec2(0.0f, objects_height)))
    {
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn("Object", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("Truth (m)", ImGuiTableColumnFlags_WidthStretch, 1.4f);
        ImGui::TableSetupColumn("Belief (m)", ImGuiTableColumnFlags_WidthStretch, 1.4f);
        ImGui::TableSetupColumn("Error", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Conf.", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Seen", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();
        p_app.world->each<robotik::ecs::Shape>(
            [&](compages::world::Entity p_entity, robotik::ecs::Shape const& p_object)
            {
                if (p_entity.find<robotik::ecs::Decoration>() != nullptr)
                {
                    return;
                }
                shown.insert(p_object.name);
                robotik::Vector3 const at = simulation.pose(p_object.name).position;
                robotik::WorldObject const* belief = beliefs.find(p_object.name);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::ColorButton("##color",
                                   ImVec4(p_object.color[0], p_object.color[1], p_object.color[2], 1.0f),
                                   ImGuiColorEditFlags_NoTooltip, ImVec2(10.0f, 10.0f));
                ImGui::SameLine();
                ImGui::TextUnformatted(p_object.name.c_str());
                ImGui::TableNextColumn();
                ImGui::Text("%.3f %.3f %.3f", at.x, at.y, at.z);
                ImGui::TableNextColumn();
                if (belief == nullptr)
                {
                    ImGui::TextDisabled("unknown");
                    ImGui::TableNextColumn();
                    ImGui::TableNextColumn();
                    ImGui::TableNextColumn();
                    return;
                }
                // Beliefs are in the robot base frame.
                robotik::Vector3 const believed = simulation.robot().base().pose * belief->position;
                ImGui::Text("%.3f %.3f %.3f", believed.x, believed.y, believed.z);
                ImGui::TableNextColumn();
                double const error = 1000.0 * robotik::norm(believed - at);
                ImGui::TextColored(error < 10.0 ? GREEN : error < 30.0 ? ORANGE : RED, "%.1f mm", error);
                ImGui::TableNextColumn();
                ImGui::Text("%.0f%%", static_cast<double>(belief->confidence) * 100.0);
                ImGui::TableNextColumn();
                if (belief->observed())
                {
                    ImGui::Text("%.2f s", belief->seen.value());
                }
                else
                {
                    ImGui::TextDisabled("prior");
                }
            });
        for (robotik::WorldObject const& belief : beliefs.objects())
        {
            if (shown.contains(belief.name))
            {
                continue;
            }
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(belief.name.c_str());
            ImGui::TableNextColumn();
            ImGui::TextDisabled("not in the world");
            ImGui::TableNextColumn();
            ImGui::Text("%.3f %.3f %.3f", belief.position.x, belief.position.y, belief.position.z);
            ImGui::TableNextColumn();
            ImGui::TableNextColumn();
            ImGui::Text("%.0f%%", static_cast<double>(belief.confidence) * 100.0);
            ImGui::TableNextColumn();
        }
        endTable();
    }
    if (symbolic)
    {
        ImGui::SeparatorText("Symbolic beliefs");
        ImGui::SameLine();
        helpMarker("The same beliefs as atoms of the PDDL domain, read by the observers. "
                   "The planner plans from them.");
        if (ImGui::BeginChild("symbolic"))
        {
            beliefsView(p_app);
        }
        ImGui::EndChild();
    }
    else
    {
        ImGui::TextDisabled("No planner in this scenario: no symbolic beliefs.");
    }
    ImGui::End();
}

void perceptionPanel(App& p_app)
{
    if (!ImGui::Begin("Perception"))
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
    robotik::Simulation& simulation = *p_app.simulation;
    robotik::Camera const* camera = simulation.camera();
    if (camera == nullptr)
    {
        emptyState("The robot has no camera: nothing to detect.");
        ImGui::End();
        return;
    }
    ImGui::TextDisabled("Detections of %s, frame #%llu", camera->name().c_str(),
                        static_cast<unsigned long long>(camera->frame().sequence));
    if (!simulation.robot().resources().available(camera->name()))
    {
        ImGui::SameLine();
        badge("CAMERA FAULT", RED);
        ImGui::TextDisabled("The mission may go on with what the world model already knows.");
    }
    robotik::Detections const& detections = simulation.perception().detections();
    if (detections.items.empty())
    {
        ImGui::TextDisabled("Nothing detected in the last frame.");
        ImGui::End();
        return;
    }
    ImGuiTableFlags const flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY;
    if (beginTable("detections", 5, flags))
    {
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn("Label");
        ImGui::TableSetupColumn("Id", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Confidence", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Box (px)");
        ImGui::TableSetupColumn("3D (optical, m)");
        ImGui::TableHeadersRow();
        for (robotik::Detection const& item : detections.items)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(item.label.c_str());
            ImGui::TableNextColumn();
            if (item.id >= 0)
            {
                ImGui::Text("%d", item.id);
            }
            else
            {
                ImGui::TextDisabled("-");
            }
            ImGui::TableNextColumn();
            ImGui::Text("%.0f%%", static_cast<double>(item.confidence) * 100.0);
            ImGui::TableNextColumn();
            ImGui::Text("%d,%d - %d,%d", item.box[0], item.box[1], item.box[2], item.box[3]);
            ImGui::TableNextColumn();
            if (item.position)
            {
                ImGui::Text("%.3f %.3f %.3f", item.position->x, item.position->y, item.position->z);
            }
            else
            {
                ImGui::TextDisabled("-");
            }
        }
        endTable();
    }
    ImGui::End();
}
