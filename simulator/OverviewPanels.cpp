// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.
#include "Ui.hpp"

#include "Robotik/Scenario/Scenario.hpp"

#include <imgui.h>

#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

namespace
{

char const* typeOf(robotik::EntitySpec::Type p_type)
{
    switch (p_type)
    {
        case robotik::EntitySpec::Type::Robot:
            return "robot";
        case robotik::EntitySpec::Type::Tool:
            return "tool";
        case robotik::EntitySpec::Type::Body:
            return "body";
        case robotik::EntitySpec::Type::Prop:
            return "prop";
    }
    return "";
}

char const* typeOf(robotik::SensorSpec::Type p_type)
{
    switch (p_type)
    {
        case robotik::SensorSpec::Type::Camera:
            return "camera";
        case robotik::SensorSpec::Type::Imu:
            return "imu";
        case robotik::SensorSpec::Type::Range:
            return "range";
        case robotik::SensorSpec::Type::ForceTorque:
            return "force/torque";
    }
    return "";
}

char const* typeOf(robotik::ActuatorSpec::Type p_type)
{
    switch (p_type)
    {
        case robotik::ActuatorSpec::Type::JointGroup:
            return "joint group";
        case robotik::ActuatorSpec::Type::Motor:
            return "motor";
        case robotik::ActuatorSpec::Type::Vacuum:
            return "vacuum";
    }
    return "";
}

char const* oracleOf(robotik::OraclePolicy p_policy)
{
    switch (p_policy)
    {
        case robotik::OraclePolicy::Never:
            return "never";
        case robotik::OraclePolicy::WhenBlind:
            return "when blind";
        case robotik::OraclePolicy::Always:
            return "always";
    }
    return "";
}

//! @brief Two-column "key  value" rows.
void field(char const* p_key, char const* p_format, ...) IM_FMTARGS(2);
void field(char const* p_key, char const* p_format, ...)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", p_key);
    ImGui::TableNextColumn();
    va_list args;
    va_start(args, p_format);
    ImGui::TextWrappedV(p_format, args);
    va_end(args);
}

std::string parentOf(robotik::EntitySpec const& p_entity)
{
    if (!p_entity.attach)
    {
        return {};
    }
    std::string_view const parent = p_entity.attach->parent;
    return std::string(parent.substr(0, parent.find('.')));
}

void entityRow(App& p_app, robotik::WorldDescription const& p_world, robotik::EntitySpec const& p_entity);

void children(App& p_app, robotik::WorldDescription const& p_world, std::string const& p_parent)
{
    for (robotik::EntitySpec const& entity : p_world.entities)
    {
        if (parentOf(entity) == p_parent && entity.id != p_parent)
        {
            entityRow(p_app, p_world, entity);
        }
    }
}

void leaf(char const* p_name, char const* p_type, char const* p_detail)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TreeNodeEx(p_name, ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen |
                                  ImGuiTreeNodeFlags_Bullet | ImGuiTreeNodeFlags_SpanFullWidth);
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", p_type);
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(p_detail);
    ImGui::TableNextColumn();
}

void entityRow(App& p_app, robotik::WorldDescription const& p_world, robotik::EntitySpec const& p_entity)
{
    robotik::Simulation const& simulation = *p_app.simulation;
    bool has_children = !p_entity.sensors.empty() || !p_entity.actuators.empty();
    for (robotik::EntitySpec const& other : p_world.entities)
    {
        has_children = has_children || (parentOf(other) == p_entity.id && other.id != p_entity.id);
    }
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_SpanFullWidth | ImGuiTreeNodeFlags_DefaultOpen;
    if (!has_children)
    {
        flags |= ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
    }
    bool const open = ImGui::TreeNodeEx(p_entity.id.c_str(), flags);
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", typeOf(p_entity.type));
    ImGui::TableNextColumn();
    if (!p_entity.model_ref.empty())
    {
        ImGui::TextUnformatted(p_entity.model_ref.c_str());
    }
    else if (p_entity.shape)
    {
        robotik::Vector3 const& size = p_entity.shape->size;
        ImGui::ColorButton("##color",
                           ImVec4(p_entity.shape->color[0], p_entity.shape->color[1],
                                  p_entity.shape->color[2], 1.0f),
                           ImGuiColorEditFlags_NoTooltip, ImVec2(10.0f, 10.0f));
        ImGui::SameLine();
        ImGui::Text("%s %.2fx%.2fx%.2f m",
                    p_entity.shape->type == robotik::ShapeSpec::Type::Container ? "container" : "box",
                    size.x, size.y, size.z);
    }
    ImGui::TableNextColumn();
    if (p_entity.type != robotik::EntitySpec::Type::Robot)
    {
        try
        {
            robotik::Vector3 const at = simulation.pose(p_entity.id).position;
            ImGui::Text("%.2f %.2f %.2f", at.x, at.y, at.z);
        }
        catch (std::exception const&)
        {
            ImGui::TextDisabled(p_entity.type == robotik::EntitySpec::Type::Tool ? "stored" : "-");
        }
    }
    if (!has_children || !open)
    {
        return;
    }
    for (robotik::SensorSpec const& sensor : p_entity.sensors)
    {
        leaf(sensor.name.c_str(), "sensor", typeOf(sensor.type));
    }
    for (robotik::ActuatorSpec const& actuator : p_entity.actuators)
    {
        leaf(actuator.name.c_str(), "actuator", typeOf(actuator.type));
    }
    children(p_app, p_world, p_entity.id);
    ImGui::TreePop();
}

void textFile(char const* p_tab, std::filesystem::path const& p_path, std::string& p_text)
{
    if (p_text.empty() || !ImGui::BeginTabItem(p_tab))
    {
        return;
    }
    ImGui::TextDisabled("%s", p_path.string().c_str());
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy path"))
    {
        ImGui::SetClipboardText(p_path.string().c_str());
    }
    ImGui::InputTextMultiline("##text", p_text.data(), p_text.size() + 1u, ImVec2(-1.0f, -1.0f),
                              ImGuiInputTextFlags_ReadOnly);
    ImGui::EndTabItem();
}

} // namespace

void pluginPanel(App& p_app)
{
    if (!ImGui::Begin("Plugin"))
    {
        ImGui::End();
        return;
    }
    robotik::PluginMatch const match = p_app.plugins.catalog().matchScenario(p_app.scenario_path);
    robotik::PluginPackage const* package = match.package;
    if (package == nullptr)
    {
        emptyState("No plugin loaded.\nChoose a scenario in the Demos menu.");
        ImGui::End();
        return;
    }
    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 1.25f);
    ImGui::TextUnformatted(package->name.c_str());
    ImGui::PopFont();
    ImGui::TextDisabled("%s  ·  v%s  ·  %s", package->id.c_str(), package->version.toString().c_str(),
                        package->category.c_str());
    if (!package->packs.empty())
    {
        std::string packs;
        for (robotik::PackRequirement const& pack : package->packs)
        {
            packs += packs.empty() ? "" : ", ";
            packs += pack.id + " " + pack.version.toString();
        }
        ImGui::TextDisabled("Asset packs: %s", packs.c_str());
    }

    ImGui::SeparatorText("Scenarios");
    for (robotik::PluginScenario const& scenario : package->scenarios)
    {
        bool const current = match.scenario != nullptr && match.scenario->id == scenario.id;
        if (ImGui::Selectable(scenario.name.c_str(), current) && !current)
        {
            requestScenario(p_app, scenario.file);
        }
        ImGui::SetItemTooltip("%s\nClick to load.", scenario.file.string().c_str());
    }

    if (!package->experiments.empty())
    {
        ImGui::SeparatorText("Experiments");
        ImGui::SameLine();
        helpMarker("An experiment runs several variants and seeds without window, and writes a "
                   "report. Copy the command and run it in a terminal.");
        for (robotik::PluginScenario const& experiment : package->experiments)
        {
            ImGui::PushID(experiment.id.c_str());
            ImGui::BulletText("%s", experiment.name.c_str());
            ImGui::SameLine();
            if (ImGui::SmallButton("Copy command"))
            {
                std::string const command = "./build/Robotik-Headless " + experiment.file.string() +
                                            " --report " + experiment.id + ".json";
                ImGui::SetClipboardText(command.c_str());
            }
            ImGui::PopID();
        }
    }

    if (p_app.simulation)
    {
        robotik::Scenario const& scenario = p_app.simulation->scenario();
        ImGui::SeparatorText("This scenario");
        if (!scenario.description.empty())
        {
            ImGui::TextWrapped("%s", scenario.description.c_str());
            ImGui::Spacing();
        }
        if (ImGui::BeginTable("scenario", 2, ImGuiTableFlags_SizingStretchProp))
        {
            ImGui::TableSetupColumn("key", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
            field("id", "%s", scenario.id.c_str());
            field("world", "%s", scenario.world.id.c_str());
            field("robot", "%s", scenario.world.robot().model_ref.c_str());
            field("controller", "%s",
                  scenario.controller.behavior_tree.empty()
                      ? "plugin mission (C++)"
                      : scenario.controller.behavior_tree.filename().string().c_str());
            field("seed", "%llu", static_cast<unsigned long long>(scenario.seed));
            field("timeout", "%.0f s", scenario.timeout.value());
            if (!scenario.stop_when.empty())
            {
                field("stop when", "%s", scenario.stop_when.c_str());
            }
            field("faults", "%zu planned, %zu random", scenario.faults.size(), scenario.random_faults.size());
            field("oracle", "%s%s", oracleOf(scenario.oracle),
                  p_app.simulation->oracle() ? " (active: ground truth feeds the beliefs)" : "");
            ImGui::EndTable();
        }
    }
    ImGui::End();
}

void worldPanel(App& p_app)
{
    if (!ImGui::Begin("World"))
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
    robotik::WorldDescription const& world = p_app.simulation->scenario().world;
    ImGui::TextDisabled("%s  ·  %zu entities  ·  %s", world.id.c_str(), world.entities.size(),
                        world.floor ? "physical floor" : "no physical floor");
    ImGuiTableFlags const flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY;
    if (ImGui::BeginTable("entities", 4, flags))
    {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Entity", ImGuiTableColumnFlags_WidthStretch, 1.5f);
        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Model", ImGuiTableColumnFlags_WidthStretch, 1.5f);
        ImGui::TableSetupColumn("Position (m)", ImGuiTableColumnFlags_WidthStretch, 1.3f);
        ImGui::TableHeadersRow();
        for (robotik::EntitySpec const& entity : world.entities)
        {
            std::string const parent = parentOf(entity);
            if (parent.empty() || world.find(parent) == nullptr)
            {
                entityRow(p_app, world, entity);
            }
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

void filesPanel(App& p_app)
{
    if (!ImGui::Begin("Files"))
    {
        ImGui::End();
        return;
    }
    if (p_app.scenario_text.empty())
    {
        emptyState("No scenario file loaded.");
        ImGui::End();
        return;
    }
    if (ImGui::BeginTabBar("files"))
    {
        textFile("Scenario", p_app.scenario_path, p_app.scenario_text);
        textFile("World", p_app.world_path, p_app.world_text);
        if (p_app.simulation)
        {
            textFile("Behavior tree", p_app.simulation->scenario().controller.behavior_tree, p_app.tree_text);
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}

void assertionsPanel(App& p_app, char const* p_key)
{
    std::string const title = panelTitle("Assertions", p_key);
    if (!ImGui::Begin(title.c_str()))
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
    robotik::Simulation const& simulation = *p_app.simulation;
    std::size_t passed = 0;
    std::size_t failed = 0;
    for (robotik::Check const& check : simulation.checks())
    {
        passed += check.verdict == robotik::Check::Verdict::Passed ? 1u : 0u;
        failed += check.verdict == robotik::Check::Verdict::Failed ? 1u : 0u;
    }
    std::size_t const pending = simulation.checks().size() - passed - failed;
    ImGui::TextColored(GREEN, "%zu passed", passed);
    ImGui::SameLine();
    ImGui::TextColored(failed > 0u ? RED : GREY, "%zu failed", failed);
    ImGui::SameLine();
    ImGui::TextColored(GREY, "%zu pending", pending);
    ImGui::SameLine();
    ImGui::TextDisabled("   run: %s   contacts seen: %d", robotik::toString(simulation.outcome()),
                        simulation.maxContacts());
    ImGui::SameLine();
    helpMarker("An assertion is a condition judged at a moment. A failed one keeps its value, "
               "its bound and its time; it does not stop the run unless it says stop: true. "
               "Pending rows show the live value: green when the condition holds now.");

    ImGuiTableFlags const flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY;
    if (ImGui::BeginTable("assertions", 6, flags))
    {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Assertion", ImGuiTableColumnFlags_WidthStretch, 2.0f);
        ImGui::TableSetupColumn("When", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch, 0.7f);
        ImGui::TableSetupColumn("Expected", ImGuiTableColumnFlags_WidthStretch, 0.7f);
        ImGui::TableSetupColumn("At", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();
        for (robotik::Check const& check : simulation.checks())
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            switch (check.verdict)
            {
                case robotik::Check::Verdict::Passed:
                    badge("PASS", GREEN);
                    break;
                case robotik::Check::Verdict::Failed:
                    badge("FAIL", RED);
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, IM_COL32(120, 30, 30, 70));
                    break;
                case robotik::Check::Verdict::Pending:
                    badge(check.holds ? "holds" : "wait", check.holds ? GREEN : GREY);
                    break;
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(check.text.c_str());
            if (!check.detail.empty())
            {
                ImGui::SetItemTooltip("%s", check.detail.c_str());
            }
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", robotik::toString(check.phase));
            ImGui::TableNextColumn();
            ImGui::Text("%.4g", check.value);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(check.expected.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%.2f s", check.time.value());
        }
        ImGui::EndTable();
    }
    ImGui::End();
}
