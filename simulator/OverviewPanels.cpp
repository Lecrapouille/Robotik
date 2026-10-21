// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.
#include "Ui.hpp"

#include "Robotik/Common/Config.hpp"
#include "Robotik/Package/Scenario/FileRole.hpp"
#include "Robotik/Package/Scenario/Scenario.hpp"

#include <imgui.h>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <vector>

/// Display name of an entity type.
static char const* typeOf(robotik::EntitySpec::Type p_type)
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

/// Display name of a sensor type.
static char const* typeOf(robotik::SensorSpec::Type p_type)
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

/// Display name of an actuator type.
static char const* typeOf(robotik::ActuatorSpec::Type p_type)
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

/// Id of the entity this one is attached to, without the link.
static std::string parentOf(robotik::EntitySpec const& p_entity)
{
    if (!p_entity.attach)
    {
        return {};
    }
    std::string_view const parent = p_entity.attach->parent;
    return std::string(parent.substr(0, parent.find('.')));
}

/// Forward declaration: one entity and the devices hanging from it.
static void entityRow(App& p_app,
                      robotik::WorldDescription const& p_world,
                      robotik::EntitySpec const& p_entity);

/// Rows of the entities attached to this parent.
static void children(App& p_app,
                     robotik::WorldDescription const& p_world,
                     std::string const& p_parent)
{
    for (robotik::EntitySpec const& entity : p_world.entities)
    {
        if (parentOf(entity) == p_parent && entity.id != p_parent)
        {
            entityRow(p_app, p_world, entity);
        }
    }
}

/// Physics cell of an entity or an actuator: its level, then what the
/// physics backend makes of it. A level set in physics.overrides is bright,
/// the level of the scenario is grey.
static void physicsCell(App const& p_app,
                        std::string const& p_id,
                        char const* p_engine)
{
    robotik::PhysicsSpec const& physics =
        p_app.simulation->scenario().physics;
    bool const own = physics.overrides.contains(p_id);
    char const* const level = robotik::toString(fidelityOf(physics, p_id));
    if (own)
    {
        ImGui::TextUnformatted(level);
    }
    else
    {
        ImGui::TextDisabled("%s", level);
    }
    ImGui::SetItemTooltip(own ? "Set for %s in physics.overrides."
                              : "Level of the scenario (physics.fidelity).",
                          p_id.c_str());
    if (p_engine != nullptr)
    {
        ImGui::SameLine();
        ImGui::TextDisabled("· %s", p_engine);
    }
}

/// What the physics backend makes of an entity, read from the description
/// like the cell does when it builds the physics.
static char const* engineOf(App const& p_app,
                            robotik::EntitySpec const& p_entity)
{
    switch (p_entity.type)
    {
        case robotik::EntitySpec::Type::Robot:
            return "joints";
        case robotik::EntitySpec::Type::Tool:
            return "on the flange";
        case robotik::EntitySpec::Type::Body:
            return p_entity.physics == robotik::EntitySpec::Physics::Free
                       ? "free body"
                       : "fixed";
        case robotik::EntitySpec::Type::Prop:
            if (!p_app.simulation->cell().rigidProp(p_entity))
            {
                return "moved by the grasp";
            }
            return p_entity.shape->type == robotik::ShapeSpec::Type::Box
                       ? "free body"
                       : "static walls";
    }
    return "";
}

/// One childless row: name, type and a detail.
static void leaf(char const* p_name, char const* p_type, char const* p_detail)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TreeNodeEx(
        p_name,
        ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen |
            ImGuiTreeNodeFlags_Bullet | ImGuiTreeNodeFlags_SpanFullWidth);
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", p_type);
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(p_detail);
    ImGui::TableNextColumn();
    ImGui::TableNextColumn();
}

/// One entity row and the sensors, actuators and children under it.
static void entityRow(App& p_app,
                      robotik::WorldDescription const& p_world,
                      robotik::EntitySpec const& p_entity)
{
    robotik::Simulation const& simulation = *p_app.simulation;
    bool has_children =
        !p_entity.sensors.empty() || !p_entity.actuators.empty();
    for (robotik::EntitySpec const& other : p_world.entities)
    {
        has_children = has_children || (parentOf(other) == p_entity.id &&
                                        other.id != p_entity.id);
    }
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGuiTreeNodeFlags flags =
        ImGuiTreeNodeFlags_SpanFullWidth | ImGuiTreeNodeFlags_DefaultOpen;
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
                           ImVec4(p_entity.shape->color[0],
                                  p_entity.shape->color[1],
                                  p_entity.shape->color[2],
                                  1.0F),
                           ImGuiColorEditFlags_NoTooltip,
                           ImVec2(10.0F, 10.0F));
        ImGui::SameLine();
        ImGui::Text("%s %.2fx%.2fx%.2f m",
                    p_entity.shape->type == robotik::ShapeSpec::Type::Container
                        ? "container"
                        : "box",
                    size.x,
                    size.y,
                    size.z);
    }
    ImGui::TableNextColumn();
    physicsCell(p_app, p_entity.id, engineOf(p_app, p_entity));
    ImGui::TableNextColumn();
    if (p_entity.type != robotik::EntitySpec::Type::Robot)
    {
        try
        {
            robotik::Vector3 const at = simulation.cell().pose(p_entity.id).position;
            ImGui::Text("%.2f %.2f %.2f", at.x, at.y, at.z);
        }
        catch (std::exception const&)
        {
            ImGui::TextDisabled(p_entity.type == robotik::EntitySpec::Type::Tool
                                    ? "stored"
                                    : "-");
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
        if (actuator.type == robotik::ActuatorSpec::Type::Vacuum)
        {
            // The level of a gripper says how it holds: the cell welds the
            // object at rigid, moves it with the tool at abstract.
            ImGui::TableSetColumnIndex(3);
            bool const rigid =
                fidelityOf(p_app.simulation->scenario().physics,
                           actuator.name) != robotik::Fidelity::Abstract;
            physicsCell(p_app, actuator.name, rigid ? "welds" : "carries");
        }
    }
    children(p_app, p_world, p_entity.id);
    ImGui::TreePop();
}

std::string readShown(std::filesystem::path const& p_path)
{
    constexpr std::uintmax_t LIMIT = 2U * 1024U * 1024U;
    std::error_code error;
    std::uintmax_t const size = std::filesystem::file_size(p_path, error);
    if (error)
    {
        return "# Cannot read this file: " + error.message();
    }
    if (size > LIMIT)
    {
        return "# " + std::to_string(size / (1024U * 1024U)) +
               " MB: too large to show here.";
    }
    std::ifstream file(p_path, std::ios::binary);
    std::string text((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
    if (text.find('\0') != std::string::npos)
    {
        return "# Binary file, " + std::to_string(size) + " bytes.";
    }
    return text;
}

void worldView(App& p_app)
{
    if (!p_app.simulation)
    {
        emptyState("No scenario loaded.");
        return;
    }
    robotik::WorldDescription const& world = p_app.simulation->scenario().world;
    robotik::PhysicsSpec const& physics = p_app.simulation->scenario().physics;
    ImGui::TextDisabled("%s  ·  %zu entities  ·  %s  ·  physics %s, %zu "
                        "override%s",
                        world.id.c_str(),
                        world.entities.size(),
                        world.floor ? "physical floor" : "no physical floor",
                        robotik::toString(physics.fidelity),
                        physics.overrides.size(),
                        physics.overrides.size() == 1U ? "" : "s");
    ImGui::SameLine();
    helpMarker(
        "Physics: the level of each entity, then what the physics engine "
        "makes of it.\n\n"
        "abstract: a held object follows the tool, a released one is put "
        "down where it belongs. No contact, no fall.\n"
        "rigid: objects are bodies of the engine; a grasp welds the object "
        "to the tool, a release lets it fall.\n"
        "contact: the tool holds by force (suction, friction). Not built "
        "yet.\n\n"
        "Bright: set in physics.overrides of the scenario. Grey: the level "
        "of the scenario. A rigid gripper makes the boxes it may take free "
        "bodies, even at the abstract level.");
    ImGuiTableFlags const flags =
        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
        ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY;
    if (beginTable("entities", 5, flags))
    {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn(
            "Entity", ImGuiTableColumnFlags_WidthStretch, 1.5F);
        ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn(
            "Model", ImGuiTableColumnFlags_WidthStretch, 1.5F);
        ImGui::TableSetupColumn(
            "Physics", ImGuiTableColumnFlags_WidthStretch, 1.3F);
        ImGui::TableSetupColumn(
            "Position (m)", ImGuiTableColumnFlags_WidthStretch, 1.3F);
        ImGui::TableHeadersRow();
        for (robotik::EntitySpec const& entity : world.entities)
        {
            std::string const parent = parentOf(entity);
            if (parent.empty() || robotik::findEntity(world, parent) == nullptr)
            {
                entityRow(p_app, world, entity);
            }
        }
        endTable();
    }
}

/// True when both paths name the same file.
static bool same(std::filesystem::path const& p_a,
                 std::filesystem::path const& p_b)
{
    std::error_code error;
    return !p_a.empty() && !p_b.empty() &&
           std::filesystem::equivalent(p_a, p_b, error);
}

/// Kind of a file: from its double extension (FileRole.hpp), else
/// from the first top-level key of a YAML file.
static FileTab kindOf(std::filesystem::path const& p_path,
                      std::string const& p_text)
{
    switch (robotik::fileRole(p_path))
    {
        case robotik::FileRole::World:
            return FileTab::World;
        case robotik::FileRole::Scenario:
        case robotik::FileRole::Rl:
            return FileTab::Scenario;
        case robotik::FileRole::Tree:
            return FileTab::Tree;
        case robotik::FileRole::Experiment:
            return FileTab::Experiment;
        case robotik::FileRole::Domain:
            return FileTab::Planner;
        case robotik::FileRole::Trajectory:
        case robotik::FileRole::Grafcet:
        case robotik::FileRole::Workflow:
            return FileTab::Other;
        case robotik::FileRole::Unknown:
            break;
    }
    std::string const name = p_path.filename().string();
    if (!name.ends_with(".yml") && !name.ends_with(".yaml"))
    {
        return FileTab::Other;
    }
    std::string_view rest(p_text);
    while (!rest.empty())
    {
        std::size_t const end = rest.find('\n');
        std::string_view const line = rest.substr(0, end);
        rest = end == std::string_view::npos ? std::string_view{}
                                             : rest.substr(end + 1U);
        if (line.empty() || line.front() == '#' || line.front() == ' ')
        {
            continue;
        }
        if (line.starts_with("world:"))
        {
            return FileTab::World;
        }
        if (line.starts_with("scenario:"))
        {
            return FileTab::Scenario;
        }
        if (line.starts_with("experiment:"))
        {
            return FileTab::Experiment;
        }
        if (line.starts_with("BehaviorTree:"))
        {
            return FileTab::Tree;
        }
        break;
    }
    return FileTab::Other;
}

/// The file a tab shows: the one chosen in Files, else the loaded one.
static ShownFile shownIn(App const& p_app, FileTab p_tab)
{
    ShownFile const& chosen =
        p_app.ui.shown_files[static_cast<std::size_t>(p_tab)];
    if (!chosen.path.empty())
    {
        return chosen;
    }
    switch (p_tab)
    {
        case FileTab::Scenario:
            return { p_app.scenario_path, p_app.scenario_text };
        case FileTab::World:
            return { p_app.world_path, p_app.world_text };
        case FileTab::Tree:
            if (p_app.simulation)
            {
                return { p_app.simulation->scenario().controller.behavior_tree,
                         p_app.tree_text };
            }
            return {};
        case FileTab::Planner:
        case FileTab::Experiment:
        case FileTab::Asset:
        case FileTab::Other:
            return {};
    }
    return {};
}

/// Planner tab: not one file but the texts of the planner of a scenario
/// that plans, or a domain chosen in Files.
static void plannerTab(App& p_app)
{
    ShownFile& chosen =
        p_app.ui.shown_files[static_cast<std::size_t>(FileTab::Planner)];
    bool const plans =
        p_app.simulation && p_app.simulation->cell().planner() != nullptr;
    ImGuiTabItemFlags flags = 0;
    if (p_app.ui.select_tab == FileTab::Planner)
    {
        flags |= ImGuiTabItemFlags_SetSelected;
        p_app.ui.select_tab.reset();
    }
    if (ImGui::BeginTabItem("Planner###Planner", nullptr, flags))
    {
        if (p_app.ui.active_tab != FileTab::Planner)
        {
            p_app.ui.active_tab = FileTab::Planner;
            p_app.ui.view = FileTab::Planner;
        }
        if (chosen.path.empty() && !plans)
        {
            emptyState("This scenario does not plan.\nClick a .domain.pddl "
                       "in the Plugin panel to read it.");
        }
        else if (chosen.path.empty())
        {
            plannerTexts(p_app);
        }
        else
        {
            ImGui::TextColored(ORANGE,
                               plans ? "Not the domain of the loaded scenario."
                                     : "The loaded scenario does not plan.");
            if (plans)
            {
                ImGui::SameLine();
                if (ImGui::SmallButton("Show the planner"))
                {
                    chosen = {};
                }
            }
            if (!chosen.path.empty())
            {
                textView(chosen.path, chosen.text);
            }
        }
        ImGui::EndTabItem();
    }
}

/// Selects one package file and shows it in the text panel.
static void fileTab(App& p_app, FileTab p_tab, char const* p_label)
{
    std::size_t const index = static_cast<std::size_t>(p_tab);
    ShownFile const shown = shownIn(p_app, p_tab);
    // The Behavior tree tab stays, empty, for a scenario without a tree.
    if (shown.path.empty() && p_tab != FileTab::Tree)
    {
        return;
    }
    bool const loaded = p_app.ui.shown_files[index].path.empty();
    bool keep = true;
    // The Other tab may be closed; the others follow the loaded scenario.
    bool* const closable = p_tab == FileTab::Other ? &keep : nullptr;
    ImGuiTabItemFlags flags = 0;
    if (p_app.ui.select_tab == p_tab)
    {
        flags |= ImGuiTabItemFlags_SetSelected;
        p_app.ui.select_tab.reset();
    }
    // A scenario driven by a learned policy (x.rl.yaml) is titled RL.
    char const* const title =
        p_tab == FileTab::Scenario &&
                robotik::fileRole(shown.path) == robotik::FileRole::Rl
            ? "RL"
            : p_label;
    std::string const label = p_tab == FileTab::Other
                                  ? shown.path.filename().string() + "###other"
                                  : std::string(title) + "###" + p_label;
    if (ImGui::BeginTabItem(label.c_str(), closable, flags))
    {
        // The view panel follows the tab read here.
        if (p_app.ui.active_tab != p_tab)
        {
            p_app.ui.active_tab = p_tab;
            if (p_tab != FileTab::Other)
            {
                p_app.ui.view = p_tab;
            }
        }
        if (!loaded && p_tab != FileTab::Experiment && p_tab != FileTab::Other)
        {
            ImGui::TextColored(ORANGE, "Not the file of the loaded scenario.");
            ImGui::SameLine();
            if (ImGui::SmallButton("Show the loaded one"))
            {
                p_app.ui.shown_files[index] = {};
            }
        }
        if (shown.path.empty())
        {
            emptyState("This scenario has no behavior tree.\nClick a .bt.yaml "
                       "in the Plugin panel to read it.");
        }
        else
        {
            textView(shown.path, shown.text);
        }
        ImGui::EndTabItem();
    }
    if (!keep)
    {
        p_app.ui.shown_files[index] = {};
    }
}

/// Asset tab: the file clicked in the Assets panel.
static void assetTab(App& p_app)
{
    AssetPreview& preview = p_app.ui.asset_preview;
    ImGuiTabItemFlags flags = 0;
    if (p_app.ui.select_tab == FileTab::Asset)
    {
        flags |= ImGuiTabItemFlags_SetSelected;
        p_app.ui.select_tab.reset();
    }
    if (ImGui::BeginTabItem("Asset###Asset", nullptr, flags))
    {
        p_app.ui.active_tab = FileTab::Asset;
        if (preview.path.empty())
        {
            emptyState("No asset shown.\nClick a model or a file in the "
                       "Assets panel.");
        }
        else
        {
            ImGui::SetItemTooltip("%s", preview.path.string().c_str());
            assetView(p_app);
        }
        ImGui::EndTabItem();
    }
}

void selectExperiment(App& p_app,
                      std::filesystem::path const& p_path,
                      bool p_focus)
{
    UiState& ui = p_app.ui;
    ui.experiment_path = p_path;
    ui.experiment.reset();
    ui.experiment_error.clear();
    robotik::Result<robotik::Experiment> loaded = robotik::loadExperiment(p_path);
    if (loaded)
    {
        ui.experiment = loaded.take();
    }
    else
    {
        ui.experiment_error = loaded.error();
    }
    ui.shown_files[static_cast<std::size_t>(FileTab::Experiment)] = {
        p_path, readShown(p_path)
    };
    if (p_focus)
    {
        showView(p_app, FileTab::Experiment, true);
    }
}

/// First scenario of @p_package whose planner reads the domain
/// @p_domain, or null.
static std::filesystem::path const*
scenarioOfDomain(robotik::PluginPackage const& p_package,
                 std::filesystem::path const& p_domain)
{
    for (robotik::PluginScenario const& scenario : p_package.scenarios)
    {
        try
        {
            robotik::Result<robotik::ConfigNode> loaded =
                robotik::ConfigNode::load(scenario.file);
            if (!loaded)
            {
                continue;
            }
            robotik::ConfigNode const node = loaded.take();
            std::string const domain = node.at("scenario")
                                           .at("controller")
                                           .at("planner")
                                           .text("domain");
            if (!domain.empty() && same(p_package.root / domain, p_domain))
            {
                return &scenario.file;
            }
        }
        catch (std::exception const&)
        {
            // A broken scenario reads no domain; the Demos menu says why.
        }
    }
    return nullptr;
}

void showFile(App& p_app, std::filesystem::path const& p_path)
{
    robotik::PluginPackage const* package =
        p_app.plugins.catalog().matchScenario(p_app.scenario_path).package;
    if (package != nullptr)
    {
        // A scenario of the package: load it, as the Demos menu does.
        for (robotik::PluginScenario const& scenario : package->scenarios)
        {
            if (same(scenario.file, p_path))
            {
                if (!same(scenario.file, p_app.scenario_path))
                {
                    requestScenario(p_app, scenario.file);
                }
                p_app.ui.shown_files[static_cast<std::size_t>(
                    FileTab::Scenario)] = {};
                showView(p_app, FileTab::Scenario, true);
                return;
            }
        }
    }
    // The domain of the running planner: the Planner tab shows it with the
    // problems and plans made from it.
    if (p_app.simulation && p_app.simulation->cell().planner() != nullptr &&
        same(p_app.simulation->cell().planner()->spec().domain, p_path))
    {
        p_app.ui.shown_files[static_cast<std::size_t>(FileTab::Planner)] = {};
        showView(p_app, FileTab::Planner, true);
        return;
    }
    // The domain of another scenario of the package: load that scenario, the
    // Planner tab then shows the domain with its plans.
    if (package != nullptr &&
        robotik::fileRole(p_path) == robotik::FileRole::Domain)
    {
        if (std::filesystem::path const* scenario =
                scenarioOfDomain(*package, p_path))
        {
            requestScenario(p_app, *scenario);
            p_app.ui
                .shown_files[static_cast<std::size_t>(FileTab::Scenario)] = {};
            p_app.ui
                .shown_files[static_cast<std::size_t>(FileTab::Planner)] = {};
            showView(p_app, FileTab::Planner, true);
            return;
        }
    }
    std::string text = readShown(p_path);
    FileTab const tab = kindOf(p_path, text);
    if (tab == FileTab::Experiment)
    {
        selectExperiment(p_app, p_path, true);
        return;
    }
    ShownFile& shown = p_app.ui.shown_files[static_cast<std::size_t>(tab)];
    shown = {};
    // The file the scenario loaded is shown as such, not as a choice.
    if (tab == FileTab::Other || !same(shownIn(p_app, tab).path, p_path))
    {
        shown = { p_path, std::move(text) };
    }
    showView(p_app, tab, true);
}

void filesPanel(App& p_app)
{
    if (!ImGui::Begin("Files"))
    {
        ImGui::End();
        return;
    }
    UiState& ui = p_app.ui;
    if (ui.shown_for != p_app.scenario_path)
    {
        ui.shown_for = p_app.scenario_path;
        for (FileTab const tab : { FileTab::Scenario,
                                   FileTab::World,
                                   FileTab::Tree,
                                   FileTab::Planner })
        {
            ui.shown_files[static_cast<std::size_t>(tab)] = {};
        }
    }
    if (ImGui::BeginTabBar("files", ImGuiTabBarFlags_FittingPolicyScroll))
    {
        fileTab(p_app, FileTab::Scenario, "Scenario");
        fileTab(p_app, FileTab::World, "World");
        fileTab(p_app, FileTab::Tree, "Behavior tree");
        plannerTab(p_app);
        fileTab(p_app, FileTab::Experiment, "Experiment");
        assetTab(p_app);
        fileTab(p_app, FileTab::Other, "File");
        ImGui::EndTabBar();
    }
    // A tab that has nothing to show is not waited for.
    ui.select_tab.reset();
    ImGui::End();
}

void assertionsView(App& p_app)
{
    if (!p_app.simulation)
    {
        emptyState("No scenario loaded.");
        return;
    }
    robotik::Simulation const& simulation = *p_app.simulation;
    std::size_t passed = 0;
    std::size_t failed = 0;
    for (robotik::Check const& check : simulation.checks())
    {
        passed += check.verdict == robotik::Check::Verdict::Passed ? 1U : 0U;
        failed += check.verdict == robotik::Check::Verdict::Failed ? 1U : 0U;
    }
    std::size_t const pending = simulation.checks().size() - passed - failed;
    ImGui::TextColored(GREEN, "%zu passed", passed);
    ImGui::SameLine();
    ImGui::TextColored(failed > 0U ? RED : GREY, "%zu failed", failed);
    ImGui::SameLine();
    ImGui::TextColored(GREY, "%zu pending", pending);
    ImGui::SameLine();
    ImGui::TextDisabled("   run: %s   contacts seen: %d",
                        robotik::toString(simulation.outcome()),
                        simulation.cell().maxContacts());
    ImGui::SameLine();
    helpMarker("An assertion is a condition judged at a moment. A failed one "
               "keeps its value, "
               "its bound and its time; it does not stop the run unless it "
               "says stop: true. "
               "Pending rows show the live value: green when the condition "
               "holds now.");

    ImGuiTableFlags const flags =
        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
        ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY;
    if (beginTable("assertions", 6, flags))
    {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn(
            "Assertion", ImGuiTableColumnFlags_WidthStretch, 2.0F);
        ImGui::TableSetupColumn("When", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn(
            "Value", ImGuiTableColumnFlags_WidthStretch, 0.7F);
        ImGui::TableSetupColumn(
            "Expected", ImGuiTableColumnFlags_WidthStretch, 0.7F);
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
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1,
                                           IM_COL32(120, 30, 30, 70));
                    break;
                case robotik::Check::Verdict::Pending:
                    badge(check.holds ? "holds" : "wait",
                          check.holds ? GREEN : GREY);
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
        endTable();
    }
}

void assertionsPanel(App& p_app, char const* p_key)
{
    std::string const title = panelTitle("Assertions", p_key);
    if (ImGui::Begin(title.c_str()))
    {
        assertionsView(p_app);
    }
    ImGui::End();
}
