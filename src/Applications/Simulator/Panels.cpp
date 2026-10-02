#include "App.hpp"

#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/ECS/ActuatorComponents.hpp"
#include "Robotik/ECS/Queries.hpp"
#include "Robotik/Runtime/RobotRuntime.hpp"
#include "Robotik/Systems/GraspSystem.hpp"

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
static char const* const ROBOT = "Robot";
static char const* const SCENARIO = "Scenario";

static ImVec4 const GREY(0.55f, 0.55f, 0.58f, 1.0f);
static ImVec4 const ORANGE(1.00f, 0.70f, 0.20f, 1.0f);
static ImVec4 const GREEN(0.35f, 0.85f, 0.40f, 1.0f);
static ImVec4 const RED(0.95f, 0.35f, 0.35f, 1.0f);

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
static ImVec4 colorOf(robotik::Status p_status)
{
    switch (p_status)
    {
        case robotik::Status::IDLE:
            return GREY;
        case robotik::Status::RUNNING:
            return ORANGE;
        case robotik::Status::SUCCESS:
            return GREEN;
        case robotik::Status::FAILURE:
            return RED;
    }
    return GREY;
}

//------------------------------------------------------------------------------
static char const* textOf(robotik::Status p_status)
{
    switch (p_status)
    {
        case robotik::Status::IDLE:
            return "idle";
        case robotik::Status::RUNNING:
            return "running";
        case robotik::Status::SUCCESS:
            return "success";
        case robotik::Status::FAILURE:
            return "failure";
    }
    return "idle";
}

//------------------------------------------------------------------------------
//! @brief Layout the docking nodes.
//! @param p_dock The dock ID.
//------------------------------------------------------------------------------
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
        center, ImGuiDir_Down, 0.28f, nullptr, &center);
    ImGuiID right_bottom = 0;
    ImGuiID const right_top = ImGui::DockBuilderSplitNode(
        right, ImGuiDir_Up, 0.45f, nullptr, &right_bottom);
    ImGui::DockBuilderDockWindow(SCENARIO, left);
    ImGui::DockBuilderDockWindow(ROBOT, left);
    ImGui::DockBuilderDockWindow(WORLD, center);
    ImGui::DockBuilderDockWindow(SKILLS, bottom);
    ImGui::DockBuilderDockWindow(CAMERA, right_top);
    ImGui::DockBuilderDockWindow(TREE, right_bottom);
    ImGui::DockBuilderFinish(p_dock);
}

//------------------------------------------------------------------------------
//! @brief Draw the toolbar.
//! @param p_app The application.
//------------------------------------------------------------------------------
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
    ImGui::SetNextItemWidth(320.0f);
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
    if (ImGui::Button("Reset"))
    {
        p_app.load();
    }
    ImGui::SetNextItemWidth(120.0f);
    ImGui::SliderFloat("##speed", &p_app.speed, 0.1f, 3.0f, "speed x%.1f");
    ImGui::Separator();
    if (p_app.simulation)
    {
        ImGui::Text("t = %6.2f s", p_app.simulation->runtime().time().value());
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

//------------------------------------------------------------------------------
//! @brief Draw the world panel.
//! @param p_app The application.
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
//! @brief Draw the camera panel.
//! @param p_app The application.
//------------------------------------------------------------------------------
static void cameraPanel(App const& p_app)
{
    if (!ImGui::Begin(CAMERA))
    {
        ImGui::End();
        return;
    }
    compages::world::Entity camera = p_app.simulation
                                         ? p_app.simulation->camera()
                                         : compages::world::Entity{};
    if (!camera || p_app.robot_view.width == 0)
    {
        ImGui::TextDisabled("The scenario mounts no camera.");
        ImGui::End();
        return;
    }
    float const avail = ImGui::GetContentRegionAvail().x;
    float const zoom = avail / static_cast<float>(p_app.robot_view.width);
    ImVec2 const size(avail,
                      static_cast<float>(p_app.robot_view.height) * zoom);
    ImVec2 const at = ImGui::GetCursorScreenPos();
    ImGui::Image(ImTextureRef(static_cast<ImTextureID>(
                     p_app.robot_view.color.nativeId())),
                 size,
                 ImVec2(0.0f, 1.0f),
                 ImVec2(1.0f, 0.0f));

    auto const& detected = camera.get<robotik::ecs::DetectedObjects>();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    for (auto const& item : detected.items)
    {
        ImVec2 const top(at.x + static_cast<float>(item.x0) * zoom,
                         at.y + static_cast<float>(item.y0) * zoom);
        ImVec2 const end(at.x + static_cast<float>(item.x1 + 1) * zoom,
                         at.y + static_cast<float>(item.y1 + 1) * zoom);
        draw->AddRect(top, end, IM_COL32(80, 255, 120, 255), 0.0f, 0, 2.0f);
        draw->AddText(ImVec2(top.x, top.y - 16.0f),
                      IM_COL32(80, 255, 120, 255),
                      item.label.c_str());
    }

    auto const& sensor = camera.get<robotik::ecs::CameraSensor>();
    ImGui::Text("%ux%u  fov %.0f deg  frame %llu",
                sensor.width,
                sensor.height,
                static_cast<double>(sensor.fov_degrees),
                static_cast<unsigned long long>(detected.frame));
    if (ImGui::BeginTable(
            "detections", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Detection");
        ImGui::TableSetupColumn("Fill");
        ImGui::TableSetupColumn("Box (px)");
        ImGui::TableHeadersRow();
        for (auto const& item : detected.items)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(item.label.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%.0f %%",
                        static_cast<double>(item.confidence) * 100.0);
            ImGui::TableNextColumn();
            ImGui::Text("%d,%d - %d,%d", item.x0, item.y0, item.x1, item.y1);
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

//------------------------------------------------------------------------------
//! @brief Draw a tree node.
//! @param p_node The node.
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
//! @brief Draw the tree panel.
//! @param p_app The application.
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
//! @brief Draw the skills panel.
//! @param p_app The application.
//------------------------------------------------------------------------------
static void skillsPanel(App const& p_app)
{
    if (!ImGui::Begin(SKILLS) || !p_app.simulation)
    {
        ImGui::End();
        return;
    }
    auto const& entries = p_app.simulation->trace().entries;
    double const now = p_app.simulation->runtime().time().value();
    double const span = std::max(now, 1.0);

    // Timeline: one bar per skill run, on a common time axis.
    ImVec2 const at = ImGui::GetCursorScreenPos();
    float const width = ImGui::GetContentRegionAvail().x;
    float const row = 18.0f;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    for (std::size_t i = 0; i < entries.size(); ++i)
    {
        auto const& entry = entries[i];
        float const y = at.y + static_cast<float>(i) * row;
        float const x0 =
            at.x + 160.0f +
            static_cast<float>(entry.start.value() / span) * (width - 170.0f);
        float const x1 =
            at.x + 160.0f +
            static_cast<float>(entry.end.value() / span) * (width - 170.0f);
        draw->AddText(ImVec2(at.x, y),
                      ImGui::GetColorU32(colorOf(entry.status)),
                      entry.name.c_str());
        draw->AddRectFilled(ImVec2(x0, y + 3.0f),
                            ImVec2(std::max(x1, x0 + 3.0f), y + row - 3.0f),
                            ImGui::GetColorU32(colorOf(entry.status)),
                            3.0f);
    }
    ImGui::Dummy(
        ImVec2(width, static_cast<float>(entries.size()) * row + 4.0f));

    if (ImGui::BeginTable("skills",
                          4,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
                              ImGuiTableFlags_ScrollY))
    {
        ImGui::TableSetupColumn("Skill");
        ImGui::TableSetupColumn("Status");
        ImGui::TableSetupColumn("Start (s)");
        ImGui::TableSetupColumn("Duration (s)");
        ImGui::TableHeadersRow();
        for (auto const& entry : entries)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(entry.name.c_str());
            ImGui::TableNextColumn();
            ImGui::TextColored(
                colorOf(entry.status), "%s", textOf(entry.status));
            ImGui::TableNextColumn();
            ImGui::Text("%.2f", entry.start.value());
            ImGui::TableNextColumn();
            ImGui::Text("%.2f", (entry.end - entry.start).value());
        }
        ImGui::EndTable();
    }
    ImGui::End();
}

//------------------------------------------------------------------------------
static void drawJointTableRow(compages::world::Entity,
                              robotik::ecs::Joint const& p_joint,
                              robotik::ecs::JointState const& p_state,
                              robotik::ecs::JointCommand const& p_command,
                              robotik::ecs::JointLimits const& p_limits,
                              robotik::ecs::ActuatorCommand const& p_output)
{
    double const degrees = 180.0 / std::numbers::pi;
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(p_joint.name.c_str());
    ImGui::TableNextColumn();
    double const lower = robotik::ecs::limitLowerSi(p_limits);
    double const upper = robotik::ecs::limitUpperSi(p_limits);
    double const position = robotik::ecs::positionSi(p_state);
    double const command = robotik::ecs::commandPositionSi(p_command);
    double const range = upper - lower;
    float const ratio =
        range > 0.0 ? static_cast<float>((position - lower) / range) : 0.5f;
    char const* label = nullptr;
    char const* label_end = nullptr;
    if (p_joint.mechanism == robotik::ecs::JointMechanism::Revolute)
    {
        ImFormatStringToTempBuffer(
            &label, &label_end, "%.1f deg", position * degrees);
        ImGui::ProgressBar(ratio, ImVec2(-1.0f, 0.0f), label);
        ImGui::TableNextColumn();
        ImGui::Text("%.1f", command * degrees);
    }
    else
    {
        ImFormatStringToTempBuffer(&label, &label_end, "%.3f m", position);
        ImGui::ProgressBar(ratio, ImVec2(-1.0f, 0.0f), label);
        ImGui::TableNextColumn();
        ImGui::Text("%.3f", command);
    }
    ImGui::TableNextColumn();
    double const max_effort = robotik::ecs::limitMaxEffortSi(p_limits);
    bool const saturated =
        max_effort > 0.0 && std::abs(p_output.effort) >= max_effort;
    ImGui::TextColored(saturated ? RED
                                 : ImGui::GetStyle().Colors[ImGuiCol_Text],
                       "%.1f",
                       p_output.effort);
}

//------------------------------------------------------------------------------
static void drawSceneObjectTableRow(compages::world::Entity p_entity,
                                    robotik::ecs::SceneObject& p_object)
{
    auto const at = p_entity.position();
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::ColorButton(
        "##color",
        ImVec4(p_object.color[0], p_object.color[1], p_object.color[2], 1.0f),
        ImGuiColorEditFlags_NoTooltip,
        ImVec2(12.0f, 12.0f));
    ImGui::SameLine();
    ImGui::TextUnformatted(p_object.name.c_str());
    ImGui::TableNextColumn();
    ImGui::Text("%.3f %.3f %.3f",
                static_cast<double>(at.x),
                static_cast<double>(at.y),
                static_cast<double>(at.z));
}

//------------------------------------------------------------------------------
//! @brief Draw the robot panel.
//! @param p_app The application.
//------------------------------------------------------------------------------
static void robotPanel(App const& p_app)
{
    if (!ImGui::Begin(ROBOT) || !p_app.simulation)
    {
        ImGui::End();
        return;
    }
    compages::world::World& world = *p_app.world;
    robotik::PinocchioBackend const& kinematics =
        p_app.simulation->runtime().kinematics();

    // Draw the joints table
    ImGui::SeparatorText("Joints");
    if (ImGui::BeginTable(
            "joints", 4, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Joint");
        ImGui::TableSetupColumn("Position", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Goal");
        ImGui::TableSetupColumn("Torque");
        ImGui::TableHeadersRow();
        world.each<robotik::ecs::Joint,
                   robotik::ecs::JointState,
                   robotik::ecs::JointCommand,
                   robotik::ecs::JointLimits,
                   robotik::ecs::ActuatorCommand>(drawJointTableRow);
        ImGui::EndTable();
    }

    // Draw the tool table
    compages::world::Entity tool = robotik::findTool(world);
    if (tool)
    {
        ImGui::SeparatorText("Tool");
        robotik::Pose const flange =
            kinematics.framePose(tool.get<robotik::ecs::EndEffector>().name);
        auto const tip = robotik::toolTip(world, kinematics);
        ImGui::Text("Flange %s",
                    tool.get<robotik::ecs::EndEffector>().name.c_str());
        ImGui::Text("  xyz  %.3f %.3f %.3f m", flange.px, flange.py, flange.pz);
        ImGui::Text("  quat %.2f %.2f %.2f %.2f",
                    flange.qw,
                    flange.qx,
                    flange.qy,
                    flange.qz);
        ImGui::Text("Suction cup  %.3f %.3f %.3f m", tip[0], tip[1], tip[2]);
        auto const& gripper = tool.get<robotik::ecs::VacuumGripper>();
        if (world.alive(gripper.held))
        {
            ImGui::TextColored(
                ORANGE,
                "Holding %s",
                compages::world::Entity(world, gripper.held).name().c_str());
        }
        else
        {
            ImGui::TextDisabled("Gripper empty");
        }
    }
    ImGui::End();
}

//------------------------------------------------------------------------------
//! @brief Draw the scenario panel.
//! @param p_app The application.
//------------------------------------------------------------------------------
static void scenarioPanel(App const& p_app)
{
    if (!ImGui::Begin(SCENARIO) || !p_app.simulation)
    {
        ImGui::End();
        return;
    }

    // Draw the scenario name and robot model
    robotik::Scenario const& scenario = p_app.simulation->scenario();
    ImGui::Text("%s", scenario.name.c_str());
    ImGui::TextDisabled("%s", scenario.robot_model.string().c_str());
    ImGui::SeparatorText("Task");
    ImGui::TextWrapped("%s", scenario.task.c_str());

    // Draw the objects table
    ImGui::SeparatorText("Objects");
    if (ImGui::BeginTable(
            "objects", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
    {
        ImGui::TableSetupColumn("Name");
        ImGui::TableSetupColumn("Position (m)");
        ImGui::TableHeadersRow();
        p_app.world->each<robotik::ecs::SceneObject>(drawSceneObjectTableRow);
        ImGui::EndTable();
    }

    // Draw the assertions
    ImGui::SeparatorText("Assertions");
    bool const done = p_app.simulation->finished();
    for (auto const& check : p_app.simulation->checks())
    {
        ImVec4 const color = check.passed ? GREEN : (done ? RED : GREY);
        ImGui::TextColored(color,
                           "%s  %s",
                           check.passed ? "[ok]" : "[--]",
                           check.text.c_str());
    }
    ImGui::TextDisabled("Contacts seen: %d", p_app.simulation->maxContacts());
    ImGui::End();
}

//------------------------------------------------------------------------------
//! @brief Draw the panels.
//! @param p_app The application.
//------------------------------------------------------------------------------
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
    scenarioPanel(p_app);
    robotPanel(p_app);
}
