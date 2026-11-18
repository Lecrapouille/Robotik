// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file InspectorPanel.cpp
/// The Inspector: everything about the item selected in any panel.
///
/// A skill shows its state and why, the resources it needs and who holds
/// them, its preconditions and its last phases. A resource shows who holds
/// it, who waits for it, who needs it and its faults. A robot shows its
/// skills and its joints. An object shows what the robot believes of it
/// against the truth of the simulator. Every related item is a link that
/// selects it.
/// With nothing selected, it sums up the run: what runs, what waits, what
/// is in fault.
///
/// @par Contract
/// Reads the simulation, never commands it: selecting is not acting.
/// "Pin" keeps one item in the Inspector while the other panels follow the
/// selection.

#include "SkillView.hpp"
#include "Ui.hpp"

#include "Robotik/Robot/Components/Objects.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

/// Phases listed under a skill.
static constexpr std::size_t PHASES_SHOWN = 8U;

void selectItem(App& p_app, SelectionKind p_kind, std::string const& p_name)
{
    p_app.ui.selection = { p_kind, p_name };
}

bool isSelected(App const& p_app,
                SelectionKind p_kind,
                std::string const& p_name)
{
    return p_app.ui.selection.kind == p_kind &&
           p_app.ui.selection.name == p_name;
}

bool itemLink(App& p_app, SelectionKind p_kind, std::string const& p_name)
{
    ImGui::PushID(static_cast<int>(p_kind));
    bool const clicked = ImGui::TextLink(p_name.c_str());
    ImGui::PopID();
    if (clicked)
    {
        selectItem(p_app, p_kind, p_name);
    }
    return clicked;
}

/// Robot a skill drives: its description, else the first robot.
static std::string robotOf(robotik::Cell const& p_cell, robotik::SkillId p_id)
{
    std::string const& robot = p_cell.skills().description(p_id).robot;
    if (!robot.empty() || p_cell.robots().empty())
    {
        return robot;
    }
    return p_cell.robots().front()->id();
}

/// Small grey caption above a title, e.g. "SKILL · left".
static void caption(std::string const& p_text)
{
    ImGui::TextDisabled("%s", p_text.c_str());
}

/// Name of the item, larger than the text.
static void title(std::string const& p_text)
{
    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 1.35F);
    ImGui::TextUnformatted(p_text.c_str());
    ImGui::PopFont();
}

/// One fact: grey name on the left, value after it.
static void fact(char const* p_name, char const* p_format, ...)
{
    ImGui::TextDisabled("%s", p_name);
    ImGui::SameLine(ImGui::GetFontSize() * 8.0F);
    va_list args;
    va_start(args, p_format);
    ImGui::TextV(p_format, args);
    va_end(args);
}

/// Text in @p_color, wrapped to the panel.
static void note(ImVec4 const& p_color, std::string const& p_text)
{
    ImGui::PushStyleColor(ImGuiCol_Text, p_color);
    ImGui::TextWrapped("%s", p_text.c_str());
    ImGui::PopStyleColor();
}

/// A skill link in the color of its state.
static void skillLink(App& p_app, robotik::SkillId p_id)
{
    robotik::SkillScheduler const& skills = p_app.simulation->cell().skills();
    ImGui::PushStyleColor(ImGuiCol_TextLink, ::colorOf(skills.state(p_id)));
    itemLink(p_app, SelectionKind::Skill, skills.name(p_id));
    ImGui::PopStyleColor();
}

/// "free", "held", "shared" or "FAULT", and its color.
static char const* resourceState(robotik::ResourceManager const& p_resources,
                                 robotik::ResourceId p_id,
                                 ImVec4& p_color)
{
    if (!p_resources.available(p_id))
    {
        p_color = RED;
        return "FAULT";
    }
    if (p_resources.owner(p_id) != robotik::NO_OWNER)
    {
        p_color = ORANGE;
        return "held";
    }
    if (p_resources.users(p_id) > 0U)
    {
        p_color = BLUE;
        return "shared";
    }
    p_color = GREY;
    return "free";
}

/// Skills holding @p_id now.
static std::vector<robotik::SkillId>
holders(robotik::SkillScheduler const& p_skills,
        robotik::ResourceManager const& p_resources,
        robotik::ResourceId p_id)
{
    std::vector<robotik::SkillId> found;
    robotik::OwnerId const owner = p_resources.owner(p_id);
    for (robotik::SkillId id = 0; id < p_skills.size(); ++id)
    {
        if (p_skills.state(id) != robotik::SkillState::Running)
        {
            continue;
        }
        for (robotik::ResourceRequirement const& need :
             p_skills.description(id).resources)
        {
            if (need.id == p_id &&
                (owner == id || need.access == robotik::Access::Shared))
            {
                found.push_back(id);
                break;
            }
        }
    }
    return found;
}

// --- Skill ------------------------------------------------------------------

/// Everything about one skill.
static void skillView(App& p_app, robotik::SkillId p_id)
{
    robotik::Simulation const& simulation = *p_app.simulation;
    robotik::Cell const& cell = simulation.cell();
    robotik::SkillScheduler const& skills = cell.skills();
    robotik::ResourceManager const& resources = cell.robot().resources();
    robotik::SkillDescription const& description = skills.description(p_id);
    robotik::SkillState const state = skills.state(p_id);
    double const now = simulation.time().value();
    std::span<robotik::SkillRun const> const trace = skills.trace();

    std::string const robot = robotOf(cell, p_id);
    caption("SKILL");
    title(description.name);
    badge(robotik::toString(state), ::colorOf(state));
    for (std::size_t i = trace.size(); i > 0U; --i)
    {
        robotik::SkillRun const& run = trace[i - 1U];
        if (run.skill == p_id && run.state == state &&
            (state == robotik::SkillState::Running ||
             state == robotik::SkillState::Waiting))
        {
            ImGui::SameLine();
            ImGui::TextDisabled("for %.2f s", now - run.start.value());
            break;
        }
    }
    std::string const why = skillWhy(skills, resources, p_id);
    if (!why.empty())
    {
        note(state == robotik::SkillState::Failed ? RED : ORANGE, why);
    }
    if (!robot.empty())
    {
        ImGui::TextDisabled("Robot");
        ImGui::SameLine(ImGui::GetFontSize() * 8.0F);
        itemLink(p_app, SelectionKind::Robot, robot);
    }

    ImGui::SeparatorText("Resources");
    if (description.resources.empty())
    {
        ImGui::TextDisabled("None.");
    }
    for (robotik::ResourceRequirement const& need : description.resources)
    {
        ImGui::PushID(static_cast<int>(need.id));
        itemLink(p_app, SelectionKind::Resource, resources.name(need.id));
        ImGui::SameLine();
        ImGui::TextDisabled(
            need.access == robotik::Access::Shared ? "shared" : "exclusive");
        ImGui::SameLine();
        ImVec4 color;
        char const* word = resourceState(resources, need.id, color);
        std::vector<robotik::SkillId> const held =
            holders(skills, resources, need.id);
        bool const other = std::ranges::any_of(
            held, [p_id](robotik::SkillId p_h) { return p_h != p_id; });
        if (!resources.available(need.id) || !other)
        {
            ImGui::TextColored(color, "%s", word);
        }
        else
        {
            ImGui::TextColored(color, "held by");
            for (robotik::SkillId const holder : held)
            {
                if (holder != p_id)
                {
                    ImGui::SameLine();
                    skillLink(p_app, holder);
                }
            }
        }
        ImGui::PopID();
    }

    if (!description.preconditions.empty())
    {
        ImGui::SeparatorText("Preconditions");
        robotik::Precondition const* failed = skills.precondition(p_id);
        for (robotik::Precondition const& condition : description.preconditions)
        {
            bool const is_failed = failed == &condition;
            ImGui::TextColored(is_failed ? RED : GREY,
                               is_failed ? "false" : "-");
            ImGui::SameLine();
            ImGui::TextWrapped("%s", condition.text.c_str());
        }
    }

    ImGui::SeparatorText("Settings");
    fact("Priority", "%d", static_cast<int>(description.priority));
    fact("When busy", "%s", description.wait ? "waits" : "fails at once");
    fact("Preemption", "%s", description.cancellable ? "allowed" : "refused");

    ImGui::SeparatorText("Last phases");
    std::vector<std::size_t> phases;
    for (std::size_t i = trace.size(); i > 0U && phases.size() < PHASES_SHOWN;
         --i)
    {
        if (trace[i - 1U].skill == p_id)
        {
            phases.push_back(i - 1U);
        }
    }
    if (phases.empty())
    {
        ImGui::TextDisabled("Never requested since the start.");
        return;
    }
    if (beginTable("phases", 4, ImGuiTableFlags_RowBg))
    {
        ImGui::TableSetupColumn("State");
        ImGui::TableSetupColumn("Start");
        ImGui::TableSetupColumn("Lasted");
        ImGui::TableSetupColumn("Why");
        ImGui::TableHeadersRow();
        for (std::size_t const index : phases)
        {
            robotik::SkillRun const& run = trace[index];
            ImGui::TableNextRow();
            if (index == p_app.ui.timeline_memory.selected)
            {
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1,
                                       ImGui::GetColorU32(ImGuiCol_Header));
            }
            ImGui::TableNextColumn();
            ImGui::TextColored(
                ::colorOf(run.state), "%s", robotik::toString(run.state));
            ImGui::TableNextColumn();
            ImGui::Text("%.2f s", run.start.value());
            ImGui::TableNextColumn();
            ImGui::Text("%.2f s", run.end.value() - run.start.value());
            ImGui::TableNextColumn();
            if (run.reason == robotik::SkillReason::None)
            {
                ImGui::TextDisabled("-");
            }
            else if (run.blocker != robotik::NO_RESOURCE)
            {
                ImGui::Text("%s: %s",
                            robotik::toString(run.reason),
                            resources.name(run.blocker).c_str());
            }
            else
            {
                ImGui::TextUnformatted(robotik::toString(run.reason));
            }
        }
        endTable();
    }
}

// --- Resource ---------------------------------------------------------------

/// Everything about one resource.
static void resourceView(App& p_app, robotik::ResourceId p_id)
{
    robotik::Cell const& cell = p_app.simulation->cell();
    robotik::SkillScheduler const& skills = cell.skills();
    robotik::ResourceManager const& resources = cell.robot().resources();

    caption("RESOURCE");
    title(resources.name(p_id));
    ImVec4 color;
    char const* word = resourceState(resources, p_id, color);
    badge(word, color);

    ImGui::SeparatorText("Held by");
    std::vector<robotik::SkillId> const held = holders(skills, resources, p_id);
    if (held.empty())
    {
        ImGui::TextDisabled(resources.owner(p_id) != robotik::NO_OWNER ||
                                    resources.users(p_id) > 0U
                                ? "Someone outside the skills (plugin, "
                                  "teach pendant)."
                                : "Nobody.");
    }
    for (robotik::SkillId const id : held)
    {
        skillLink(p_app, id);
    }

    std::vector<robotik::SkillId> waiting;
    std::vector<robotik::SkillId> needed;
    for (robotik::SkillId id = 0; id < skills.size(); ++id)
    {
        bool const needs = std::ranges::any_of(
            skills.description(id).resources,
            [p_id](robotik::ResourceRequirement const& p_need)
            { return p_need.id == p_id; });
        if (!needs)
        {
            continue;
        }
        needed.push_back(id);
        if (skills.state(id) == robotik::SkillState::Waiting &&
            skills.blocker(id) == p_id)
        {
            waiting.push_back(id);
        }
    }
    if (!waiting.empty())
    {
        ImGui::SeparatorText("Waited for by");
        for (robotik::SkillId const id : waiting)
        {
            skillLink(p_app, id);
        }
    }
    ImGui::SeparatorText("Needed by");
    if (needed.empty())
    {
        ImGui::TextDisabled("No skill.");
    }
    for (robotik::SkillId const id : needed)
    {
        skillLink(p_app, id);
    }

    ImGui::SeparatorText("Faults");
    bool any = false;
    for (FaultSpan const& span : p_app.ui.timeline_memory.faults)
    {
        if (span.resource != p_id)
        {
            continue;
        }
        any = true;
        if (span.open)
        {
            ImGui::TextColored(RED, "since %.2f s", span.start);
        }
        else
        {
            ImGui::Text("%.2f s to %.2f s", span.start, span.end);
        }
    }
    if (!any)
    {
        ImGui::TextDisabled("None in this run.");
    }
    ImGui::TextDisabled("Inject a fault: Resources panel.");
}

// --- Robot ------------------------------------------------------------------

/// Everything about one robot: its skills, its joints.
static void robotView(App& p_app, robotik::RobotSession const& p_robot)
{
    robotik::Cell const& cell = p_app.simulation->cell();
    robotik::SkillScheduler const& skills = cell.skills();

    caption("ROBOT");
    title(p_robot.id());
    if (p_robot.name() != p_robot.id())
    {
        ImGui::TextDisabled("%s", p_robot.name().c_str());
    }
    if (!p_robot.tool().empty())
    {
        fact("Tool", "%s", p_robot.tool().c_str());
    }
    if (p_app.teach.manual)
    {
        note(ORANGE,
             "Manual mode: the mission is suspended (Robot "
             "workspace, Teach Pendant).");
    }

    ImGui::SeparatorText("Skills");
    bool any = false;
    for (robotik::SkillId id = 0; id < skills.size(); ++id)
    {
        if (robotOf(cell, id) != p_robot.id())
        {
            continue;
        }
        any = true;
        skillLink(p_app, id);
        ImGui::SameLine();
        ImGui::TextDisabled("%s", robotik::toString(skills.state(id)));
    }
    if (!any)
    {
        ImGui::TextDisabled("No skill drives this robot.");
    }

    ImGui::SeparatorText("Joints");
    robotik::JointSet const& joints = p_robot.joints();
    for (robotik::JointId id = 0; id < joints.size(); ++id)
    {
        bool const linear = joints.isPrismatic(id);
        double lower = 0.0;
        double upper = 0.0;
        if (linear)
        {
            auto const limits = joints.limits(joints.prismatic(id));
            lower = limits.lower.value();
            upper = limits.upper.value();
        }
        else
        {
            auto const limits = joints.limits(joints.revolute(id));
            lower = limits.lower.value();
            upper = limits.upper.value();
        }
        double const position = joints.position(id);
        if (!std::isfinite(lower) || !std::isfinite(upper))
        {
            lower = position - std::numbers::pi;
            upper = position + std::numbers::pi;
        }
        bool const commanded = joints.mode(id) == robotik::JointMode::Position;
        char label[64];
        std::snprintf(label,
                      sizeof label,
                      linear ? "%s  %.3f m" : "%s  %.1f°",
                      joints.name(id).c_str(),
                      linear ? position : position * 180.0 / std::numbers::pi);
        ImGui::PushID(static_cast<int>(id));
        rangeBar(position,
                 lower,
                 upper,
                 commanded ? joints.target(id) : position,
                 commanded,
                 label,
                 joints.mode(id) == robotik::JointMode::Disabled ? RED : BLUE);
        ImGui::PopID();
    }
    ImGui::TextDisabled("Move the joints: Robot workspace.");
}

// --- Object -----------------------------------------------------------------

/// Word and color of where a belief comes from.
static char const* sourceOf(robotik::BeliefSource p_source, ImVec4& p_color)
{
    switch (p_source)
    {
        case robotik::BeliefSource::Prior:
            p_color = GREY;
            return "prior";
        case robotik::BeliefSource::Sensed:
            p_color = GREEN;
            return "sensed";
        case robotik::BeliefSource::NoisyTruth:
            p_color = ORANGE;
            return "noisy truth (mock)";
        case robotik::BeliefSource::Truth:
            p_color = ORANGE;
            return "truth (mock)";
    }
    p_color = GREY;
    return "";
}

/// True when @p_name is an object (a shape, not a decoration) of the world.
static bool inWorld(App& p_app, std::string const& p_name)
{
    bool found = false;
    p_app.world->each<robotik::ecs::Shape>(
        [&](compages::world::Entity p_entity,
            robotik::ecs::Shape const& p_shape)
        {
            if (p_shape.name == p_name &&
                p_entity.find<robotik::ecs::Decoration>() == nullptr)
            {
                found = true;
            }
        });
    return found;
}

/// What the robot believes of an object, against the truth.
static void objectView(App& p_app, std::string const& p_name)
{
    robotik::Cell const& cell = p_app.simulation->cell();
    robotik::WorldObject const* belief = cell.worldModel().find(p_name);
    bool const real = inWorld(p_app, p_name);

    caption("OBJECT");
    title(p_name);
    if (belief == nullptr)
    {
        badge("unknown to the robot", GREY);
    }
    else
    {
        ImVec4 color;
        char const* word = sourceOf(belief->source, color);
        badge(word, color);
        if (robotik::observed(*belief))
        {
            ImGui::SameLine();
            ImGui::TextDisabled("seen at %.2f s", belief->seen.value());
        }
    }

    // Both positions in the world frame, and the distance between them.
    robotik::Vector3 truth{};
    bool has_truth = false;
    if (real)
    {
        try
        {
            truth = cell.pose(p_name).position;
            has_truth = true;
        }
        catch (std::invalid_argument const&)
        {
            has_truth = false;
        }
    }
    ImGui::SeparatorText("Belief against truth");
    if (belief != nullptr)
    {
        robotik::Vector3 const believed = cell.beliefFrame() * belief->position;
        fact("Belief", "%.3f %.3f %.3f m", believed.x, believed.y, believed.z);
        if (has_truth)
        {
            fact("Truth", "%.3f %.3f %.3f m", truth.x, truth.y, truth.z);
            double const error = 1000.0 * robotik::norm(believed - truth);
            ImGui::TextDisabled("Error");
            ImGui::SameLine(ImGui::GetFontSize() * 8.0F);
            ImGui::TextColored(error < 10.0   ? GREEN
                               : error < 30.0 ? ORANGE
                                              : RED,
                               "%.1f mm",
                               error);
        }
        else
        {
            fact("Truth", "%s", "not in the world");
        }
        fact("Confidence",
             "%.0f %%",
             static_cast<double>(belief->confidence) * 100.0);
        fact("Observations", "%u", belief->observations);
        if (belief->size.x > 0.0 || belief->size.y > 0.0 ||
            belief->size.z > 0.0)
        {
            fact("Size",
                 "%.3f %.3f %.3f m",
                 belief->size.x,
                 belief->size.y,
                 belief->size.z);
        }
    }
    else if (has_truth)
    {
        fact("Truth", "%.3f %.3f %.3f m", truth.x, truth.y, truth.z);
        ImGui::TextDisabled("The robot has no belief about it.");
    }
    ImGui::Spacing();
    ImGui::TextDisabled("Skills see the belief only; the truth is here to "
                        "compare.");
}

// --- Nothing selected ---------------------------------------------------------

/// The run in a few lines: what runs, what waits, what is in fault.
static void overview(App& p_app)
{
    robotik::Cell const& cell = p_app.simulation->cell();
    robotik::SkillScheduler const& skills = cell.skills();
    robotik::ResourceManager const& resources = cell.robot().resources();
    caption("RUN");
    title(p_app.scenario_title.empty() ? std::string("Run")
                                       : p_app.scenario_title);
    ImGui::TextDisabled("Click a skill, a resource, a robot or an object "
                        "in any panel to see it here.");

    ImGui::SeparatorText("Robots");
    for (auto const& robot : cell.robots())
    {
        itemLink(p_app, SelectionKind::Robot, robot->id());
    }

    std::vector<robotik::SkillId> running;
    std::vector<robotik::SkillId> waiting;
    std::vector<robotik::SkillId> failed;
    for (robotik::SkillId id = 0; id < skills.size(); ++id)
    {
        switch (skills.state(id))
        {
            case robotik::SkillState::Running:
                running.push_back(id);
                break;
            case robotik::SkillState::Waiting:
                waiting.push_back(id);
                break;
            case robotik::SkillState::Failed:
                failed.push_back(id);
                break;
            case robotik::SkillState::Idle:
            case robotik::SkillState::Succeeded:
            case robotik::SkillState::Cancelled:
                break;
        }
    }
    ImGui::SeparatorText("Running");
    if (running.empty())
    {
        ImGui::TextDisabled("Nothing.");
    }
    for (robotik::SkillId const id : running)
    {
        skillLink(p_app, id);
    }
    if (!waiting.empty())
    {
        ImGui::SeparatorText("Waiting");
        for (robotik::SkillId const id : waiting)
        {
            skillLink(p_app, id);
            ImGui::SameLine();
            ImGui::TextDisabled("%s", skillWhy(skills, resources, id).c_str());
        }
    }
    if (!failed.empty())
    {
        ImGui::SeparatorText("Failed (last run)");
        for (robotik::SkillId const id : failed)
        {
            skillLink(p_app, id);
        }
    }
    std::span<robotik::WorldObject const> const objects =
        cell.worldModel().objects();
    if (!objects.empty())
    {
        ImGui::SeparatorText("Objects believed");
        for (robotik::WorldObject const& object : objects)
        {
            ImGui::PushStyleColor(
                ImGuiCol_TextLink,
                robotik::observed(object)
                    ? ImGui::GetStyleColorVec4(ImGuiCol_TextLink)
                    : GREY);
            itemLink(p_app, SelectionKind::Object, object.name);
            ImGui::PopStyleColor();
        }
    }
    bool fault = false;
    for (robotik::ResourceId id = 0; id < resources.size(); ++id)
    {
        if (resources.available(id))
        {
            continue;
        }
        if (!fault)
        {
            ImGui::SeparatorText("Resources in fault");
            fault = true;
        }
        ImGui::PushStyleColor(ImGuiCol_TextLink, RED);
        itemLink(p_app, SelectionKind::Resource, resources.name(id));
        ImGui::PopStyleColor();
    }
}

void showInspector(App& p_app)
{
    if (p_app.ui.workspace == Workspace::Plugin)
    {
        p_app.ui.workspace = Workspace::Mission;
        p_app.ui.place_plugin_panels = true;
    }
    showPanel(p_app, PanelKind::Inspector);
}

void inspectorPanel(App& p_app, char const* p_key)
{
    std::string const window = panelTitle("Inspector", p_key);
    if (!beginPanel(p_app, PanelKind::Inspector, window.c_str()))
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
    UiState& ui = p_app.ui;
    bool const pinned = ui.pinned.kind != SelectionKind::None;
    Selection const shown = pinned ? ui.pinned : ui.selection;

    // Pin, back to the overview.
    if (shown.kind != SelectionKind::None)
    {
        if (ImGui::SmallButton(pinned ? "Unpin" : "Pin"))
        {
            ui.pinned = pinned ? Selection{} : ui.selection;
        }
        ImGui::SetItemTooltip(pinned ? "Follow the selection again."
                                     : "Keep this item here while the other "
                                       "panels follow the selection.");
        ImGui::SameLine();
        if (ImGui::SmallButton("Overview"))
        {
            ui.pinned = {};
            ui.selection = {};
        }
        ImGui::SetItemTooltip("What runs, waits and is in fault.");
        if (pinned)
        {
            ImGui::SameLine();
            ImGui::TextColored(ORANGE, "Pinned");
        }
    }

    robotik::Cell const& cell = p_app.simulation->cell();
    switch (shown.kind)
    {
        case SelectionKind::None:
            overview(p_app);
            break;
        case SelectionKind::Skill:
        {
            robotik::SkillId const id = cell.skills().find(shown.name);
            if (id == robotik::NO_SKILL)
            {
                emptyState("This skill is not in the loaded scenario.");
                break;
            }
            skillView(p_app, id);
            break;
        }
        case SelectionKind::Resource:
        {
            robotik::ResourceManager const& resources =
                cell.robot().resources();
            robotik::ResourceId const id = resources.find(shown.name);
            if (id == robotik::NO_RESOURCE)
            {
                emptyState("This resource is not in the loaded scenario.");
                break;
            }
            resourceView(p_app, id);
            break;
        }
        case SelectionKind::Robot:
        {
            robotik::RobotSession const* robot = cell.robot(shown.name);
            if (robot == nullptr)
            {
                emptyState("This robot is not in the loaded scenario.");
                break;
            }
            robotView(p_app, *robot);
            break;
        }
        case SelectionKind::Object:
            objectView(p_app, shown.name);
            break;
    }
    ImGui::End();
}
