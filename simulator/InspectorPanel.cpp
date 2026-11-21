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
        // Only workspace Mission opens the Inspector at first: elsewhere
        // the first click opens it, later ones leave the focus alone.
        if (!p_app.ui.panels[static_cast<std::size_t>(PanelKind::Inspector)])
        {
            showInspector(p_app);
        }
    }
    return clicked;
}

/// Robot a skill drives: its description, else the first robot.
static std::string robotOf(robotik::RoboticCell const& p_cell,
                           robotik::SkillId p_id)
{
    std::string const& robot = p_cell.skills().description(p_id).robot;
    if (!robot.empty() || p_cell.robots().empty())
    {
        return robot;
    }
    return p_cell.robots().front()->id();
}

/// Top of every view: the kind in small grey capitals with Pin and
/// Overview beside it, then the name, larger, with its state as a badge.
static void header(App& p_app,
                   char const* p_kind,
                   std::string const& p_name,
                   char const* p_state,
                   ImVec4 const& p_color)
{
    UiState& ui = p_app.ui;
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("%s", p_kind);
    if (ui.selection.kind != SelectionKind::None ||
        ui.pinned.kind != SelectionKind::None)
    {
        bool const pinned = ui.pinned.kind != SelectionKind::None;
        ImGui::SameLine(0.0F, 16.0F);
        if (ImGui::SmallButton(pinned ? "Unpin" : "Pin"))
        {
            ui.pinned = pinned ? Selection{} : ui.selection;
        }
        ImGui::SetItemTooltip(pinned ? "Follow the selection again."
                                     : "Keep this item here while you select "
                                       "others.");
        ImGui::SameLine();
        if (ImGui::SmallButton("Overview"))
        {
            ui.pinned = {};
            ui.selection = {};
        }
        ImGui::SetItemTooltip("What runs, waits and is in fault now.");
    }
    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 1.3F);
    ImGui::TextUnformatted(p_name.c_str());
    ImGui::PopFont();
    if (p_state != nullptr)
    {
        ImGui::SameLine(0.0F, 12.0F);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 3.0F);
        badge(p_state, p_color);
    }
}

/// What the item is doing now, in one plain sentence.
static void sentence(ImVec4 const& p_color, std::string const& p_text)
{
    ImGui::PushStyleColor(ImGuiCol_Text, p_color);
    ImGui::TextWrapped("%s", p_text.c_str());
    ImGui::PopStyleColor();
    ImGui::Spacing();
}

/// Starts a table of facts: a grey name on the left, its value on the
/// right. Call endTable() only when it returns true.
static bool beginFacts(char const* p_id)
{
    if (!beginTable(p_id, 2, ImGuiTableFlags_SizingFixedFit))
    {
        return false;
    }
    ImGui::TableSetupColumn("##name", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("##value", ImGuiTableColumnFlags_WidthStretch);
    return true;
}

/// Starts one fact: writes its name, the caller writes the value.
static void factName(char const* p_name)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", p_name);
    ImGui::TableNextColumn();
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

/// Everything about one skill: what it does now, then its settings, its
/// resources, its preconditions and its last runs.
static void skillView(App& p_app, robotik::SkillId p_id)
{
    robotik::Simulation const& simulation = *p_app.simulation;
    robotik::RoboticCell const& cell = simulation.cell();
    robotik::SkillScheduler const& skills = cell.skills();
    robotik::ResourceManager const& resources = cell.robot().resources();
    robotik::SkillDescription const& description = skills.description(p_id);
    robotik::SkillState const state = skills.state(p_id);
    double const now = simulation.time().value();
    std::span<robotik::SkillRun const> const trace = skills.trace();

    header(p_app,
           "SKILL",
           description.name,
           robotik::toString(state),
           ::colorOf(state));

    // Its last phase, for the sentence.
    robotik::SkillRun const* last = nullptr;
    for (std::size_t i = trace.size(); i > 0U; --i)
    {
        if (trace[i - 1U].skill == p_id)
        {
            last = &trace[i - 1U];
            break;
        }
    }
    std::string const why = skillWhy(skills, resources, p_id);
    char text[96];
    switch (state)
    {
        case robotik::SkillState::Running:
            std::snprintf(text,
                          sizeof text,
                          "Running for %.2f s.",
                          last != nullptr ? now - last->start.value() : 0.0);
            sentence(ORANGE, text);
            break;
        case robotik::SkillState::Waiting:
            sentence(ORANGE, "Waiting. " + why);
            break;
        case robotik::SkillState::Failed:
            sentence(RED, "Failed. " + why);
            break;
        case robotik::SkillState::Succeeded:
        case robotik::SkillState::Cancelled:
            std::snprintf(text,
                          sizeof text,
                          "%s at %.2f s.",
                          state == robotik::SkillState::Succeeded ? "Succeeded"
                                                                  : "Cancelled",
                          last != nullptr ? last->end.value() : now);
            sentence(state == robotik::SkillState::Succeeded ? GREEN : GREY,
                     text);
            break;
        case robotik::SkillState::Idle:
            sentence(GREY,
                     last == nullptr ? "Idle: never requested in this run."
                                     : "Idle.");
            break;
    }

    if (beginFacts("skill"))
    {
        std::string const robot = robotOf(cell, p_id);
        if (!robot.empty())
        {
            factName("Robot");
            itemLink(p_app, SelectionKind::Robot, robot);
        }
        factName("Priority");
        ImGui::Text("%d, %s",
                    static_cast<int>(description.priority),
                    description.cancellable
                        ? "a higher priority can take its resources"
                        : "never preempted");
        factName("When busy");
        ImGui::TextUnformatted(description.wait ? "waits for its resources"
                                                : "fails at once");
        for (robotik::ResourceRequirement const& need : description.resources)
        {
            ImGui::PushID(static_cast<int>(need.id));
            factName(&need == &description.resources.front() ? "Resources"
                                                             : "");
            itemLink(p_app, SelectionKind::Resource, resources.name(need.id));
            ImGui::SameLine();
            ImGui::TextDisabled(need.access == robotik::Access::Shared
                                    ? "shared,"
                                    : "exclusive,");
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
        if (description.resources.empty())
        {
            factName("Resources");
            ImGui::TextDisabled("none");
        }
        robotik::Precondition const* failed = skills.precondition(p_id);
        for (robotik::Precondition const& condition : description.preconditions)
        {
            factName(&condition == &description.preconditions.front() ? "Needs"
                                                                      : "");
            bool const is_failed = failed == &condition;
            ImGui::PushStyleColor(
                ImGuiCol_Text,
                is_failed ? RED : ImGui::GetStyleColorVec4(ImGuiCol_Text));
            ImGui::TextWrapped(
                "%s%s", condition.text.c_str(), is_failed ? "  (false)" : "");
            ImGui::PopStyleColor();
        }
        endTable();
    }

    ImGui::SeparatorText("Last runs");
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

/// Links to @p_ids on one line, or "none" in grey.
static void skillLinks(App& p_app, std::vector<robotik::SkillId> const& p_ids)
{
    if (p_ids.empty())
    {
        ImGui::TextDisabled("none");
        return;
    }
    for (robotik::SkillId const id : p_ids)
    {
        if (id != p_ids.front())
        {
            ImGui::SameLine();
        }
        skillLink(p_app, id);
    }
}

/// Everything about one resource: who holds it, who waits for it, who
/// needs it, its faults.
static void resourceView(App& p_app, robotik::ResourceId p_id)
{
    robotik::RoboticCell const& cell = p_app.simulation->cell();
    robotik::SkillScheduler const& skills = cell.skills();
    robotik::ResourceManager const& resources = cell.robot().resources();

    ImVec4 color;
    char const* word = resourceState(resources, p_id, color);
    header(p_app, "RESOURCE", resources.name(p_id), word, color);

    std::vector<robotik::SkillId> const held = holders(skills, resources, p_id);
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
    bool const outside =
        held.empty() && (resources.owner(p_id) != robotik::NO_OWNER ||
                         resources.users(p_id) > 0U);
    if (!resources.available(p_id))
    {
        sentence(RED,
                 "In fault: the skills that need it wait or fail until it "
                 "comes back.");
    }
    else if (outside)
    {
        sentence(ORANGE,
                 "Held outside the skills (the plugin or the Teach Pendant).");
    }
    else if (held.empty())
    {
        sentence(GREY, "Free: no skill holds it.");
    }
    else
    {
        sentence(color,
                 held.size() > 1U ? "Shared by the skills below."
                                  : "Held by the skill below.");
    }

    if (beginFacts("resource"))
    {
        factName("Held by");
        skillLinks(p_app, held);
        factName("Waited for by");
        skillLinks(p_app, waiting);
        factName("Needed by");
        skillLinks(p_app, needed);
        factName("Faults");
        bool any = false;
        for (FaultSpan const& span : p_app.ui.timeline_memory.faults)
        {
            if (span.resource != p_id)
            {
                continue;
            }
            if (any)
            {
                ImGui::SameLine();
            }
            any = true;
            if (span.open)
            {
                ImGui::TextColored(RED, "since %.2f s", span.start);
            }
            else
            {
                ImGui::Text("%.2f-%.2f s", span.start, span.end);
            }
        }
        if (!any)
        {
            ImGui::TextDisabled("none in this run");
        }
        endTable();
    }
    ImGui::TextDisabled("Inject or restore a fault in the Resources panel.");
}

// --- Robot ------------------------------------------------------------------

/// Everything about one robot: who drives it, its joints, its skills.
static void robotView(App& p_app, robotik::RobotSession const& p_robot)
{
    robotik::RoboticCell const& cell = p_app.simulation->cell();
    robotik::SkillScheduler const& skills = cell.skills();

    bool const manual = p_app.teach.manual;
    header(p_app,
           "ROBOT",
           p_robot.id(),
           manual ? "MANUAL" : "MISSION",
           manual ? ORANGE : BLUE);

    std::vector<robotik::SkillId> active;
    for (robotik::SkillId id = 0; id < skills.size(); ++id)
    {
        if (robotOf(cell, id) == p_robot.id() &&
            (skills.state(id) == robotik::SkillState::Running ||
             skills.state(id) == robotik::SkillState::Waiting))
        {
            active.push_back(id);
        }
    }
    if (manual)
    {
        sentence(ORANGE,
                 "Moved by hand: the mission is suspended (Robot workspace).");
    }
    else
    {
        sentence(active.empty() ? GREY : ORANGE,
                 active.empty() ? "No skill drives it now."
                                : "Driven by the skills below.");
    }

    if (beginFacts("robot"))
    {
        factName("Model");
        ImGui::TextUnformatted(p_robot.name().c_str());
        if (!p_robot.tool().empty())
        {
            factName("Tool");
            ImGui::TextUnformatted(p_robot.tool().c_str());
        }
        factName("Skills now");
        skillLinks(p_app, active);
        endTable();
    }

    // One row per joint: name, value, where it is between its limits.
    ImGui::SeparatorText("Joints");
    robotik::JointSet const& joints = p_robot.joints();
    if (beginTable("joints", 3, ImGuiTableFlags_SizingFixedFit))
    {
        ImGui::TableSetupColumn("##joint", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##value", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##range", ImGuiTableColumnFlags_WidthStretch);
        for (robotik::JointId id = 0; id < joints.size(); ++id)
        {
            bool const linear = joints.isPrismatic(id);
            double lower = 0.0;
            double upper = 0.0;
            if (linear)
            {
                auto const limits = joints.limits(joints.requirePrismatic(id));
                lower = limits.lower.value();
                upper = limits.upper.value();
            }
            else
            {
                auto const limits = joints.limits(joints.requireRevolute(id));
                lower = limits.lower.value();
                upper = limits.upper.value();
            }
            double const position = joints.position(id);
            if (!std::isfinite(lower) || !std::isfinite(upper))
            {
                lower = position - std::numbers::pi;
                upper = position + std::numbers::pi;
            }
            bool const commanded =
                joints.mode(id) == robotik::JointMode::Position;
            ImGui::PushID(static_cast<int>(id));
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(joints.name(id).c_str());
            ImGui::TableNextColumn();
            ImGui::Text(linear ? "%.3f m" : "%.1f°",
                        linear ? position
                               : position * 180.0 / std::numbers::pi);
            ImGui::TableNextColumn();
            rangeBar(position,
                     lower,
                     upper,
                     commanded ? joints.target(id) : position,
                     commanded,
                     "",
                     joints.mode(id) == robotik::JointMode::Disabled ? RED
                                                                     : BLUE);
            ImGui::PopID();
        }
        endTable();
    }
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
    robotik::RoboticCell const& cell = p_app.simulation->cell();
    robotik::WorldObject const* belief = cell.worldModel().find(p_name);
    bool const real = inWorld(p_app, p_name);

    ImVec4 source_color = GREY;
    char const* source =
        belief == nullptr ? "unknown" : sourceOf(belief->source, source_color);
    header(p_app, "OBJECT", p_name, source, source_color);

    // Both positions in the world frame, and the distance between them.
    robotik::Vector3 truth{};
    bool has_truth = false;
    if (real)
    {
        try
        {
            truth = cell.requirePose(p_name).position;
            has_truth = true;
        }
        catch (std::invalid_argument const&)
        {
            has_truth = false;
        }
    }
    if (belief == nullptr)
    {
        sentence(GREY, "The robot has no belief about it.");
        if (has_truth && beginFacts("object"))
        {
            factName("Truth");
            ImGui::Text("%.3f %.3f %.3f m", truth.x, truth.y, truth.z);
            endTable();
        }
        return;
    }
    robotik::Vector3 const believed = cell.beliefFrame() * belief->position;
    double const error =
        has_truth ? 1000.0 * robotik::norm(believed - truth) : 0.0;
    ImVec4 const error_color = error < 10.0   ? GREEN
                               : error < 30.0 ? ORANGE
                                              : RED;
    char text[128];
    if (has_truth)
    {
        std::snprintf(text,
                      sizeof text,
                      "Believed %.1f mm from where it really is.",
                      error);
        sentence(error_color, text);
    }
    else
    {
        sentence(ORANGE, "Believed, but not in the world.");
    }
    if (beginFacts("object"))
    {
        factName("Belief");
        ImGui::Text("%.3f %.3f %.3f m", believed.x, believed.y, believed.z);
        factName("Truth");
        if (has_truth)
        {
            ImGui::Text("%.3f %.3f %.3f m", truth.x, truth.y, truth.z);
            factName("Error");
            ImGui::TextColored(error_color, "%.1f mm", error);
        }
        else
        {
            ImGui::TextDisabled("not in the world");
        }
        factName("Confidence");
        ImGui::Text("%.0f %%", static_cast<double>(belief->confidence) * 100.0);
        factName("Seen");
        if (robotik::observed(*belief))
        {
            ImGui::Text("%u times, last at %.2f s",
                        belief->observations,
                        belief->seen.value());
        }
        else
        {
            ImGui::TextDisabled("never");
        }
        if (belief->size.x > 0.0 || belief->size.y > 0.0 ||
            belief->size.z > 0.0)
        {
            factName("Size");
            ImGui::Text("%.3f %.3f %.3f m",
                        belief->size.x,
                        belief->size.y,
                        belief->size.z);
        }
        endTable();
    }
    ImGui::TextDisabled("Skills see the belief only; the truth is here to "
                        "compare.");
}

// --- Nothing selected ---------------------------------------------------------

/// The run in a few lines: what runs, what waits, what is in fault.
static void overview(App& p_app)
{
    robotik::RoboticCell const& cell = p_app.simulation->cell();
    robotik::SkillScheduler const& skills = cell.skills();
    robotik::ResourceManager const& resources = cell.robot().resources();
    header(p_app,
           "RUN",
           p_app.scenario_title.empty() ? std::string("Run")
                                        : p_app.scenario_title,
           nullptr,
           GREY);
    sentence(GREY,
             "Click a skill, a resource, a robot or an object anywhere "
             "to see it here.");

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
    if (!beginFacts("overview"))
    {
        return;
    }
    factName("Robots");
    for (auto const& robot : cell.robots())
    {
        if (robot != cell.robots().front())
        {
            ImGui::SameLine();
        }
        itemLink(p_app, SelectionKind::Robot, robot->id());
    }
    factName("Running");
    skillLinks(p_app, running);
    factName("Waiting");
    skillLinks(p_app, waiting);
    if (!failed.empty())
    {
        factName("Failed");
        skillLinks(p_app, failed);
    }
    factName("In fault");
    bool fault = false;
    for (robotik::ResourceId id = 0; id < resources.size(); ++id)
    {
        if (resources.available(id))
        {
            continue;
        }
        if (fault)
        {
            ImGui::SameLine();
        }
        fault = true;
        ImGui::PushStyleColor(ImGuiCol_TextLink, RED);
        itemLink(p_app, SelectionKind::Resource, resources.name(id));
        ImGui::PopStyleColor();
    }
    if (!fault)
    {
        ImGui::TextDisabled("none");
    }
    std::span<robotik::WorldObject const> const objects =
        cell.worldModel().objects();
    if (!objects.empty())
    {
        factName("Objects");
        for (robotik::WorldObject const& object : objects)
        {
            if (&object != &objects.front())
            {
                ImGui::SameLine();
            }
            ImGui::PushStyleColor(
                ImGuiCol_TextLink,
                robotik::observed(object)
                    ? ImGui::GetStyleColorVec4(ImGuiCol_TextLink)
                    : GREY);
            itemLink(p_app, SelectionKind::Object, object.name);
            ImGui::PopStyleColor();
        }
    }
    endTable();
}

void showInspector(App& p_app)
{
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

    robotik::RoboticCell const& cell = p_app.simulation->cell();
    ImGui::PushID(static_cast<int>(shown.kind));
    ImGui::PushID(shown.name.c_str());
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
    ImGui::PopID();
    ImGui::PopID();
    ImGui::End();
}
