// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file TimelinePanel.cpp
/// The Timeline panel: what the skills did, on one time axis.
///
/// One row per skill, grouped by robot when there are several, then one row
/// per resource (who held it, when it was in fault). By default the window
/// of time follows the present and keeps its width, so a phase keeps its
/// size while the run goes on. The wheel zooms, a drag or Shift+wheel moves
/// in time, a double-click follows the present again. Hovering a phase shows
/// its details; the line under the rows reads the phase hovered or clicked,
/// or what ran at the time under the mouse.
///
/// @par Contract
/// Reads the scheduler trace (SkillScheduler::trace) and the resource
/// manager; never commands them. The simulation cannot go back in time: the
/// cursor inspects the past, it does not replay it.

#include "SkillView.hpp"
#include "Ui.hpp"

#include "Robotik/Task/Planning/PlanExecutor.hpp"

#include <imgui.h>
#include <imgui_internal.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <span>
#include <string>
#include <vector>

/// Narrowest and widest windows of time, in seconds.
static constexpr double MIN_WINDOW_S = 0.2;
static constexpr double MAX_WINDOW_S = 7200.0;
/// Share of the window kept after the present when following it.
static constexpr double LEAD = 0.04;
/// Fewest pixels between two ticks of the ruler.
static constexpr float TICK_PX = 70.0F;

// ****************************************************************************
/// What one row of the Timeline shows.
// ****************************************************************************
enum class RowKind : std::uint8_t
{
    TreeFailure, ///< A composite or decorator of the tree that failed.
    Robot,       ///< Header of the skills of one robot (several robots).
    Skill,       ///< The phases of one skill.
    Resources,   ///< Header of the resource rows.
    Resource,    ///< Who held one resource, and its faults.
};

// ****************************************************************************
/// One row: its kind, the index of what it shows, the robot it belongs to.
// ****************************************************************************
struct Row
{
    RowKind kind = RowKind::Skill;
    /// Tree failure, skill or resource index.
    std::size_t index = 0;
    std::string label;
    std::string robot;
};

// ****************************************************************************
/// The window of time drawn and its pixel scale.
// ****************************************************************************
struct TimeAxis
{
    double t0 = 0.0;
    double t1 = 1.0;
    /// Left edge and width of the lanes, in screen pixels.
    float x = 0.0F;
    float width = 1.0F;
};

/// Screen x of time @p_t.
static float xOf(TimeAxis const& p_axis, double p_t)
{
    return p_axis.x + static_cast<float>((p_t - p_axis.t0) /
                                         (p_axis.t1 - p_axis.t0)) *
                          p_axis.width;
}

/// Time at screen x @p_x.
static double timeAt(TimeAxis const& p_axis, float p_x)
{
    return p_axis.t0 + static_cast<double>((p_x - p_axis.x) / p_axis.width) *
                           (p_axis.t1 - p_axis.t0);
}

/// "12.4 s", with more digits when the window is narrow.
static std::string formatTime(double p_t, double p_step)
{
    char text[32];
    if (p_step < 1.0)
    {
        std::snprintf(text, sizeof text, "%.1f s", p_t);
    }
    else
    {
        std::snprintf(text, sizeof text, "%g s", std::round(p_t));
    }
    return text;
}

/// Robot a skill drives: its description, else the first robot.
static std::string const& robotOf(robotik::Cell const& p_cell, robotik::SkillId p_id)
{
    static std::string const none;
    std::string const& robot = p_cell.skills().description(p_id).robot;
    if (!robot.empty() || p_cell.robots().empty())
    {
        return robot.empty() ? none : robot;
    }
    return p_cell.robots().front()->id();
}

/// True when @p_folded lists @p_robot.
static bool isFolded(TimelineView const& p_view, std::string const& p_robot)
{
    return std::ranges::find(p_view.folded, p_robot) != p_view.folded.end();
}

/// Folds or unfolds the rows of @p_robot.
static void toggleFolded(TimelineView& p_view, std::string const& p_robot)
{
    auto const at = std::ranges::find(p_view.folded, p_robot);
    if (at == p_view.folded.end())
    {
        p_view.folded.push_back(p_robot);
    }
    else
    {
        p_view.folded.erase(at);
    }
}

/// Does skill @p_skill reserve resource @p_resource, and how.
static robotik::ResourceRequirement const*
needOf(robotik::SkillScheduler const& p_skills,
       robotik::SkillId p_skill,
       robotik::ResourceId p_resource)
{
    for (robotik::ResourceRequirement const& need :
         p_skills.description(p_skill).resources)
    {
        if (need.id == p_resource)
        {
            return &need;
        }
    }
    return nullptr;
}

/// Short text written inside a phase.
static std::string phaseText(robotik::SkillScheduler const& p_skills,
                             robotik::ResourceManager const& p_resources,
                             robotik::SkillRun const& p_run)
{
    switch (p_run.state)
    {
        case robotik::SkillState::Waiting:
            if (p_run.reason == robotik::SkillReason::Precondition)
            {
                return "waits: precondition";
            }
            if (p_run.blocker != robotik::NO_RESOURCE)
            {
                return "waits: " + p_resources.name(p_run.blocker);
            }
            return "waits";
        case robotik::SkillState::Failed:
            return "failed";
        case robotik::SkillState::Cancelled:
            return robotik::toString(p_run.reason);
        case robotik::SkillState::Idle:
        case robotik::SkillState::Running:
        case robotik::SkillState::Succeeded:
            break;
    }
    return p_skills.name(p_run.skill);
}

/// Last phase of skill @p_id in @p_trace, or NO_PHASE.
static std::size_t lastPhase(std::span<robotik::SkillRun const> p_trace,
                             robotik::SkillId p_id)
{
    for (std::size_t i = p_trace.size(); i > 0U; --i)
    {
        if (p_trace[i - 1U].skill == p_id)
        {
            return i - 1U;
        }
    }
    return NO_PHASE;
}

/// End of phase @p_index: the present while it goes on (its last update
/// may be one step older than the clock).
static double phaseEnd(robotik::SkillScheduler const& p_skills,
                       robotik::SkillRun const& p_run,
                       bool p_last,
                       double p_now)
{
    bool const ongoing = p_last && p_skills.state(p_run.skill) == p_run.state &&
                         (p_run.state == robotik::SkillState::Running ||
                          p_run.state == robotik::SkillState::Waiting);
    return ongoing ? std::max(p_now, p_run.end.value()) : p_run.end.value();
}

/// Index of the last phase of each skill in @p_trace (NO_PHASE: none).
static std::vector<std::size_t>
lastPhases(std::span<robotik::SkillRun const> p_trace, std::size_t p_skills)
{
    std::vector<std::size_t> last(p_skills, NO_PHASE);
    for (std::size_t i = 0; i < p_trace.size(); ++i)
    {
        last[p_trace[i].skill] = i;
    }
    return last;
}

/// One sentence on phase @p_index: skill, state, reason, times.
static std::string phaseSentence(robotik::Simulation const& p_simulation,
                                 std::size_t p_index)
{
    robotik::SkillScheduler const& skills = p_simulation.cell().skills();
    robotik::ResourceManager const& resources =
        p_simulation.cell().robot().resources();
    robotik::SkillRun const& run = skills.trace()[p_index];
    std::string text = skills.name(run.skill);
    text += " · ";
    text += robotik::toString(run.state);
    bool const current = lastPhase(skills.trace(), run.skill) == p_index &&
                         skills.state(run.skill) == run.state;
    std::string const why = current ? skillWhy(skills, resources, run.skill)
                                    : std::string();
    if (!why.empty())
    {
        text += " · " + why;
    }
    else if (run.reason != robotik::SkillReason::None)
    {
        text += " · ";
        text += robotik::toString(run.reason);
        if (run.blocker != robotik::NO_RESOURCE)
        {
            text += ": " + resources.name(run.blocker);
        }
    }
    double const end =
        phaseEnd(skills, run, lastPhase(skills.trace(), run.skill) == p_index,
                 p_simulation.time().value());
    char times[96];
    std::snprintf(times,
                  sizeof times,
                  " · %.2f s to %.2f s (%.2f s)",
                  run.start.value(),
                  end,
                  end - run.start.value());
    text += times;
    return text;
}

/// What ran and what waited at time @p_t.
static std::string momentSentence(robotik::Simulation const& p_simulation,
                                  double p_t)
{
    robotik::SkillScheduler const& skills = p_simulation.cell().skills();
    robotik::ResourceManager const& resources =
        p_simulation.cell().robot().resources();
    std::string running;
    std::string waiting;
    double const eps = 1e-6;
    std::span<robotik::SkillRun const> const trace = skills.trace();
    std::vector<std::size_t> const last = lastPhases(trace, skills.size());
    double const now = p_simulation.time().value();
    for (std::size_t i = 0; i < trace.size(); ++i)
    {
        robotik::SkillRun const& run = trace[i];
        double const end = phaseEnd(skills, run, last[run.skill] == i, now);
        if (run.start.value() > p_t + eps || end < p_t - eps)
        {
            continue;
        }
        if (run.state == robotik::SkillState::Waiting)
        {
            waiting += waiting.empty() ? "" : ", ";
            waiting += skills.name(run.skill);
            if (run.blocker != robotik::NO_RESOURCE)
            {
                waiting += " (" + resources.name(run.blocker) + ")";
            }
        }
        else
        {
            running += running.empty() ? "" : ", ";
            running += skills.name(run.skill);
        }
    }
    char head[32];
    std::snprintf(head, sizeof head, "%.2f s", p_t);
    std::string text = head;
    text += " · running: " + (running.empty() ? std::string("-") : running);
    if (!waiting.empty())
    {
        text += " · waiting: " + waiting;
    }
    return text;
}

/// Window of time shown, from the view and the present.
static void timeWindow(TimelineView const& p_view,
                       double p_now,
                       double& p_t0,
                       double& p_t1)
{
    if (p_view.whole_run)
    {
        p_t0 = 0.0;
        p_t1 = std::max(p_now, 1.0) * (1.0 + LEAD);
    }
    else if (p_view.follow)
    {
        p_t1 = std::max(p_view.window_s, p_now + LEAD * p_view.window_s);
        p_t0 = p_t1 - p_view.window_s;
    }
    else
    {
        p_t0 = p_view.start_s;
        p_t1 = p_view.start_s + p_view.window_s;
    }
}

/// Rows to draw, from the skills, the robots and the resources.
static std::vector<Row> rowsOf(App const& p_app)
{
    robotik::Cell const& cell = p_app.simulation->cell();
    robotik::SkillScheduler const& skills = cell.skills();
    robotik::ResourceManager const& resources = cell.robot().resources();
    TimelineView const& view = p_app.ui.timeline;
    std::vector<Row> rows;
    std::vector<TreeFailure> const& failures =
        p_app.ui.timeline_memory.tree_failures;
    for (std::size_t i = 0; i < failures.size(); ++i)
    {
        rows.push_back({ RowKind::TreeFailure, i, failures[i].label, {} });
    }
    // Robots in the order of their first skill.
    std::vector<std::string> robots;
    for (robotik::SkillId id = 0; id < skills.size(); ++id)
    {
        std::string const& robot = robotOf(cell, id);
        if (std::ranges::find(robots, robot) == robots.end())
        {
            robots.push_back(robot);
        }
    }
    bool const headers = robots.size() > 1U;
    for (std::string const& robot : robots)
    {
        if (headers)
        {
            rows.push_back({ RowKind::Robot, 0, robot, robot });
            if (isFolded(view, robot))
            {
                continue;
            }
        }
        for (robotik::SkillId id = 0; id < skills.size(); ++id)
        {
            if (robotOf(cell, id) != robot ||
                (view.active_only &&
                 skills.state(id) == robotik::SkillState::Idle))
            {
                continue;
            }
            rows.push_back({ RowKind::Skill, id, skills.name(id), robot });
        }
    }
    if (resources.size() > 0U)
    {
        rows.push_back({ RowKind::Resources,
                         0,
                         "Resources (" + std::to_string(resources.size()) + ")",
                         {} });
        if (view.resources_open)
        {
            for (robotik::ResourceId id = 0; id < resources.size(); ++id)
            {
                rows.push_back({ RowKind::Resource, id, resources.name(id), {} });
            }
        }
    }
    return rows;
}

/// @p_color with its alpha multiplied by @p_alpha.
static ImU32 faded(ImVec4 p_color, float p_alpha)
{
    p_color.w *= p_alpha;
    return ImGui::GetColorU32(p_color);
}

/// Left margin of the name of a row.
static float indentOf(Row const& p_row, bool p_robot_headers)
{
    switch (p_row.kind)
    {
        case RowKind::Skill:
            return p_robot_headers ? 18.0F : 4.0F;
        case RowKind::Resource:
            return 18.0F;
        case RowKind::TreeFailure:
        case RowKind::Robot:
        case RowKind::Resources:
            break;
    }
    return 4.0F;
}

/// Fill and outline of a phase of state @p_state.
static void phaseColors(robotik::SkillState p_state, ImU32& p_fill, ImU32& p_line)
{
    ImVec4 color = ::colorOf(p_state);
    p_line = ImGui::GetColorU32(color);
    color.w = p_state == robotik::SkillState::Waiting ? 0.22F : 0.85F;
    p_fill = ImGui::GetColorU32(color);
}

/// A block from @p_x0 to @p_x1 with @p_text inside when it fits.
static void block(ImDrawList* p_draw,
                  float p_x0,
                  float p_x1,
                  float p_y,
                  float p_height,
                  ImU32 p_fill,
                  ImU32 p_line,
                  ImU32 p_text_color,
                  std::string const& p_text)
{
    float const x1 = std::max(p_x1, p_x0 + 3.0F);
    ImVec2 const a(p_x0, p_y + 3.0F);
    ImVec2 const b(x1, p_y + p_height - 3.0F);
    p_draw->AddRectFilled(a, b, p_fill, 3.0F);
    p_draw->AddRect(a, b, p_line, 3.0F);
    float const room = x1 - p_x0 - 8.0F;
    if (room > 12.0F && !p_text.empty())
    {
        p_draw->PushClipRect(a, b, true);
        p_draw->AddText(
            ImVec2(p_x0 + 4.0F,
                   p_y + (p_height - ImGui::GetTextLineHeight()) * 0.5F),
            p_text_color,
            p_text.c_str());
        p_draw->PopClipRect();
    }
}

/// Text color readable on a filled block.
static ImU32 inkOn(robotik::SkillState p_state)
{
    return p_state == robotik::SkillState::Waiting
               ? ImGui::GetColorU32(ImGuiCol_Text)
               : IM_COL32(20, 22, 25, 255);
}

/// Who holds each resource now, in one line.
static std::string holdersLine(robotik::SkillScheduler const& p_skills,
                               robotik::ResourceManager const& p_resources)
{
    std::string text;
    for (robotik::ResourceId id = 0; id < p_resources.size(); ++id)
    {
        std::string who;
        if (!p_resources.available(id))
        {
            who = "FAULT";
        }
        else if (p_resources.owner(id) != robotik::NO_OWNER)
        {
            who = skillOwnerName(p_skills, p_resources.owner(id));
        }
        else if (p_resources.users(id) > 0U)
        {
            who = "shared by " + std::to_string(p_resources.users(id));
        }
        if (who.empty())
        {
            continue;
        }
        text += text.empty() ? "" : " · ";
        text += p_resources.name(id) + ": " + who;
    }
    return text.empty() ? std::string("all free") : text;
}

/// Buttons above the rows: follow, whole run, active only.
static void toolbar(TimelineView& p_view, double p_now)
{
    auto toggle = [](char const* p_label, bool p_on)
    {
        if (p_on)
        {
            ImGui::PushStyleColor(ImGuiCol_Button,
                                  ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        }
        bool const pressed = ImGui::SmallButton(p_label);
        if (p_on)
        {
            ImGui::PopStyleColor();
        }
        return pressed;
    };
    if (toggle("Follow", p_view.follow && !p_view.whole_run))
    {
        p_view.follow = true;
        p_view.whole_run = false;
    }
    ImGui::SetItemTooltip("The window ends at the present and moves with it "
                          "(double-click the rows).");
    ImGui::SameLine();
    if (toggle("Whole run", p_view.whole_run))
    {
        p_view.whole_run = !p_view.whole_run;
        p_view.follow = !p_view.whole_run;
    }
    ImGui::SetItemTooltip("From 0 to the present.");
    ImGui::SameLine();
    ImGui::TextDisabled("%s", p_view.whole_run
                                  ? formatTime(std::max(p_now, 1.0), 1.0).c_str()
                                  : formatTime(p_view.window_s,
                                               p_view.window_s < 10.0 ? 0.5 : 1.0)
                                        .c_str());
    ImGui::SameLine(0.0F, 16.0F);
    ImGui::Checkbox("Active only", &p_view.active_only);
    ImGui::SetItemTooltip("Hide the skills never requested since the start.");
    ImGui::SameLine();
    helpMarker("Wheel: zoom.\nDrag or Shift+wheel: move in time.\n"
               "Double-click: follow the present again.\n"
               "Click a phase: its details on the bottom line.\n"
               "Wheel over the names: scroll the rows.\n"
               "Click a robot or Resources: fold or unfold its rows.");
}

/// Time ruler: ticks, their times, the present and the time under the mouse.
static void ruler(TimeAxis const& p_axis,
                  double p_now,
                  double p_hover_t,
                  double& p_step)
{
    float const height = ImGui::GetTextLineHeight() + 6.0F;
    ImVec2 const at = ImGui::GetCursorScreenPos();
    float const full = ImGui::GetContentRegionAvail().x;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    double const span = p_axis.t1 - p_axis.t0;
    p_step = 3600.0;
    for (double const step : { 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 15.0,
                               30.0, 60.0, 120.0, 300.0, 600.0, 1800.0, 3600.0 })
    {
        if (static_cast<double>(p_axis.width) * step / span >= TICK_PX)
        {
            p_step = step;
            break;
        }
    }
    ImU32 const muted = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    draw->AddRectFilled(at,
                        ImVec2(at.x + full, at.y + height),
                        ImGui::GetColorU32(ImGuiCol_TableHeaderBg));
    draw->PushClipRect(ImVec2(p_axis.x, at.y),
                       ImVec2(p_axis.x + p_axis.width + 40.0F, at.y + height),
                       true);
    for (double t = std::floor(p_axis.t0 / p_step) * p_step; t <= p_axis.t1;
         t += p_step)
    {
        if (t < -1e-9)
        {
            continue;
        }
        float const x = xOf(p_axis, t);
        draw->AddLine(ImVec2(x, at.y + height - 5.0F),
                      ImVec2(x, at.y + height),
                      muted);
        draw->AddText(ImVec2(x + 3.0F, at.y + 2.0F),
                      muted,
                      formatTime(t, p_step).c_str());
    }
    ImU32 const text = ImGui::GetColorU32(ImGuiCol_Text);
    float const now_x = xOf(p_axis, p_now);
    draw->AddTriangleFilled(ImVec2(now_x - 5.0F, at.y + height - 6.0F),
                            ImVec2(now_x + 5.0F, at.y + height - 6.0F),
                            ImVec2(now_x, at.y + height),
                            text);
    if (p_hover_t >= 0.0)
    {
        std::string const label = formatTime(p_hover_t, 0.5);
        float const x = xOf(p_axis, p_hover_t);
        ImVec2 const size = ImGui::CalcTextSize(label.c_str());
        draw->AddRectFilled(ImVec2(x - 2.0F, at.y),
                            ImVec2(x + size.x + 6.0F, at.y + height),
                            ImGui::GetColorU32(ImGuiCol_PopupBg));
        draw->AddText(ImVec2(x + 2.0F, at.y + 2.0F), text, label.c_str());
    }
    draw->PopClipRect();
    draw->AddText(ImVec2(at.x + 4.0F, at.y + 2.0F),
                  muted,
                  formatTime(p_now, 0.01).c_str());
    ImGui::Dummy(ImVec2(full, height));
}

void sampleTimeline(App& p_app)
{
    if (!p_app.simulation)
    {
        return;
    }
    robotik::Simulation const& simulation = *p_app.simulation;
    TimelineMemory& memory = p_app.ui.timeline_memory;
    double const now = simulation.time().value();
    if (now + 1e-9 < memory.sampled)
    {
        memory = {};
    }
    if (memory.selected != NO_PHASE &&
        memory.selected >= simulation.cell().skills().trace().size())
    {
        memory.selected = NO_PHASE;
    }
    if (now == memory.sampled)
    {
        return;
    }
    memory.sampled = now;

    // Resource faults: the manager knows only the present.
    robotik::ResourceManager const& resources =
        simulation.cell().robot().resources();
    for (robotik::ResourceId id = 0; id < resources.size(); ++id)
    {
        auto open = std::ranges::find_if(
            memory.faults,
            [id](FaultSpan const& p_span)
            { return p_span.open && p_span.resource == id; });
        if (!resources.available(id))
        {
            if (open == memory.faults.end())
            {
                memory.faults.push_back({ id, now, now, true });
            }
            else
            {
                open->end = now;
            }
        }
        else if (open != memory.faults.end())
        {
            open->end = now;
            open->open = false;
        }
    }

    // Steps of the plan: the executor keeps their state, not their time.
    if (robotik::PlanExecutor const* executor = simulation.cell().planner())
    {
        std::vector<robotik::StepStatus> const& status = executor->steps();
        std::vector<robotik::GroundAction> const& plan = executor->plan();
        if (executor->history().size() != memory.plans ||
            memory.step_seen.size() != status.size())
        {
            memory.plans = executor->history().size();
            memory.step_seen.assign(
                status.size(), static_cast<int>(robotik::StepStatus::Pending));
        }
        for (std::size_t i = 0; i < status.size() && i < plan.size(); ++i)
        {
            int const now_status = static_cast<int>(status[i]);
            if (now_status == memory.step_seen[i])
            {
                continue;
            }
            memory.step_seen[i] = now_status;
            if (status[i] == robotik::StepStatus::Pending)
            {
                continue;
            }
            memory.steps.push_back(
                { now,
                  memory.plans,
                  i + 1U,
                  robotik::toString(robotik::Atom{ plan[i].name, plan[i].args }),
                  status[i] == robotik::StepStatus::Skipped,
                  status[i] == robotik::StepStatus::Failed });
        }
    }

    // Tree failures: the first time a control node is seen failed.
    bt::Tree const* tree = simulation.cell().tree();
    if (tree != memory.tree)
    {
        memory.tree_failures.clear();
        memory.tree = tree;
    }
    if (tree == nullptr || !tree->hasRoot())
    {
        memory.tree_failures.clear();
        return;
    }
    std::vector<std::string> labels;
    std::vector<bt::Node const*> stack{ &tree->getRoot() };
    while (!stack.empty())
    {
        bt::Node const* node = stack.back();
        stack.pop_back();
        auto const* composite = dynamic_cast<bt::Composite const*>(node);
        auto const* decorator = dynamic_cast<bt::Decorator const*>(node);
        if ((composite != nullptr || decorator != nullptr) &&
            node->status() == bt::Status::FAILURE)
        {
            std::string label = node->typeName();
            if (!node->name.empty())
            {
                label += ' ';
                label += node->name;
            }
            labels.push_back(std::move(label));
        }
        if (composite != nullptr)
        {
            for (std::size_t i = composite->childIndices().size(); i > 0U; --i)
            {
                stack.push_back(&composite->childAt(i - 1U));
            }
        }
        else if (decorator != nullptr && decorator->hasChild())
        {
            stack.push_back(&decorator->childNode());
        }
        else if (bt::Node const* nested = subtreeRoot(*node))
        {
            stack.push_back(nested);
        }
    }
    std::vector<TreeFailure> next;
    next.reserve(labels.size());
    for (std::string const& label : labels)
    {
        auto const known =
            std::ranges::find_if(memory.tree_failures,
                                 [&](TreeFailure const& p_mark)
                                 { return p_mark.label == label; });
        next.push_back(known == memory.tree_failures.end()
                           ? TreeFailure{ label, now }
                           : *known);
    }
    memory.tree_failures = std::move(next);
}

void timelinePanel(App& p_app)
{
    if (!ImGui::Begin("Timeline"))
    {
        ImGui::End();
        return;
    }
    if (viewElsewhere(p_app, ViewKind::Timeline))
    {
        viewNote(p_app, ViewKind::Timeline);
    }
    else
    {
        if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows))
        {
            p_app.ui.hovered_view = ViewKind::Timeline;
        }
        timelineView(p_app);
    }
    ImGui::End();
}

void timelineView(App& p_app)
{
    if (!p_app.simulation)
    {
        emptyState("No scenario loaded.");
        return;
    }
    robotik::Simulation const& simulation = *p_app.simulation;
    robotik::Cell const& cell = simulation.cell();
    robotik::SkillScheduler const& skills = cell.skills();
    robotik::ResourceManager const& resources = cell.robot().resources();
    std::span<robotik::SkillRun const> const trace = skills.trace();
    TimelineView& view = p_app.ui.timeline;
    TimelineMemory& memory = p_app.ui.timeline_memory;
    double const now = simulation.time().value();
    ImGuiIO const& io = ImGui::GetIO();

    toolbar(view, now);
    // ViewKind, the line above the view has the button to come back.
    if (p_app.ui.maximized != ViewKind::Timeline)
    {
        ImGui::SameLine();
        if (ImGui::SmallButton("Full view"))
        {
            toggleMaximized(p_app, ViewKind::Timeline);
        }
        ImGui::SetItemTooltip("Ctrl+Space over a view; Esc to come back.");
    }
    std::vector<Row> const rows = rowsOf(p_app);

    // Geometry shared by the ruler and the rows.
    float const row_h = ImGui::GetTextLineHeight() + 9.0F;
    bool const robot_headers =
        std::ranges::any_of(rows,
                            [](Row const& p_row)
                            { return p_row.kind == RowKind::Robot; });
    float label_w = 110.0F;
    for (Row const& row : rows)
    {
        label_w = std::max(label_w,
                           ImGui::CalcTextSize(row.label.c_str()).x +
                               indentOf(row, robot_headers) + 30.0F);
    }
    float const full_w = ImGui::GetContentRegionAvail().x;
    label_w = std::min(label_w, std::max(110.0F, full_w * 0.35F));
    TimeAxis axis;
    timeWindow(view, now, axis.t0, axis.t1);
    axis.x = ImGui::GetCursorScreenPos().x + label_w;
    axis.width = std::max(full_w - label_w - ImGui::GetStyle().ScrollbarSize,
                          20.0F);

    // The time under the mouse, from the previous frame's rows rectangle.
    ImVec2 const mouse = io.MousePos;
    double hover_t = -1.0;
    ImGuiID const rows_id = ImGui::GetID("rows");
    if (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) &&
        mouse.y >= ImGui::GetCursorScreenPos().y && mouse.x >= axis.x && mouse.x <= axis.x + axis.width)
    {
        hover_t = std::max(timeAt(axis, mouse.x), 0.0);
    }
    double step = 1.0;
    ruler(axis, now, hover_t, step);

    // Rows, scrolled vertically; the line of details stays below them.
    float const footer = ImGui::GetTextLineHeightWithSpacing() + 4.0F;
    ImVec2 const child(0.0F,
                       std::max(ImGui::GetContentRegionAvail().y - footer,
                                row_h * 2.0F));
    std::size_t hovered = NO_PHASE;
    if (ImGui::BeginChild(rows_id, child, ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollWithMouse))
    {
        ImDrawList* draw = ImGui::GetWindowDrawList();
        ImVec2 const origin = ImGui::GetCursorScreenPos();
        float const height = static_cast<float>(rows.size()) * row_h;
        ImU32 const muted = ImGui::GetColorU32(ImGuiCol_TextDisabled);
        ImU32 const text = ImGui::GetColorU32(ImGuiCol_Text);
        ImU32 const grid = ImGui::GetColorU32(ImGuiCol_Separator, 0.30F);
        ImU32 const stripe = ImGui::GetColorU32(ImGuiCol_TableRowBgAlt);
        ImU32 const header = ImGui::GetColorU32(ImGuiCol_TableHeaderBg);
        ImU32 const red = ImGui::GetColorU32(RED);
        Selection const& chosen = p_app.ui.selection;
        robotik::SkillId const selected_skill =
            chosen.kind == SelectionKind::Skill ? skills.find(chosen.name)
                                                : robotik::NO_SKILL;

        // Backgrounds, labels, and the clickable names.
        std::vector<std::size_t> const last = lastPhases(trace, skills.size());
        std::vector<float> skill_y(skills.size(), -1.0F);
        std::vector<float> resource_y(resources.size(), -1.0F);
        std::vector<std::pair<std::string, float>> folded_y;
        float resources_y = -1.0F;
        for (std::size_t r = 0; r < rows.size(); ++r)
        {
            Row const& row = rows[r];
            float const y = origin.y + static_cast<float>(r) * row_h;
            ImVec2 const a(origin.x, y);
            ImVec2 const b(origin.x + full_w, y + row_h);
            bool const is_header =
                row.kind == RowKind::Robot || row.kind == RowKind::Resources;
            if (is_header)
            {
                draw->AddRectFilled(a, b, header);
            }
            else if (r % 2U == 1U)
            {
                draw->AddRectFilled(a, b, stripe);
            }
            bool const chosen_row =
                (row.kind == RowKind::Skill && row.index == selected_skill) ||
                (row.kind == RowKind::Resource &&
                 isSelected(p_app, SelectionKind::Resource, row.label)) ||
                (row.kind == RowKind::Robot &&
                 isSelected(p_app, SelectionKind::Robot, row.robot));
            if (chosen_row)
            {
                draw->AddRectFilled(a, b, ImGui::GetColorU32(ImGuiCol_Header));
            }
            float indent = indentOf(row, robot_headers);
            float const text_y = y + (row_h - ImGui::GetTextLineHeight()) * 0.5F;
            ImU32 color = text;
            if (row.kind == RowKind::TreeFailure)
            {
                color = red;
            }
            else if (row.kind == RowKind::Skill)
            {
                color = ImGui::GetColorU32(
                    ::colorOf(skills.state(static_cast<robotik::SkillId>(row.index))));
            }
            else if (row.kind == RowKind::Resource &&
                     !resources.available(
                         static_cast<robotik::ResourceId>(row.index)))
            {
                color = red;
            }
            if (is_header)
            {
                bool const open = row.kind == RowKind::Robot
                                      ? !isFolded(view, row.robot)
                                      : view.resources_open;
                ImGui::RenderArrow(draw,
                                   ImVec2(origin.x + indent, text_y),
                                   text,
                                   open ? ImGuiDir_Down : ImGuiDir_Right,
                                   0.7F);
                indent += 14.0F;
            }
            draw->PushClipRect(a, ImVec2(axis.x - 4.0F, y + row_h), true);
            draw->AddText(ImVec2(origin.x + indent, text_y), color,
                          row.label.c_str());
            draw->PopClipRect();
            ImGui::SetCursorScreenPos(a);
            ImGui::PushID(static_cast<int>(r));
            if (ImGui::InvisibleButton("name", ImVec2(label_w - 4.0F, row_h)))
            {
                // The arrow folds a robot; its name selects it.
                bool const on_arrow = io.MousePos.x < a.x + indent;
                if (row.kind == RowKind::Robot && on_arrow)
                {
                    toggleFolded(view, row.robot);
                }
                else if (row.kind == RowKind::Robot)
                {
                    selectItem(p_app, SelectionKind::Robot, row.robot);
                }
                else if (row.kind == RowKind::Resources)
                {
                    view.resources_open = !view.resources_open;
                }
                else if (row.kind == RowKind::Resource)
                {
                    selectItem(p_app, SelectionKind::Resource, row.label);
                }
                else if (row.kind == RowKind::Skill)
                {
                    selectItem(p_app, SelectionKind::Skill, row.label);
                    memory.selected = lastPhase(
                        trace, static_cast<robotik::SkillId>(row.index));
                }
            }
            if (row.kind == RowKind::Skill && ImGui::IsItemHovered())
            {
                ImGui::SetTooltip(
                    "%s · %s\n%s",
                    row.label.c_str(),
                    robotik::toString(
                        skills.state(static_cast<robotik::SkillId>(row.index))),
                    row.robot.empty() ? "" : ("robot: " + row.robot).c_str());
            }
            ImGui::PopID();
            switch (row.kind)
            {
                case RowKind::Skill:
                    skill_y[row.index] = y;
                    break;
                case RowKind::Resource:
                    resource_y[row.index] = y;
                    break;
                case RowKind::Robot:
                    if (isFolded(view, row.robot))
                    {
                        folded_y.emplace_back(row.robot, y);
                    }
                    break;
                case RowKind::Resources:
                    resources_y = y;
                    break;
                case RowKind::TreeFailure:
                    break;
            }
        }

        // The lanes: clicks, drags and the wheel act on the time axis.
        ImGui::SetCursorScreenPos(ImVec2(axis.x, origin.y));
        ImGui::InvisibleButton("lanes",
                               ImVec2(axis.width, std::max(height, 1.0F)));
        bool const lanes_hovered = ImGui::IsItemHovered();
        bool const lanes_active = ImGui::IsItemActive();
        bool const released = ImGui::IsItemDeactivated();
        bool const double_clicked =
            lanes_hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left);

        draw->PushClipRect(ImVec2(axis.x, origin.y),
                           ImVec2(axis.x + axis.width, origin.y + height),
                           true);
        // Grid on the ticks of the ruler.
        for (double t = std::floor(axis.t0 / step) * step; t <= axis.t1;
             t += step)
        {
            if (t >= -1e-9)
            {
                float const x = xOf(axis, t);
                draw->AddLine(ImVec2(x, origin.y), ImVec2(x, origin.y + height),
                              grid);
            }
        }
        auto visible = [&](double p_start, double p_end)
        { return p_end >= axis.t0 && p_start <= axis.t1; };
        auto hit = [&](float p_x0, float p_x1, float p_y)
        {
            return lanes_hovered && mouse.y >= p_y && mouse.y < p_y + row_h &&
                   mouse.x >= p_x0 - 2.0F && mouse.x <= std::max(p_x1, p_x0 + 3.0F) + 2.0F;
        };

        // Tree failures: from the first sight to the present.
        for (std::size_t r = 0; r < rows.size(); ++r)
        {
            if (rows[r].kind != RowKind::TreeFailure)
            {
                continue;
            }
            float const y = origin.y + static_cast<float>(r) * row_h;
            TreeFailure const& failure = memory.tree_failures[rows[r].index];
            block(draw, xOf(axis, failure.time), xOf(axis, now), y, row_h,
                  faded(RED, 0.5F), red, text, "failed");
        }

        // Skill phases, and the robots folded into their header.
        for (std::size_t i = 0; i < trace.size(); ++i)
        {
            robotik::SkillRun const& run = trace[i];
            double const end = phaseEnd(skills, run, last[run.skill] == i, now);
            if (!visible(run.start.value(), end))
            {
                continue;
            }
            float y = skill_y[run.skill];
            if (y < 0.0F)
            {
                std::string const& robot = robotOf(cell, run.skill);
                auto const at = std::ranges::find_if(
                    folded_y,
                    [&](auto const& p_pair) { return p_pair.first == robot; });
                if (at == folded_y.end())
                {
                    continue;
                }
                y = at->second;
            }
            float const x0 = xOf(axis, run.start.value());
            float const x1 = xOf(axis, end);
            ImU32 fill = 0;
            ImU32 line = 0;
            phaseColors(run.state, fill, line);
            if (i == memory.selected)
            {
                line = text;
            }
            block(draw, x0, x1, y, row_h, fill, line, inkOn(run.state),
                  phaseText(skills, resources, run));
            if (i == memory.selected)
            {
                draw->AddRect(ImVec2(x0 - 1.0F, y + 2.0F),
                              ImVec2(std::max(x1, x0 + 3.0F) + 1.0F,
                                     y + row_h - 2.0F),
                              text, 3.0F, 0, 2.0F);
            }
            if (hit(x0, x1, y))
            {
                hovered = i;
            }
        }

        // Resources: held by the running phases of the skills needing them.
        for (robotik::ResourceId id = 0; id < resources.size(); ++id)
        {
            float const y = resource_y[id];
            if (y < 0.0F)
            {
                continue;
            }
            for (std::size_t i = 0; i < trace.size(); ++i)
            {
                robotik::SkillRun const& run = trace[i];
                double const end =
                    phaseEnd(skills, run, last[run.skill] == i, now);
                if (run.state == robotik::SkillState::Waiting ||
                    end <= run.start.value() || !visible(run.start.value(), end))
                {
                    continue;
                }
                robotik::ResourceRequirement const* need =
                    needOf(skills, run.skill, id);
                if (need == nullptr)
                {
                    continue;
                }
                ImVec4 const color =
                    need->access == robotik::Access::Shared ? BLUE : ORANGE;
                float const x0 = xOf(axis, run.start.value());
                float const x1 = xOf(axis, end);
                block(draw, x0, x1, y, row_h, faded(color, 0.55F),
                      ImGui::GetColorU32(color),
                      IM_COL32(20, 22, 25, 255), skills.name(run.skill));
                if (hit(x0, x1, y))
                {
                    hovered = i;
                }
            }
        }
        // Faults over the resource rows, or as marks on their header.
        for (FaultSpan const& span : memory.faults)
        {
            float y = resource_y[span.resource];
            bool const thin = y < 0.0F;
            if (thin)
            {
                y = resources_y;
            }
            if (y < 0.0F || !visible(span.start, span.end))
            {
                continue;
            }
            float const x0 = xOf(axis, span.start);
            float const x1 = std::max(xOf(axis, span.end), x0 + 3.0F);
            if (thin)
            {
                draw->AddRectFilled(ImVec2(x0, y + row_h - 5.0F),
                                    ImVec2(x1, y + row_h - 2.0F), red);
            }
            else
            {
                block(draw, x0, x1, y, row_h, faded(RED, 0.7F),
                      red, text, "fault");
            }
            if (lanes_hovered && mouse.y >= y && mouse.y < y + row_h &&
                mouse.x >= x0 && mouse.x <= x1)
            {
                ImGui::SetTooltip("%s in fault from %.2f s%s",
                                  resources.name(span.resource).c_str(),
                                  span.start,
                                  span.open ? " (still)" : "");
            }
        }
        // Who holds what now, on the folded header of the resources.
        if (resources_y >= 0.0F && !view.resources_open)
        {
            std::string const line = holdersLine(skills, resources);
            draw->AddText(ImVec2(axis.x + 6.0F,
                                 resources_y +
                                     (row_h - ImGui::GetTextLineHeight()) * 0.5F),
                          muted, line.c_str());
        }

        // The present, and the time under the mouse.
        float const now_x = xOf(axis, now);
        draw->AddLine(ImVec2(now_x, origin.y), ImVec2(now_x, origin.y + height),
                      text, 1.5F);
        if (hover_t >= 0.0)
        {
            float const x = xOf(axis, hover_t);
            draw->AddLine(ImVec2(x, origin.y), ImVec2(x, origin.y + height),
                          muted);
        }
        draw->PopClipRect();

        if (hovered != NO_PHASE)
        {
            ImGui::SetTooltip("%s", phaseSentence(simulation, hovered).c_str());
        }

        // Mouse on the lanes: wheel zoom, drag and Shift+wheel move.
        double const width_s = axis.t1 - axis.t0;
        double const per_px = width_s / static_cast<double>(axis.width);
        auto freeze = [&]
        {
            view.follow = false;
            view.whole_run = false;
            view.start_s = axis.t0;
            view.window_s = width_s;
        };
        if (lanes_hovered && io.MouseWheel != 0.0F && !io.KeyShift)
        {
            double const t = timeAt(axis, mouse.x);
            double const next =
                std::clamp(width_s * std::pow(0.8, io.MouseWheel),
                           MIN_WINDOW_S, MAX_WINDOW_S);
            if (view.follow && !view.whole_run)
            {
                view.window_s = next;
            }
            else
            {
                freeze();
                view.start_s = t - (t - axis.t0) * next / width_s;
                view.window_s = next;
            }
        }
        float const pan_px = io.KeyShift ? io.MouseWheel * 60.0F + io.MouseWheelH * 60.0F
                                         : io.MouseWheelH * 60.0F;
        if (lanes_hovered && pan_px != 0.0F)
        {
            freeze();
            view.start_s -= static_cast<double>(pan_px) * per_px;
        }
        if (lanes_active && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 3.0F) &&
            io.MouseDelta.x != 0.0F)
        {
            freeze();
            view.start_s -= static_cast<double>(io.MouseDelta.x) * per_px;
        }
        if (!view.follow && !view.whole_run)
        {
            view.start_s = std::clamp(view.start_s, -0.5 * view.window_s,
                                      std::max(now, 0.0));
        }
        if (released && io.MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] < 9.0F)
        {
            memory.selected = hovered;
            if (hovered != NO_PHASE)
            {
                selectItem(p_app, SelectionKind::Skill,
                           skills.name(trace[hovered].skill));
            }
        }
        if (double_clicked)
        {
            view.follow = true;
            view.whole_run = false;
        }
        // Wheel over the names scrolls the rows.
        if (ImGui::IsWindowHovered() && !lanes_hovered && io.MouseWheel != 0.0F)
        {
            ImGui::SetScrollY(ImGui::GetScrollY() - io.MouseWheel * row_h * 3.0F);
        }
        ImGui::SetCursorScreenPos(origin);
        ImGui::Dummy(ImVec2(full_w - ImGui::GetStyle().ScrollbarSize, height));
    }
    ImGui::EndChild();

    // One line of details: the phase hovered or clicked, else the moment.
    std::string line;
    if (hovered != NO_PHASE)
    {
        line = phaseSentence(simulation, hovered);
    }
    else if (memory.selected != NO_PHASE)
    {
        line = phaseSentence(simulation, memory.selected);
    }
    else
    {
        line = momentSentence(simulation, hover_t >= 0.0 ? hover_t : now);
    }
    ImGui::TextUnformatted(line.c_str());
}
