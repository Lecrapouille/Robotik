// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file EventsPanel.cpp
/// The Events panel: what happened in the run, one dated line each.
///
/// The skills that wait, start and end (and why), the resources that fall in
/// fault and come back, the composites of the tree that fail, the plans and
/// the steps of the plan, the verdicts of the assertions. Filters keep a kind
/// or the problems only; a text filters the lines. A click on a time shows
/// it in the Timeline, a click on a name selects it (Inspector).
///
/// @par Contract
/// Reads the scheduler trace, the planner history, the checks and what
/// sampleTimeline recorded; never commands them. The list is built again
/// at each frame from these sources: nothing is kept that they do not keep.

#include "SkillView.hpp"
#include "Ui.hpp"

#include "RobotIK/Package/Simulation.hpp"
#include "RobotIK/Task/Planning/PlanExecutor.hpp"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <span>
#include <string>
#include <vector>

/// "busy: gripper", "precondition", or the reason alone.
static std::string reasonText(robotik::ResourceManager const& p_resources,
                              robotik::SkillRun const& p_run)
{
    std::string text = robotik::toString(p_run.reason);
    if (p_run.blocker != robotik::NO_RESOURCE)
    {
        text += ": " + p_resources.name(p_run.blocker);
    }
    return text;
}

/// The phases of the skills: a wait, a start, an end.
static void skillEvents(robotik::Simulation const& p_simulation,
                        std::vector<RunEvent>& p_events)
{
    robotik::SkillScheduler const& skills = p_simulation.cell().skills();
    robotik::ResourceManager const& resources =
        p_simulation.cell().robot().resources();
    std::span<robotik::SkillRun const> const trace = skills.trace();
    std::vector<std::size_t> last(skills.size(), trace.size());
    for (std::size_t i = 0; i < trace.size(); ++i)
    {
        last[trace[i].skill] = i;
    }
    // Last line of each skill: the same line again only counts it.
    std::vector<std::size_t> previous(skills.size(), static_cast<std::size_t>(-1));
    for (std::size_t i = 0; i < trace.size(); ++i)
    {
        robotik::SkillRun const& run = trace[i];
        std::string const& name = skills.name(run.skill);
        auto add = [&](double p_time, bool p_problem, ImVec4 const& p_color,
                       std::string p_text)
        {
            std::size_t& before = previous[run.skill];
            if (before < p_events.size() && p_events[before].text == p_text)
            {
                p_events[before].count += 1U;
                p_events[before].until = p_time;
                return;
            }
            before = p_events.size();
            p_events.push_back({ p_time, RunEventKind::Skill, p_problem, p_color,
                                 std::move(p_text), SelectionKind::Skill,
                                 name, 1U, p_time });
        };
        double const start = run.start.value();
        double const lasted = run.end.value() - start;
        switch (run.state)
        {
            case robotik::SkillState::Waiting:
                add(start,
                    run.reason == robotik::SkillReason::Unavailable,
                    BLUE,
                    name + " waits" +
                        (run.reason == robotik::SkillReason::None
                             ? std::string()
                             : " (" + reasonText(resources, run) + ")"));
                continue;
            case robotik::SkillState::Idle:
                continue;
            case robotik::SkillState::Running:
            case robotik::SkillState::Stopping:
            case robotik::SkillState::Succeeded:
            case robotik::SkillState::Failed:
            case robotik::SkillState::Cancelled:
                break;
        }
        // A phase that ends where it starts never ran: refused at once.
        bool const refused = lasted <= 0.0 &&
                             (run.reason == robotik::SkillReason::Busy ||
                              run.reason == robotik::SkillReason::Unavailable ||
                              run.reason == robotik::SkillReason::Precondition);
        if (!refused)
        {
            add(start, false, ORANGE, name + " starts");
        }
        // Still running or stopping: no end yet.
        if (run.state == robotik::SkillState::Running ||
            run.state == robotik::SkillState::Stopping)
        {
            continue;
        }
        char took[48];
        std::snprintf(took, sizeof took, " after %.2f s", lasted);
        switch (run.state)
        {
            case robotik::SkillState::Succeeded:
                add(run.end.value(), false, GREEN, name + " succeeded" + took);
                break;
            case robotik::SkillState::Failed:
            {
                std::string why = reasonText(resources, run);
                if (last[run.skill] == i &&
                    run.reason == robotik::SkillReason::Failed &&
                    !skills.skill(run.skill).why().empty())
                {
                    why = skills.skill(run.skill).why();
                }
                add(run.end.value(), true, RED,
                    name + (refused ? " refused (" : " failed (") + why + ")");
                break;
            }
            case robotik::SkillState::Cancelled:
                add(run.end.value(),
                    run.reason == robotik::SkillReason::ResourceLost,
                    GREY,
                    name + " cancelled (" + reasonText(resources, run) + ")");
                break;
            case robotik::SkillState::Idle:
            case robotik::SkillState::Waiting:
            case robotik::SkillState::Running:
            case robotik::SkillState::Stopping:
                break;
        }
    }
}

/// Faults of the resources, failures of the tree, plans, steps, verdicts.
static void otherEvents(App const& p_app, std::vector<RunEvent>& p_events)
{
    robotik::Simulation const& simulation = *p_app.simulation;
    robotik::ResourceManager const& resources =
        simulation.cell().robot().resources();
    TimelineMemory const& memory = p_app.ui.timeline_memory;
    for (FaultSpan const& span : memory.faults)
    {
        std::string const& name = resources.name(span.resource);
        p_events.push_back({ span.start, RunEventKind::Resource, true, RED,
                             name + " in fault", SelectionKind::Resource,
                             name });
        if (!span.open)
        {
            p_events.push_back({ span.end, RunEventKind::Resource, false, GREEN,
                                 name + " restored", SelectionKind::Resource,
                                 name });
        }
    }
    for (TreeFailure const& failure : memory.tree_failures)
    {
        p_events.push_back({ failure.time, RunEventKind::Plan, true, RED,
                             failure.label + " of the tree failed",
                             SelectionKind::None, {} });
    }
    if (robotik::PlanExecutor const* executor = simulation.cell().planner())
    {
        std::vector<robotik::PlanningRecord> const& history =
            executor->history();
        for (std::size_t i = 0; i < history.size(); ++i)
        {
            robotik::PlanningRecord const& record = history[i];
            bool const solved = robotik::solved(record.result);
            char text[96];
            std::snprintf(text,
                          sizeof text,
                          "Plan %zu: %s, %zu steps (",
                          i + 1U,
                          robotik::toString(record.result.status),
                          record.result.steps.size());
            p_events.push_back({ record.time.value(), RunEventKind::Plan,
                                 !solved, solved ? BLUE : RED,
                                 text + record.cause + ")",
                                 SelectionKind::None, {} });
        }
    }
    for (StepMark const& mark : memory.steps)
    {
        char text[64];
        std::snprintf(text, sizeof text, "Step %zu of plan %zu ", mark.step,
                      mark.plan);
        std::string const word =
            mark.failed ? "failed" : mark.skipped ? "skipped" : "done";
        p_events.push_back({ mark.time, RunEventKind::Plan, mark.failed,
                             mark.failed    ? RED
                             : mark.skipped ? GREY
                                            : GREEN,
                             text + word + ": " + mark.action,
                             SelectionKind::None, {} });
    }
    for (robotik::Check const& check : simulation.checks())
    {
        if (check.verdict == robotik::Check::Verdict::Pending)
        {
            continue;
        }
        bool const failed = check.verdict == robotik::Check::Verdict::Failed;
        std::string text = (failed ? "Assertion failed: " : "Assertion holds: ") +
                           check.text;
        if (failed && !check.detail.empty())
        {
            text += " (" + check.detail + ")";
        }
        p_events.push_back({ check.time.value(), RunEventKind::Verdict, failed,
                             failed ? RED : GREEN, std::move(text),
                             SelectionKind::None, {} });
    }
}

/// True when @p_text holds @p_part, whatever the case.
static bool contains(std::string const& p_text, char const* p_part)
{
    if (p_part[0] == '\0')
    {
        return true;
    }
    auto const lower = [](unsigned char p_c)
    { return static_cast<char>(std::tolower(p_c)); };
    std::string text(p_text.size(), ' ');
    std::ranges::transform(p_text, text.begin(), lower);
    std::string part(p_part);
    std::ranges::transform(part, part.begin(), lower);
    return text.find(part) != std::string::npos;
}

/// Kept by the filters.
static bool kept(EventsView const& p_view, RunEvent const& p_event)
{
    bool const kind = (p_event.kind == RunEventKind::Skill && p_view.skills) ||
                      (p_event.kind == RunEventKind::Resource &&
                       p_view.resources) ||
                      (p_event.kind == RunEventKind::Plan && p_view.plan) ||
                      (p_event.kind == RunEventKind::Verdict && p_view.verdicts);
    return kind && (!p_view.problems_only || p_event.problem) &&
           contains(p_event.text, p_view.filter.data());
}

/// The Timeline shows time @p_time in the middle of its window.
static void showInTimeline(App& p_app, double p_time)
{
    TimelineView& view = p_app.ui.timeline;
    view.follow = false;
    view.whole_run = false;
    view.start_s = std::max(-0.5 * view.window_s, p_time - 0.5 * view.window_s);
}

/// Filters above the list.
static void filters(EventsView& p_view, std::size_t p_shown, std::size_t p_all)
{
    ImGui::Checkbox("Skills", &p_view.skills);
    ImGui::SameLine();
    ImGui::Checkbox("Resources", &p_view.resources);
    ImGui::SameLine();
    ImGui::Checkbox("Plan and tree", &p_view.plan);
    ImGui::SameLine();
    ImGui::Checkbox("Assertions", &p_view.verdicts);
    ImGui::SameLine(0.0F, 18.0F);
    ImGui::Checkbox("Problems only", &p_view.problems_only);
    ImGui::SetItemTooltip("Failures, faults, refused skills, failed "
                          "assertions.");
    ImGui::SameLine(0.0F, 18.0F);
    ImGui::SetNextItemWidth(160.0F);
    ImGui::InputTextWithHint("##filter", "Filter", p_view.filter.data(),
                             p_view.filter.size());
    ImGui::SameLine();
    ImGui::TextDisabled("%zu / %zu", p_shown, p_all);
}

std::vector<RunEvent> runEvents(App const& p_app, bool p_skills)
{
    std::vector<RunEvent> events;
    if (!p_app.simulation)
    {
        return events;
    }
    if (p_skills)
    {
        skillEvents(*p_app.simulation, events);
    }
    otherEvents(p_app, events);
    std::ranges::stable_sort(events,
                             [](RunEvent const& p_a, RunEvent const& p_b)
                             { return p_a.time < p_b.time; });
    return events;
}

void eventsView(App& p_app)
{
    if (!p_app.simulation)
    {
        emptyState("No scenario loaded.");
        return;
    }
    std::vector<RunEvent> const events = runEvents(p_app, true);
    EventsView& view = p_app.ui.events;
    std::vector<RunEvent const*> shown;
    for (RunEvent const& event : events)
    {
        if (kept(view, event))
        {
            shown.push_back(&event);
        }
    }
    filters(view, shown.size(), events.size());

    if (!beginTable("events",
                    2,
                    ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                        ImGuiTableFlags_BordersInnerV,
                    ImVec2(0.0F, 0.0F)))
    {
        return;
    }
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed,
                            ImGui::CalcTextSize("0000.00 s").x);
    ImGui::TableSetupColumn("What happened",
                            ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableHeadersRow();
    // The list follows the new lines until the operator scrolls up, and
    // again once back at the bottom.
    bool const at_end = ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 2.0F;
    if (ImGui::IsWindowHovered() &&
        (ImGui::GetIO().MouseWheel > 0.0F ||
         (ImGui::IsMouseDown(ImGuiMouseButton_Left) && !at_end)))
    {
        view.follow = false;
    }
    else if (at_end)
    {
        view.follow = true;
    }
    ImGuiListClipper clipper;
    clipper.Begin(static_cast<int>(shown.size()));
    while (clipper.Step())
    {
        for (int row = clipper.DisplayStart; row < clipper.DisplayEnd; ++row)
        {
            RunEvent const& event = *shown[static_cast<std::size_t>(row)];
            ImGui::PushID(row);
            ImGui::TableNextRow();
            if (event.item != SelectionKind::None &&
                isSelected(p_app, event.item, event.name))
            {
                ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1,
                                       ImGui::GetColorU32(ImGuiCol_Header));
            }
            ImGui::TableNextColumn();
            char time[32];
            std::snprintf(time, sizeof time, "%.2f s", event.time);
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  ImGui::GetStyleColorVec4(
                                      ImGuiCol_TextDisabled));
            if (ImGui::Selectable(time))
            {
                showInTimeline(p_app, event.time);
            }
            ImGui::PopStyleColor();
            ImGui::SetItemTooltip("Show this time in the Timeline.");
            ImGui::TableNextColumn();
            ImGui::PushStyleColor(ImGuiCol_Text, event.color);
            std::string text = event.text;
            if (event.count > 1U)
            {
                char again[64];
                std::snprintf(again, sizeof again, "   x%zu, until %.2f s",
                              event.count, event.until);
                text += again;
            }
            if (event.item != SelectionKind::None)
            {
                if (ImGui::Selectable(text.c_str()))
                {
                    selectItem(p_app, event.item, event.name);
                }
                ImGui::SetItemTooltip("Select %s (Inspector, Timeline).",
                                      event.name.c_str());
            }
            else
            {
                ImGui::TextUnformatted(text.c_str());
            }
            ImGui::PopStyleColor();
            ImGui::PopID();
        }
    }
    if (view.follow)
    {
        ImGui::SetScrollY(ImGui::GetScrollMaxY());
    }
    endTable();
}

void eventsPanel(App& p_app)
{
    if (!beginPanel(p_app, PanelKind::Events, "Events"))
    {
        ImGui::End();
        return;
    }
    eventsView(p_app);
    ImGui::End();
}
