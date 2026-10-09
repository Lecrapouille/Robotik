// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "App.hpp"
#include "SkillView.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <optional>
#include <span>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{

constexpr float kPriorityWidth = 16.0f;
constexpr float kIconWidth = 14.0f;

struct FaultNote
{
    robotik::SkillId skill = robotik::NO_SKILL;
    std::size_t run = 0;
    std::string name;
    std::string resource;
    std::string message;
    std::string when;
};

struct BoardMemory
{
    bool expand_all = false;
    std::optional<bool> force_open;
    std::unordered_set<robotik::SkillId> opened_failures;
    std::vector<FaultNote> faults;
    std::vector<std::size_t> acknowledged;
    const robotik::Simulation* simulation = nullptr;
    std::size_t trace_size = 0;
};

ImU32 colorMuted()
{
    return ImGui::GetColorU32(ImGuiCol_TextDisabled);
}

ImU32 colorPriority(int p_level)
{
    if (p_level >= 3)
    {
        return IM_COL32(255, 154, 92, 255);
    }
    if (p_level == 2)
    {
        return IM_COL32(240, 192, 64, 255);
    }
    return IM_COL32(124, 196, 238, 255);
}

int priorityLevel(robotik::Priority p_priority)
{
    if (p_priority >= 1000)
    {
        return 3;
    }
    if (p_priority >= 100)
    {
        return 2;
    }
    return 1;
}

char const* priorityLabel(int p_level)
{
    if (p_level >= 3)
    {
        return "High";
    }
    if (p_level == 2)
    {
        return "Medium";
    }
    return "Low";
}

std::string formatSeconds(double p_seconds)
{
    if (p_seconds < 0.0)
    {
        p_seconds = 0.0;
    }
    char buffer[32];
    if (p_seconds < 60.0)
    {
        std::snprintf(buffer, sizeof buffer, "%.1f s", p_seconds);
        return buffer;
    }
    int const total = static_cast<int>(p_seconds);
    std::snprintf(buffer, sizeof buffer, "%d min %02d", total / 60, total % 60);
    return buffer;
}

robotik::SkillRun const* latestRun(robotik::SkillScheduler const& p_skills,
                                   robotik::SkillId p_id,
                                   std::size_t* p_index)
{
    std::span<robotik::SkillRun const> const trace = p_skills.trace();
    for (std::size_t i = trace.size(); i > 0; --i)
    {
        if (trace[i - 1].skill == p_id)
        {
            if (p_index != nullptr)
            {
                *p_index = i - 1;
            }
            return &trace[i - 1];
        }
    }
    return nullptr;
}

bool isBlocked(robotik::SkillScheduler const& p_skills, robotik::SkillId p_id)
{
    if (p_skills.state(p_id) != robotik::SkillState::Waiting)
    {
        return false;
    }
    robotik::SkillReason const reason = p_skills.reason(p_id);
    return reason == robotik::SkillReason::Busy ||
           reason == robotik::SkillReason::Unavailable;
}

bool terminal(robotik::SkillState p_state)
{
    return p_state == robotik::SkillState::Succeeded ||
           p_state == robotik::SkillState::Failed ||
           p_state == robotik::SkillState::Cancelled;
}

float chipWidth(std::string const& p_text)
{
    return ImGui::CalcTextSize(p_text.c_str()).x + 12.0f;
}

void chip(std::string const& p_text)
{
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec2 const at = ImGui::GetCursorScreenPos();
    ImVec2 const size(chipWidth(p_text), ImGui::GetTextLineHeight());
    ImGui::Dummy(size);
    draw->AddRect(at,
                  ImVec2(at.x + size.x, at.y + size.y),
                  colorMuted(),
                  size.y * 0.5f,
                  0,
                  1.2f);
    draw->AddText(ImVec2(at.x + 6.0f, at.y), colorMuted(), p_text.c_str());
}

void priorityBars(int p_level, robotik::Priority p_value)
{
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec2 const at = ImGui::GetCursorScreenPos();
    float const height = ImGui::GetTextLineHeight();
    ImGui::Dummy(ImVec2(kPriorityWidth, height));
    ImU32 const on = colorPriority(p_level);
    ImU32 const off = (on & 0x00FFFFFFu) | (70u << 24);
    for (int bar = 0; bar < 3; ++bar)
    {
        float const bar_height = 6.0f + 4.0f * static_cast<float>(bar);
        ImVec2 const corner(at.x + static_cast<float>(bar) * 6.0f,
                            at.y + height - bar_height - 1.0f);
        bool const lit = bar < p_level;
        draw->AddRectFilled(corner,
                            ImVec2(corner.x + 4.0f, at.y + height - 1.0f),
                            lit ? on : off,
                            1.5f);
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip(
            "%s (%d)", priorityLabel(p_level), static_cast<int>(p_value));
    }
}

void stateIcon(robotik::SkillState p_state,
               robotik::SkillReason p_reason,
               bool p_blocked)
{
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec2 const at = ImGui::GetCursorScreenPos();
    float const height = ImGui::GetTextLineHeight();
    float const radius = 5.0f;
    ImGui::Dummy(ImVec2(kIconWidth, height));
    ImVec2 const center(at.x + kIconWidth * 0.5f, at.y + height * 0.5f);
    ImU32 const color = ImGui::GetColorU32(colorOf(p_state));
    switch (p_state)
    {
        case robotik::SkillState::Waiting:
            if (p_blocked)
            {
                draw->AddTriangle(ImVec2(center.x - radius, center.y - radius),
                                  ImVec2(center.x + radius, center.y - radius),
                                  center,
                                  color,
                                  1.5f);
                draw->AddTriangle(ImVec2(center.x - radius, center.y + radius),
                                  ImVec2(center.x + radius, center.y + radius),
                                  center,
                                  color,
                                  1.5f);
            }
            else
            {
                draw->AddCircle(center, radius, color, 0, 1.5f);
            }
            break;
        case robotik::SkillState::Running:
        {
            float const angle = static_cast<float>(ImGui::GetTime()) * 5.0f;
            draw->PathArcTo(center, radius, angle, angle + 4.2f);
            draw->PathStroke(color, 0, 2.0f);
            break;
        }
        case robotik::SkillState::Cancelled:
            if (p_reason == robotik::SkillReason::Preempted)
            {
                draw->AddRectFilled(
                    ImVec2(center.x - radius, center.y - radius),
                    ImVec2(center.x - 1.5f, center.y + radius),
                    color);
                draw->AddRectFilled(
                    ImVec2(center.x + 1.5f, center.y - radius),
                    ImVec2(center.x + radius, center.y + radius),
                    color);
                break;
            }
            draw->AddCircle(center, radius, color, 0, 1.5f);
            draw->AddLine(ImVec2(center.x - radius, center.y + radius),
                          ImVec2(center.x + radius, center.y - radius),
                          color,
                          1.5f);
            break;
        case robotik::SkillState::Succeeded:
        {
            ImVec2 const points[3] = {
                ImVec2(center.x - radius, center.y),
                ImVec2(center.x - 1.5f, center.y + radius - 1.5f),
                ImVec2(center.x + radius, center.y - radius + 1.0f)
            };
            draw->AddPolyline(points, 3, color, 0, 2.0f);
            break;
        }
        case robotik::SkillState::Failed:
            draw->AddLine(ImVec2(center.x - radius, center.y - radius),
                          ImVec2(center.x + radius, center.y + radius),
                          color,
                          2.0f);
            draw->AddLine(ImVec2(center.x - radius, center.y + radius),
                          ImVec2(center.x + radius, center.y - radius),
                          color,
                          2.0f);
            break;
        case robotik::SkillState::Idle:
            draw->AddCircle(center, radius, color, 0, 1.5f);
            break;
    }
}

void dashedRect(ImDrawList* p_draw, ImVec2 p_a, ImVec2 p_b, ImU32 p_color)
{
    auto line = [&](ImVec2 p_from, ImVec2 p_to)
    {
        float const length = std::hypot(p_to.x - p_from.x, p_to.y - p_from.y);
        if (length < 1.0f)
        {
            return;
        }
        ImVec2 const step((p_to.x - p_from.x) / length,
                          (p_to.y - p_from.y) / length);
        for (float along = 0.0f; along < length; along += 10.0f)
        {
            float const end = std::min(along + 6.0f, length);
            p_draw->AddLine(
                ImVec2(p_from.x + step.x * along, p_from.y + step.y * along),
                ImVec2(p_from.x + step.x * end, p_from.y + step.y * end),
                p_color,
                1.5f);
        }
    };
    line(p_a, ImVec2(p_b.x, p_a.y));
    line(ImVec2(p_b.x, p_a.y), p_b);
    line(p_b, ImVec2(p_a.x, p_b.y));
    line(ImVec2(p_a.x, p_b.y), p_a);
}

std::string statusText(robotik::Simulation const& p_simulation,
                       robotik::SkillId p_id)
{
    robotik::SkillScheduler const& skills = p_simulation.skills();
    robotik::SkillState const state = skills.state(p_id);
    robotik::SkillReason const reason = skills.reason(p_id);
    if (state == robotik::SkillState::Waiting)
    {
        return "waiting";
    }
    robotik::SkillRun const* run = latestRun(skills, p_id, nullptr);
    if (run == nullptr)
    {
        return {};
    }
    double const end = state == robotik::SkillState::Running
                           ? p_simulation.time().value()
                           : run->end.value();
    std::string text = formatSeconds(end - run->start.value());
    if (state == robotik::SkillState::Cancelled &&
        reason == robotik::SkillReason::Preempted)
    {
        text = "preempted · " + text;
    }
    return text;
}

void rememberFaults(BoardMemory& p_memory,
                    robotik::Simulation const& p_simulation)
{
    robotik::SkillScheduler const& skills = p_simulation.skills();
    if (p_memory.simulation != &p_simulation ||
        skills.trace().size() < p_memory.trace_size)
    {
        p_memory.faults.clear();
        p_memory.acknowledged.clear();
        p_memory.opened_failures.clear();
        p_memory.simulation = &p_simulation;
    }
    p_memory.trace_size = skills.trace().size();

    for (robotik::SkillId id = 0; id < skills.size(); ++id)
    {
        if (skills.state(id) != robotik::SkillState::Failed)
        {
            continue;
        }
        std::size_t index = 0;
        robotik::SkillRun const* run = latestRun(skills, id, &index);
        if (run == nullptr)
        {
            continue;
        }
        bool const known = std::any_of(p_memory.faults.begin(),
                                       p_memory.faults.end(),
                                       [&](FaultNote const& p_note)
                                       { return p_note.run == index; }) ||
                           std::find(p_memory.acknowledged.begin(),
                                     p_memory.acknowledged.end(),
                                     index) != p_memory.acknowledged.end();
        if (known)
        {
            continue;
        }
        FaultNote note;
        note.skill = id;
        note.run = index;
        note.name = skills.name(id);
        robotik::ResourceId const blocker = skills.blocker(id);
        if (blocker != robotik::NO_RESOURCE)
        {
            note.resource = p_simulation.robot().resources().name(blocker);
        }
        note.message = skillWhy(skills, p_simulation.robot().resources(), id);
        note.when = formatSeconds(run->end.value());
        p_memory.faults.push_back(std::move(note));
    }
}

void toolbar(BoardMemory& p_memory)
{
    if (ImGui::Button(p_memory.expand_all ? "Collapse all" : "Expand all"))
    {
        p_memory.expand_all = !p_memory.expand_all;
        p_memory.force_open = p_memory.expand_all;
    }
    for (int level : { 3, 2, 1 })
    {
        ImGui::SameLine(0.0f, 12.0f);
        priorityBars(level, level >= 3 ? 1000 : level == 2 ? 100 : 0);
        ImGui::SameLine(0.0f, 4.0f);
        ImGui::TextDisabled("%s", priorityLabel(level));
    }
}

void faultsSection(BoardMemory& p_memory)
{
    if (p_memory.faults.empty())
    {
        return;
    }
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Text, RED);
    bool const open = ImGui::CollapsingHeader(
        ("Faults to acknowledge (" + std::to_string(p_memory.faults.size()) +
         ")###faults")
            .c_str());
    ImGui::PopStyleColor();
    if (!open)
    {
        return;
    }
    int acknowledge = -1;
    for (std::size_t i = 0; i < p_memory.faults.size(); ++i)
    {
        FaultNote const& note = p_memory.faults[i];
        ImGui::PushID(static_cast<int>(i));
        ImGui::BulletText("%s", note.name.c_str());
        if (!note.resource.empty())
        {
            ImGui::SameLine();
            chip(note.resource);
        }
        ImGui::SameLine();
        ImGui::TextUnformatted(note.message.c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("%s", note.when.c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("Acknowledge"))
        {
            acknowledge = static_cast<int>(i);
        }
        ImGui::PopID();
    }
    if (acknowledge >= 0)
    {
        std::size_t const index = static_cast<std::size_t>(acknowledge);
        p_memory.acknowledged.push_back(p_memory.faults[index].run);
        p_memory.faults.erase(p_memory.faults.begin() +
                              static_cast<std::ptrdiff_t>(index));
    }
}

void card(BoardMemory& p_memory,
          robotik::Simulation& p_simulation,
          robotik::SkillId p_id,
          bool p_blocked)
{
    robotik::SkillScheduler& skills = p_simulation.skills();
    robotik::SkillDescription const& description = skills.description(p_id);
    robotik::SkillState const state = skills.state(p_id);
    robotik::SkillReason const reason = skills.reason(p_id);
    robotik::ResourceManager const& resources =
        p_simulation.robot().resources();
    int const level = priorityLevel(description.priority);

    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec2 const origin = ImGui::GetCursorScreenPos();
    float const width = ImGui::GetContentRegionAvail().x;
    ImDrawListSplitter split;
    split.Split(draw, 2);
    split.SetCurrentChannel(draw, 1);
    ImGui::Dummy(ImVec2(0.0f, 2.0f));
    ImGui::Indent(8.0f);
    ImGui::PushID(static_cast<int>(p_id));

    if (p_memory.force_open)
    {
        ImGui::SetNextItemOpen(*p_memory.force_open, ImGuiCond_Always);
    }
    else if (state == robotik::SkillState::Failed &&
             p_memory.opened_failures.insert(p_id).second)
    {
        ImGui::SetNextItemOpen(true, ImGuiCond_Always);
    }

    float const row_x = ImGui::GetCursorPosX();
    float const available = ImGui::GetContentRegionAvail().x;
    bool const open = ImGui::TreeNodeEx(description.name.c_str());

    bool const waiting = state == robotik::SkillState::Waiting;
    std::vector<std::string> labels;
    if (!waiting)
    {
        for (robotik::ResourceRequirement const& need : description.resources)
        {
            labels.push_back(resources.name(need.id));
        }
    }
    std::string const status = statusText(p_simulation, p_id);
    bool const active = state == robotik::SkillState::Running || waiting;
    char const* const action = active ? "Cancel" : "Request";
    float const action_w =
        ImGui::CalcTextSize(action).x + ImGui::GetStyle().FramePadding.x * 2.0f;
    constexpr float kGap = 6.0f;
    float cluster = kIconWidth + kPriorityWidth + kGap + action_w + kGap;
    if (!status.empty())
    {
        cluster += 4.0f + ImGui::CalcTextSize(status.c_str()).x;
    }
    for (std::string const& label : labels)
    {
        cluster += chipWidth(label) + kGap;
    }
    float const right = row_x + available - cluster - 8.0f;
    if (right > ImGui::GetCursorPosX())
    {
        ImGui::SameLine(right);
    }
    else
    {
        ImGui::SameLine(0.0f, kGap);
    }
    for (std::string const& label : labels)
    {
        chip(label);
        ImGui::SameLine(0.0f, kGap);
    }
    priorityBars(level, description.priority);
    ImGui::SameLine(0.0f, kGap);
    stateIcon(state, reason, p_blocked);
    if (!status.empty())
    {
        ImGui::SameLine(0.0f, 4.0f);
        ImGui::PushStyleColor(ImGuiCol_Text, colorOf(state));
        ImGui::TextUnformatted(status.c_str());
        ImGui::PopStyleColor();
    }
    ImGui::SameLine(0.0f, kGap);
    if (ImGui::SmallButton(action))
    {
        if (active)
        {
            skills.cancel(p_id);
        }
        else
        {
            skills.request(p_id);
        }
    }

    std::string const why = skillWhy(skills, resources, p_id);
    if (state == robotik::SkillState::Failed && !why.empty())
    {
        ImGui::PushStyleColor(ImGuiCol_Text, RED);
        ImGui::Text("    Fault: %s", why.c_str());
        ImGui::PopStyleColor();
    }
    else if (waiting && !why.empty())
    {
        ImGui::PushStyleColor(ImGuiCol_Text, colorOf(state));
        ImGui::TextUnformatted(why.c_str());
        ImGui::PopStyleColor();
    }

    if (open)
    {
        ImGui::TextDisabled("Priority: %s (%d)%s",
                            priorityLabel(level),
                            static_cast<int>(description.priority),
                            description.cancellable ? "" : " · locked");
        if (!why.empty() && state != robotik::SkillState::Failed && !waiting)
        {
            ImGui::TextDisabled("%s", why.c_str());
        }
        if (!description.preconditions.empty())
        {
            ImGui::TextDisabled("Preconditions:");
            for (robotik::Precondition const& precondition :
                 description.preconditions)
            {
                ImGui::BulletText("%s", precondition.text.c_str());
            }
        }
        ImGui::TreePop();
    }

    ImGui::PopID();
    ImGui::Unindent(8.0f);
    ImGui::Dummy(ImVec2(0.0f, 2.0f));
    ImVec2 const end(origin.x + width, ImGui::GetCursorScreenPos().y);

    split.SetCurrentChannel(draw, 0);
    draw->AddRectFilled(origin, end, IM_COL32(255, 255, 255, 8), 8.0f);
    ImU32 const border = ImGui::GetColorU32(colorOf(state));
    bool const dashed = state == robotik::SkillState::Waiting ||
                        state == robotik::SkillState::Cancelled ||
                        state == robotik::SkillState::Idle;
    if (dashed)
    {
        dashedRect(draw, origin, end, border);
    }
    else
    {
        draw->AddRect(origin, end, border, 8.0f, 0, 1.5f);
    }
    if (state == robotik::SkillState::Failed)
    {
        draw->AddRect(ImVec2(origin.x + 3.0f, origin.y + 3.0f),
                      ImVec2(end.x - 3.0f, end.y - 3.0f),
                      border,
                      6.0f,
                      0,
                      1.5f);
    }
    split.Merge(draw);
    ImGui::Dummy(ImVec2(0.0f, 6.0f));
}

void section(char const* p_title,
             char const* p_empty,
             std::vector<robotik::SkillId> const& p_ids,
             BoardMemory& p_memory,
             robotik::Simulation& p_simulation,
             robotik::SkillScheduler const& p_skills,
             bool p_open)
{
    if (p_open)
    {
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    }
    std::string const title = std::string(p_title) + " (" +
                              std::to_string(p_ids.size()) + ")###" + p_title;
    if (!ImGui::CollapsingHeader(title.c_str()))
    {
        return;
    }
    if (p_ids.empty())
    {
        ImGui::TextDisabled("%s", p_empty);
        return;
    }
    for (robotik::SkillId const id : p_ids)
    {
        card(p_memory, p_simulation, id, isBlocked(p_skills, id));
    }
}

robotik::SkillId lastFinished(robotik::SkillScheduler const& p_skills)
{
    robotik::SkillId last = robotik::NO_SKILL;
    double last_end = -1.0;
    for (robotik::SkillRun const& run : p_skills.trace())
    {
        if (!terminal(run.state) || !terminal(p_skills.state(run.skill)))
        {
            continue;
        }
        if (run.end.value() >= last_end)
        {
            last_end = run.end.value();
            last = run.skill;
        }
    }
    return last;
}

} // namespace

void skillsBoard(App& p_app)
{
    if (!ImGui::Begin(SKILLS_PANEL) || !p_app.simulation)
    {
        ImGui::End();
        return;
    }

    static BoardMemory memory;
    robotik::Simulation& simulation = *p_app.simulation;
    robotik::SkillScheduler& skills = simulation.skills();
    rememberFaults(memory, simulation);
    toolbar(memory);
    faultsSection(memory);

    robotik::SkillId const last = lastFinished(skills);
    std::vector<robotik::SkillId> running;
    std::vector<robotik::SkillId> waiting;
    std::vector<robotik::SkillId> idle;
    for (robotik::SkillId id = 0; id < skills.size(); ++id)
    {
        robotik::SkillState const state = skills.state(id);
        if (state == robotik::SkillState::Running)
        {
            running.push_back(id);
        }
        else if (state == robotik::SkillState::Waiting)
        {
            waiting.push_back(id);
        }
        else if (id != last)
        {
            idle.push_back(id);
        }
    }
    auto const by_priority = [&](robotik::SkillId p_a, robotik::SkillId p_b)
    {
        robotik::Priority const a = skills.description(p_a).priority;
        robotik::Priority const b = skills.description(p_b).priority;
        return a != b ? a > b : p_a < p_b;
    };
    std::stable_sort(running.begin(), running.end(), by_priority);
    std::stable_sort(waiting.begin(), waiting.end(), by_priority);

    section("RUNNING",
            "Robot is idle",
            running,
            memory,
            simulation,
            skills,
            true);
    section("WAITING",
            "Queue is empty",
            waiting,
            memory,
            simulation,
            skills,
            true);
    std::vector<robotik::SkillId> const finished =
        last == robotik::NO_SKILL
            ? std::vector<robotik::SkillId>{}
            : std::vector<robotik::SkillId>{ last };
    section("LAST RUN",
            "Nothing has run yet",
            finished,
            memory,
            simulation,
            skills,
            true);
    section("IDLE",
            "Every skill is busy or shown above",
            idle,
            memory,
            simulation,
            skills,
            false);

    memory.force_open.reset();
    ImGui::End();
}
