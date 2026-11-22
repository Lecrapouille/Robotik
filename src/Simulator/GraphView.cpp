// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file GraphView.cpp
/// The execution graph: from the intention to the movement, as boxes and
/// arrows read from left to right.
///
/// With a PDDL planner, one box per step of the plan: its action, where it
/// is (done, running, waiting for which steps, failed) and the skills it
/// runs, each in the color of its state. An arrow goes from a step to the
/// steps that wait for it, so the steps that run together are in the same
/// column. Without a planner, the same boxes draw the behavior tree, the
/// root on the left and its actions on the right.
///
/// @par Contract
/// Reads the planner, the tree and the scheduler; never commands them. A
/// click on a skill selects it (Inspector, Timeline). Dragging the
/// background moves the graph; the wheel scrolls it.

#include "SkillView.hpp"
#include "Ui.hpp"

#include "RobotIK/Package/Simulation.hpp"
#include "RobotIK/Task/Planning/PlanExecutor.hpp"

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

/// Space between two columns and two boxes of a column, and the margin.
static constexpr float COLUMN_GAP = 56.0F;
static constexpr float ROW_GAP = 14.0F;
static constexpr float MARGIN = 16.0F;
/// Narrowest and widest box.
static constexpr float MIN_WIDTH = 170.0F;
static constexpr float MAX_WIDTH = 340.0F;

// ****************************************************************************
/// One box of the graph: a step of the plan or a node of the tree.
// ****************************************************************************
struct Box
{
    std::string title;
    /// Where it is, in words and in color.
    std::string status;
    ImVec4 color;
    /// Skills it runs (a step of the plan); a skill node of the tree is
    /// one skill itself (@ref skill).
    std::vector<std::string> skills;
    std::string skill;
    /// Priority and resources of each skill of @ref skills, or of
    /// @ref skill: "p100  arm gripper".
    std::vector<std::string> details;
    std::string detail;
    /// Shown when the mouse is over the box.
    std::string tip;
    /// Column; for the tree, the row too (a parent in the row of its first
    /// child), so that a node stays next to its children.
    std::size_t column = 0;
    float row = -1.0F;
    ImVec2 pos;
    ImVec2 size;
};

// ****************************************************************************
/// An arrow from box @c from to box @c to.
// ****************************************************************************
struct Arrow
{
    std::size_t from = 0;
    std::size_t to = 0;
};

/// Priority and resources of the skill @p_name, in a few characters; empty
/// for a name that is not a skill.
static std::string detailOf(App const& p_app, std::string const& p_name)
{
    robotik::RoboticCell const& cell = p_app.simulation->cell();
    robotik::SkillId const id = cell.skills().find(p_name);
    if (id == robotik::NO_SKILL)
    {
        return {};
    }
    robotik::SkillDescription const& description =
        cell.skills().description(id);
    std::string text = "p" + std::to_string(description.priority);
    for (robotik::ResourceRequirement const& need : description.resources)
    {
        text += (&need == &description.resources.front() ? "  " : " ") +
                cell.robot().resources().name(need.id);
    }
    return text;
}

/// Height of one line of text in a box.
static float lineHeight()
{
    return ImGui::GetTextLineHeight() + 4.0F;
}

/// Size of a box from its texts.
static ImVec2 measure(Box const& p_box)
{
    float width = std::max(ImGui::CalcTextSize(p_box.title.c_str()).x,
                           ImGui::CalcTextSize(p_box.status.c_str()).x);
    for (std::size_t k = 0; k < p_box.skills.size(); ++k)
    {
        float const detail =
            k < p_box.details.size()
                ? ImGui::CalcTextSize(p_box.details[k].c_str()).x + 10.0F
                : 0.0F;
        width = std::max(width,
                         ImGui::CalcTextSize(p_box.skills[k].c_str()).x +
                             14.0F + detail);
    }
    width = std::max(width, ImGui::CalcTextSize(p_box.detail.c_str()).x);
    width = std::clamp(width + 20.0F, MIN_WIDTH, MAX_WIDTH);
    float const lines = 2.0F + static_cast<float>(p_box.skills.size()) +
                        (p_box.detail.empty() ? 0.0F : 1.0F);
    return ImVec2(width, lines * lineHeight() + 12.0F);
}

/// Places the boxes column by column: each column as wide as its widest
/// box, the boxes of a column one under the other in their order.
static ImVec2 layout(std::vector<Box>& p_boxes)
{
    std::size_t columns = 0;
    for (Box const& box : p_boxes)
    {
        columns = std::max(columns, box.column + 1U);
    }
    std::vector<float> width(columns, 0.0F);
    for (Box& box : p_boxes)
    {
        box.size = measure(box);
        width[box.column] = std::max(width[box.column], box.size.x);
    }
    std::vector<float> x(columns, MARGIN);
    for (std::size_t c = 1; c < columns; ++c)
    {
        x[c] = x[c - 1U] + width[c - 1U] + COLUMN_GAP;
    }
    // The tree: rows of the height of the tallest box.
    float tallest = 0.0F;
    for (Box const& box : p_boxes)
    {
        tallest = std::max(tallest, box.size.y);
    }
    std::vector<float> y(columns, MARGIN);
    ImVec2 extent(0.0F, 0.0F);
    for (Box& box : p_boxes)
    {
        if (box.row >= 0.0F)
        {
            box.pos =
                ImVec2(x[box.column], MARGIN + box.row * (tallest + ROW_GAP));
        }
        else
        {
            box.pos = ImVec2(x[box.column], y[box.column]);
            y[box.column] += box.size.y + ROW_GAP;
        }
        extent.x = std::max(extent.x, box.pos.x + box.size.x + MARGIN);
        extent.y = std::max(extent.y, box.pos.y + box.size.y + MARGIN);
    }
    return extent;
}

/// Color of a status of the tree.
static ImVec4 colorOf(bt::Status p_status)
{
    switch (p_status)
    {
        case bt::Status::RUNNING:
            return ORANGE;
        case bt::Status::SUCCESS:
            return GREEN;
        case bt::Status::FAILURE:
            return RED;
        case bt::Status::INVALID:
            break;
    }
    return GREY;
}

/// Word of a status of the tree.
static char const* wordOf(bt::Status p_status)
{
    switch (p_status)
    {
        case bt::Status::RUNNING:
            return "running";
        case bt::Status::SUCCESS:
            return "success";
        case bt::Status::FAILURE:
            return "failure";
        case bt::Status::INVALID:
            break;
    }
    return "not run";
}

// --- From the plan ----------------------------------------------------------

/// One box per step of the plan, an arrow from each step to the steps that
/// wait for it.
static void fromPlan(App& p_app,
                     robotik::PlanExecutor const& p_executor,
                     std::vector<Box>& p_boxes,
                     std::vector<Arrow>& p_arrows)
{
    robotik::SkillScheduler const& skills = p_app.simulation->cell().skills();
    std::vector<robotik::GroundAction> const& plan = p_executor.plan();
    std::vector<robotik::StepStatus> const& status = p_executor.steps();
    std::vector<std::vector<std::size_t>> const& after = p_executor.after();
    std::vector<std::size_t> column(plan.size(), 0U);
    for (std::size_t i = 0; i < plan.size(); ++i)
    {
        Box box;
        char number[32];
        std::snprintf(number, sizeof number, "%zu. ", i + 1U);
        box.title = number + robotik::toString(
                                 robotik::Atom{ plan[i].name, plan[i].args });
        // A step is after the steps it waits for.
        std::vector<std::size_t> waits_for;
        if (i < after.size())
        {
            for (std::size_t const before : after[i])
            {
                if (before < plan.size())
                {
                    p_arrows.push_back({ before, i });
                    if (before < i)
                    {
                        column[i] = std::max(column[i], column[before] + 1U);
                    }
                    bool const over =
                        before < status.size() &&
                        (status[before] == robotik::StepStatus::Done ||
                         status[before] == robotik::StepStatus::Skipped);
                    if (!over)
                    {
                        waits_for.push_back(before);
                    }
                }
            }
        }
        box.column = column[i];
        robotik::StepStatus const state =
            i < status.size() ? status[i] : robotik::StepStatus::Pending;
        switch (state)
        {
            case robotik::StepStatus::Done:
                box.status = "done";
                box.color = GREEN;
                break;
            case robotik::StepStatus::Skipped:
                box.status = "skipped: its effects already held";
                box.color = GREY;
                break;
            case robotik::StepStatus::Failed:
                box.status = "failed";
                box.color = RED;
                box.tip = p_executor.failure();
                break;
            case robotik::StepStatus::Pending:
                if (p_executor.running(i))
                {
                    box.status = "running";
                    box.color = ORANGE;
                }
                else if (!waits_for.empty())
                {
                    box.status = "waits for step";
                    for (std::size_t const before : waits_for)
                    {
                        box.status += " " + std::to_string(before + 1U);
                    }
                    box.color = GREY;
                }
                else
                {
                    box.status = "next";
                    box.color = BLUE;
                }
                break;
        }
        if (plan[i].duration > 0.0)
        {
            char times[48];
            std::snprintf(times,
                          sizeof times,
                          "  (%.1f s, lasts %.1f s)",
                          plan[i].start,
                          plan[i].duration);
            box.status += times;
        }
        box.skills = p_executor.nodes(plan[i]);
        for (std::string const& name : box.skills)
        {
            box.details.push_back(detailOf(p_app, name));
        }
        // A skill that waits says why, in the tip.
        for (std::string const& name : box.skills)
        {
            robotik::SkillId const id = skills.find(name);
            if (id != robotik::NO_SKILL &&
                skills.state(id) == robotik::SkillState::Waiting)
            {
                box.tip +=
                    (box.tip.empty() ? "" : "\n") + name + ": " +
                    skillWhy(skills,
                             p_app.simulation->cell().robot().resources(),
                             id);
            }
        }
        p_boxes.push_back(std::move(box));
    }
}

// --- From the tree ----------------------------------------------------------

/// Adds the box of @p_node and of its children; returns its index. A
/// parent is in the column before its children and in the row of the first
/// one; a leaf takes the next row (@p_rows).
static std::size_t fromNode(App& p_app,
                            bt::Node const& p_node,
                            std::size_t p_column,
                            float& p_rows,
                            std::vector<Box>& p_boxes,
                            std::vector<Arrow>& p_arrows)
{
    std::size_t const index = p_boxes.size();
    Box box;
    std::string const type = p_node.typeName();
    bool const action = type == "Callback";
    box.title = action ? p_node.name : type + "  " + p_node.name;
    box.status = wordOf(p_node.status());
    box.color = colorOf(p_node.status());
    box.column = p_column;
    if (action && p_app.simulation->cell().skills().find(p_node.name) !=
                      robotik::NO_SKILL)
    {
        box.skill = p_node.name;
        box.detail = detailOf(p_app, p_node.name);
        robotik::SkillScheduler const& skills =
            p_app.simulation->cell().skills();
        robotik::SkillId const id = skills.find(p_node.name);
        if (skills.state(id) == robotik::SkillState::Waiting)
        {
            box.tip = skillWhy(
                skills, p_app.simulation->cell().robot().resources(), id);
        }
    }
    p_boxes.push_back(std::move(box));

    std::vector<bt::Node const*> children;
    if (auto const* composite = dynamic_cast<bt::Composite const*>(&p_node))
    {
        for (std::size_t i = 0; i < composite->childIndices().size(); ++i)
        {
            children.push_back(&composite->childAt(i));
        }
    }
    else if (auto const* decorator =
                 dynamic_cast<bt::Decorator const*>(&p_node);
             decorator != nullptr && decorator->hasChild())
    {
        children.push_back(&decorator->childNode());
    }
    else if (bt::Node const* nested = subtreeRoot(p_node))
    {
        children.push_back(nested);
    }
    if (children.empty())
    {
        p_boxes[index].row = p_rows;
        p_rows += 1.0F;
    }
    for (bt::Node const* child : children)
    {
        std::size_t const at =
            fromNode(p_app, *child, p_column + 1U, p_rows, p_boxes, p_arrows);
        p_arrows.push_back({ index, at });
        if (p_boxes[index].row < 0.0F)
        {
            p_boxes[index].row = p_boxes[at].row;
        }
    }
    return index;
}

// --- Drawing ----------------------------------------------------------------

/// True when @p_box runs the skill selected.
static bool chosen(App const& p_app, Box const& p_box)
{
    if (p_app.ui.selection.kind != SelectionKind::Skill)
    {
        return false;
    }
    std::string const& name = p_app.ui.selection.name;
    return p_box.skill == name ||
           std::ranges::find(p_box.skills, name) != p_box.skills.end();
}

/// Draws the arrows, then the boxes, at @p_origin; returns the box under
/// the mouse, or the count of boxes.
static std::size_t draw(App& p_app,
                        std::vector<Box> const& p_boxes,
                        std::vector<Arrow> const& p_arrows,
                        ImVec2 p_origin)
{
    robotik::SkillScheduler const& skills = p_app.simulation->cell().skills();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec2 const mouse = ImGui::GetIO().MousePos;
    std::size_t hovered = p_boxes.size();
    for (std::size_t i = 0; i < p_boxes.size(); ++i)
    {
        ImVec2 const a(p_origin.x + p_boxes[i].pos.x,
                       p_origin.y + p_boxes[i].pos.y);
        if (ImGui::IsWindowHovered() && mouse.x >= a.x && mouse.y >= a.y &&
            mouse.x < a.x + p_boxes[i].size.x &&
            mouse.y < a.y + p_boxes[i].size.y)
        {
            hovered = i;
        }
    }

    // Arrows from the right side of a box to the left side of the next.
    ImU32 const muted = ImGui::GetColorU32(ImGuiCol_TextDisabled, 0.55F);
    ImU32 const lit = ImGui::GetColorU32(ImGuiCol_CheckMark);
    for (Arrow const& arrow : p_arrows)
    {
        Box const& from = p_boxes[arrow.from];
        Box const& to = p_boxes[arrow.to];
        ImVec2 const a(p_origin.x + from.pos.x + from.size.x,
                       p_origin.y + from.pos.y + from.size.y * 0.5F);
        ImVec2 const b(p_origin.x + to.pos.x,
                       p_origin.y + to.pos.y + 0.5F * lineHeight() + 6.0F);
        float const bend = std::max(24.0F, (b.x - a.x) * 0.5F);
        bool const near = arrow.from == hovered || arrow.to == hovered;
        ImU32 const color = near ? lit : muted;
        draw->AddBezierCubic(a,
                             ImVec2(a.x + bend, a.y),
                             ImVec2(b.x - bend, b.y),
                             b,
                             color,
                             near ? 2.0F : 1.2F);
        draw->AddTriangleFilled(ImVec2(b.x, b.y),
                                ImVec2(b.x - 7.0F, b.y - 4.0F),
                                ImVec2(b.x - 7.0F, b.y + 4.0F),
                                color);
    }

    ImU32 const fill = ImGui::GetColorU32(ImGuiCol_FrameBg);
    ImU32 const text = ImGui::GetColorU32(ImGuiCol_Text);
    float const line = lineHeight();
    for (std::size_t i = 0; i < p_boxes.size(); ++i)
    {
        Box const& box = p_boxes[i];
        ImVec2 const a(p_origin.x + box.pos.x, p_origin.y + box.pos.y);
        ImVec2 const b(a.x + box.size.x, a.y + box.size.y);
        bool const picked = chosen(p_app, box);
        draw->AddRectFilled(a, b, fill, 6.0F);
        draw->AddRectFilled(a,
                            ImVec2(a.x + 4.0F, b.y),
                            ImGui::GetColorU32(box.color),
                            6.0F,
                            ImDrawFlags_RoundCornersLeft);
        draw->AddRect(a,
                      b,
                      picked
                          ? text
                          : ImGui::GetColorU32(ImVec4(
                                box.color.x, box.color.y, box.color.z, 0.55F)),
                      6.0F,
                      0,
                      picked         ? 2.5F
                      : i == hovered ? 2.0F
                                     : 1.0F);
        draw->PushClipRect(a, b, true);
        draw->AddText(ImVec2(a.x + 12.0F, a.y + 6.0F), text, box.title.c_str());
        draw->AddText(ImVec2(a.x + 12.0F, a.y + 6.0F + line),
                      ImGui::GetColorU32(box.color),
                      box.status.c_str());
        draw->AddText(ImVec2(a.x + 12.0F, a.y + 6.0F + 2.0F * line),
                      ImGui::GetColorU32(ImGuiCol_TextDisabled),
                      box.detail.c_str());
        draw->PopClipRect();

        // A skill node of the tree: the whole box selects it.
        ImGui::PushID(static_cast<int>(i));
        if (!box.skill.empty())
        {
            ImGui::SetCursorScreenPos(a);
            if (ImGui::InvisibleButton("box", box.size))
            {
                selectItem(p_app, SelectionKind::Skill, box.skill);
            }
        }
        // The skills of a step, one link each in the color of its state.
        for (std::size_t k = 0; k < box.skills.size(); ++k)
        {
            std::string const& name = box.skills[k];
            robotik::SkillId const id = skills.find(name);
            ImVec2 const at(a.x + 12.0F,
                            a.y + 6.0F + static_cast<float>(k + 2U) * line);
            ImVec4 const color =
                id == robotik::NO_SKILL ? GREY : ::colorOf(skills.state(id));
            draw->AddCircleFilled(
                ImVec2(at.x + 3.0F, at.y + 0.5F * ImGui::GetTextLineHeight()),
                3.0F,
                ImGui::GetColorU32(color));
            ImGui::SetCursorScreenPos(ImVec2(at.x + 12.0F, at.y));
            ImGui::PushID(static_cast<int>(k));
            ImGui::PushStyleColor(ImGuiCol_TextLink, color);
            if (id != robotik::NO_SKILL)
            {
                itemLink(p_app, SelectionKind::Skill, name);
            }
            else
            {
                ImGui::TextDisabled("%s", name.c_str());
            }
            ImGui::PopStyleColor();
            if (k < box.details.size() && !box.details[k].empty())
            {
                ImGui::SameLine(0.0F, 10.0F);
                ImGui::TextDisabled("%s", box.details[k].c_str());
            }
            ImGui::PopID();
        }
        ImGui::PopID();
    }
    return hovered;
}

/// Plan state, steps done and replans, above the plan.
static void planHeader(robotik::PlanExecutor const& p_executor)
{
    std::string goal = p_executor.spec().goal_pddl;
    for (robotik::Literal const& literal : p_executor.spec().goal)
    {
        goal += (goal.empty() ? "" : ", ") + robotik::toString(literal);
    }
    ImGui::TextDisabled("GOAL");
    ImGui::SameLine();
    ImGui::TextWrapped("%s", goal.c_str());
    robotik::PlanState const state = p_executor.state();
    badge(robotik::toString(state),
          state == robotik::PlanState::Succeeded ? GREEN
          : state == robotik::PlanState::Failed  ? RED
                                                 : BLUE);
    ImGui::SameLine();
    ImGui::TextDisabled("%zu / %zu steps done   replans %u",
                        p_executor.done(),
                        p_executor.plan().size(),
                        p_executor.replans());
    if (!p_executor.failure().empty())
    {
        ImGui::PushStyleColor(ImGuiCol_Text, RED);
        ImGui::TextWrapped("%s", p_executor.failure().c_str());
        ImGui::PopStyleColor();
    }
}

// --- List -------------------------------------------------------------------

/// Column cells after the name of the skill @p_id: state, priority,
/// resources, why it waits, and Request or Cancel.
static void skillCells(App& p_app, robotik::SkillId p_id)
{
    robotik::RoboticCell& cell = p_app.simulation->cell();
    robotik::SkillScheduler& skills = cell.skills();
    robotik::ResourceManager const& resources = cell.robot().resources();
    robotik::SkillDescription const& description = skills.description(p_id);
    robotik::SkillState const state = skills.state(p_id);
    ImGui::TableNextColumn();
    ImGui::TextColored(colorOf(state), "%s", robotik::toString(state));
    ImGui::TableNextColumn();
    ImGui::Text(
        "%d%s", description.priority, description.cancellable ? "" : " locked");
    ImGui::TableNextColumn();
    std::string needs;
    for (robotik::ResourceRequirement const& need : description.resources)
    {
        needs += (needs.empty() ? "" : " ") + resources.name(need.id);
    }
    ImGui::TextDisabled("%s", needs.c_str());
    ImGui::TableNextColumn();
    if (state == robotik::SkillState::Waiting)
    {
        ImGui::TextWrapped("%s", skillWhy(skills, resources, p_id).c_str());
    }
    ImGui::TableNextColumn();
    if (state == robotik::SkillState::Running ||
        state == robotik::SkillState::Waiting)
    {
        if (ImGui::SmallButton("Cancel"))
        {
            skills.cancel(p_id);
        }
        ImGui::SetItemTooltip("Stop this skill.");
    }
    else if (ImGui::SmallButton("Request"))
    {
        skills.request(p_id);
    }
    else
    {
        ImGui::SetItemTooltip("Run this skill by hand.");
    }
}

/// One skill as a leaf of the list, its name in the color of its state; a
/// click selects it.
static void skillRow(App& p_app, std::string const& p_name)
{
    robotik::SkillScheduler const& skills = p_app.simulation->cell().skills();
    robotik::SkillId const id = skills.find(p_name);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGuiTreeNodeFlags flags =
        ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen |
        ImGuiTreeNodeFlags_Bullet | ImGuiTreeNodeFlags_SpanAvailWidth;
    if (isSelected(p_app, SelectionKind::Skill, p_name))
    {
        flags |= ImGuiTreeNodeFlags_Selected;
    }
    ImGui::PushStyleColor(ImGuiCol_Text,
                          id == robotik::NO_SKILL ? GREY
                                                  : colorOf(skills.state(id)));
    ImGui::TreeNodeEx(p_name.c_str(), flags, "%s", p_name.c_str());
    ImGui::PopStyleColor();
    if (id == robotik::NO_SKILL)
    {
        return;
    }
    if (ImGui::IsItemClicked())
    {
        selectItem(p_app, SelectionKind::Skill, p_name);
    }
    skillCells(p_app, id);
}

/// Box @p_index and what is under it (the boxes after it that are deeper,
/// for a tree; its skills, for a step), as a branch that folds. Returns the
/// index of the next box at the same depth or above.
static std::size_t listBranch(App& p_app,
                              std::vector<Box> const& p_boxes,
                              std::size_t p_index,
                              bool p_tree)
{
    Box const& box = p_boxes[p_index];
    std::size_t end = p_index + 1U;
    if (p_tree)
    {
        while (end < p_boxes.size() && p_boxes[end].column > box.column)
        {
            ++end;
        }
    }
    ImGui::PushID(static_cast<int>(p_index));
    // A skill node of the tree is its skill.
    if (!box.skill.empty() && end == p_index + 1U)
    {
        skillRow(p_app, box.skill);
        ImGui::PopID();
        return end;
    }
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::PushStyleColor(ImGuiCol_Text, box.color);
    bool const open = ImGui::TreeNodeEx("##branch",
                                        ImGuiTreeNodeFlags_DefaultOpen |
                                            ImGuiTreeNodeFlags_SpanAvailWidth,
                                        "%s",
                                        box.title.c_str());
    ImGui::PopStyleColor();
    ImGui::TableNextColumn();
    ImGui::TextColored(box.color, "%s", box.status.c_str());
    if (!box.tip.empty())
    {
        ImGui::SetItemTooltip("%s", box.tip.c_str());
    }
    if (open)
    {
        for (std::string const& name : box.skills)
        {
            ImGui::PushID(name.c_str());
            skillRow(p_app, name);
            ImGui::PopID();
        }
        for (std::size_t i = p_index + 1U; i < end;)
        {
            i = listBranch(p_app, p_boxes, i, p_tree);
        }
        ImGui::TreePop();
    }
    ImGui::PopID();
    return end;
}

/// The steps of the plan (or the nodes of the tree) as a tree that folds,
/// each name in the color of where it is, the skills under their step with
/// their state, priority, resources and why they wait; then the skills that
/// are in no step, folded.
static void listView(App& p_app, std::vector<Box> const& p_boxes, bool p_tree)
{
    ImGuiTableFlags const flags =
        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
        ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY |
        ImGuiTableFlags_NoSavedSettings;
    if (!beginTable("execution_list", 6, flags))
    {
        return;
    }
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn(p_tree ? "Node / skill" : "Step / skill",
                            ImGuiTableColumnFlags_WidthStretch |
                                ImGuiTableColumnFlags_NoHide,
                            2.0F);
    // Wide enough for the longest word from the first frame on.
    ImGui::TableSetupColumn("State",
                            ImGuiTableColumnFlags_WidthFixed,
                            ImGui::CalcTextSize("waits for step 00").x);
    ImGui::TableSetupColumn("Priority", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Resources",
                            ImGuiTableColumnFlags_WidthFixed,
                            ImGui::CalcTextSize("wrist_camera gripper").x);
    ImGui::TableSetupColumn("Why", ImGuiTableColumnFlags_WidthStretch, 1.5F);
    ImGui::TableSetupColumn("##action",
                            ImGuiTableColumnFlags_WidthFixed,
                            ImGui::CalcTextSize("Request").x +
                                2.0F * ImGui::GetStyle().FramePadding.x);
    ImGui::TableHeadersRow();
    std::size_t i = 0;
    while (i < p_boxes.size())
    {
        i = listBranch(p_app, p_boxes, i, p_tree);
    }

    // Every skill is reachable here, even one no step runs (Home, Stop).
    robotik::SkillScheduler const& skills = p_app.simulation->cell().skills();
    std::vector<std::string> others;
    for (robotik::SkillId id = 0; id < skills.size(); ++id)
    {
        std::string const& name = skills.description(id).name;
        bool const listed = std::ranges::any_of(
            p_boxes,
            [&](Box const& p_box)
            {
                return p_box.skill == name ||
                       std::ranges::find(p_box.skills, name) !=
                           p_box.skills.end();
            });
        if (!listed)
        {
            others.push_back(name);
        }
    }
    if (!others.empty())
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        // Alone (no plan, no tree), they are the whole list: open.
        bool const alone = p_boxes.empty();
        std::string const title = (alone ? "Skills (" : "Other skills (") +
                                  std::to_string(others.size()) + ")";
        ImGui::PushStyleColor(ImGuiCol_Text,
                              ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        bool const open =
            ImGui::TreeNodeEx("##others",
                              ImGuiTreeNodeFlags_SpanAvailWidth |
                                  (alone ? ImGuiTreeNodeFlags_DefaultOpen : 0),
                              "%s",
                              title.c_str());
        ImGui::PopStyleColor();
        if (open)
        {
            for (std::string const& name : others)
            {
                ImGui::PushID(name.c_str());
                skillRow(p_app, name);
                ImGui::PopID();
            }
            ImGui::TreePop();
        }
    }
    endTable();
}

void graphView(App& p_app, bool p_preview)
{
    if (!p_app.simulation)
    {
        emptyState("No scenario loaded.");
        return;
    }
    robotik::RoboticCell& cell = p_app.simulation->cell();
    robotik::PlanExecutor const* executor = cell.planner();
    bt::Tree const* tree = cell.tree();
    bool preview_tree = false;
    bool& show_tree = p_preview ? preview_tree : p_app.ui.graph_tree;
    if (executor == nullptr && (tree == nullptr || !tree->hasRoot()))
    {
        // Nothing to draw, but the skills can still be run by hand.
        ImGui::TextDisabled("No behavior tree and no planner: the plugin "
                            "drives the robot itself.");
        listView(p_app, {}, false);
        return;
    }
    if (executor == nullptr)
    {
        show_tree = true;
    }
    else if (tree != nullptr && tree->hasRoot() && !p_preview)
    {
        // A plan also runs as a tree: either can be drawn.
        if (ImGui::RadioButton("Plan", !show_tree))
        {
            show_tree = false;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Tree of the plan", show_tree))
        {
            show_tree = true;
        }
        ImGui::SameLine();
    }
    else
    {
        show_tree = false;
    }
    // The same steps as a list (their skills with priority and resources)
    // or as a graph (what waits for what).
    bool& as_list = p_app.ui.graph_list;
    if (!p_preview)
    {
        if (ImGui::RadioButton("List", as_list))
        {
            as_list = true;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("Graph", !as_list))
        {
            as_list = false;
        }
        ImGui::SameLine();
        helpMarker(
            "From the intention to the movement. Each step of the plan (or "
            "node of the behavior tree) in the color of where it is, and "
            "the skills it runs: state, priority, resources, why it "
            "waits. The graph draws what waits for what, left to right; "
            "drag its background to move. Click a skill to inspect it.");
    }

    std::vector<Box> boxes;
    std::vector<Arrow> arrows;
    if (!show_tree && executor != nullptr)
    {
        if (!p_preview)
            planHeader(*executor);
        if (executor->plan().empty())
        {
            ImGui::TextDisabled("No plan yet: press Play.");
            listView(p_app, {}, false);
            return;
        }
        fromPlan(p_app, *executor, boxes, arrows);
    }
    else if (tree != nullptr && tree->hasRoot())
    {
        float rows = 0.0F;
        (void)fromNode(p_app, tree->getRoot(), 0U, rows, boxes, arrows);
    }
    if (as_list)
    {
        listView(p_app, boxes, show_tree);
        return;
    }
    ImVec2 const extent = layout(boxes);

    if (ImGui::BeginChild("canvas",
                          ImVec2(0.0F, 0.0F),
                          ImGuiChildFlags_None,
                          ImGuiWindowFlags_HorizontalScrollbar))
    {
        // A new selection: bring its box into sight.
        Selection const& chosen_item = p_app.ui.selection;
        std::string const key = chosen_item.name;
        if (key != p_app.ui.graph_seen)
        {
            p_app.ui.graph_seen = key;
            for (Box const& box : boxes)
            {
                if (chosen(p_app, box))
                {
                    ImVec2 const room = ImGui::GetContentRegionAvail();
                    ImGui::SetScrollX(std::max(
                        0.0F, box.pos.x + 0.5F * (box.size.x - room.x)));
                    ImGui::SetScrollY(std::max(
                        0.0F, box.pos.y + 0.5F * (box.size.y - room.y)));
                    break;
                }
            }
        }
        ImVec2 const origin = ImGui::GetCursorScreenPos();
        std::size_t const hovered = draw(p_app, boxes, arrows, origin);
        if (hovered < boxes.size() && !boxes[hovered].tip.empty())
        {
            ImGui::SetTooltip("%s", boxes[hovered].tip.c_str());
        }
        // The background moves the graph.
        if (ImGui::IsWindowHovered() && !ImGui::IsAnyItemActive() &&
            ImGui::IsMouseDragging(ImGuiMouseButton_Left))
        {
            ImVec2 const delta = ImGui::GetIO().MouseDelta;
            ImGui::SetScrollX(ImGui::GetScrollX() - delta.x);
            ImGui::SetScrollY(ImGui::GetScrollY() - delta.y);
        }
        ImGui::SetCursorScreenPos(origin);
        ImGui::Dummy(extent);
    }
    ImGui::EndChild();
}
