// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "SkillView.hpp"
#include "Ui.hpp"

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace
{

ImVec4 colorOf(bt::Status p_status)
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

//! @brief Draw a tree node.
//! @param p_node The node.
void treeNode(bt::Node const& p_node)
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
    // BlackThorn calls its action leaves "Callback": they are skills here.
    std::string_view const type = p_node.typeName();
    bool const action = type == "Callback";
    bool const open = action ? ImGui::TreeNodeEx("##node", flags, "%s", p_node.name.c_str())
                             : ImGui::TreeNodeEx("##node", flags, "%s  %s", p_node.typeName(),
                                                 p_node.name.c_str());
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

struct ControlFailure
{
    std::string label;
    double time = 0.0;
};

//! @brief Failed composites and decorators. Skill leaves already have a row.
void collectControlFailures(bt::Node const& p_node,
                                   std::vector<std::string>& p_labels)
{
    bool const control =
        dynamic_cast<bt::Composite const*>(&p_node) != nullptr ||
        dynamic_cast<bt::Decorator const*>(&p_node) != nullptr;
    if (control && p_node.status() == bt::Status::FAILURE)
    {
        std::string label = p_node.typeName();
        if (!p_node.name.empty())
        {
            label += ' ';
            label += p_node.name;
        }
        p_labels.push_back(std::move(label));
    }
    if (auto const* composite = dynamic_cast<bt::Composite const*>(&p_node))
    {
        for (std::size_t i = 0; i < composite->childIndices().size(); ++i)
        {
            collectControlFailures(composite->childAt(i), p_labels);
        }
    }
    else if (auto const* decorator =
                 dynamic_cast<bt::Decorator const*>(&p_node);
             decorator != nullptr && decorator->hasChild())
    {
        collectControlFailures(decorator->childNode(), p_labels);
    }
}

//! @brief Remember when a control node first failed, so the mark stays put.
std::vector<ControlFailure> const&
controlFailures(robotik::Simulation const& p_simulation)
{
    static std::vector<ControlFailure> marks;
    static bt::Tree const* tree = nullptr;
    static double last_time = 0.0;
    double const now = p_simulation.time().value();
    bt::Tree const* current = p_simulation.tree();
    if (current != tree || now + 1e-6 < last_time)
    {
        marks.clear();
        tree = current;
    }
    last_time = now;
    if (current == nullptr || !current->hasRoot())
    {
        marks.clear();
        return marks;
    }

    std::vector<std::string> labels;
    collectControlFailures(current->getRoot(), labels);
    std::vector<ControlFailure> next;
    next.reserve(labels.size());
    for (std::string const& label : labels)
    {
        auto const known = std::find_if(marks.begin(),
                                        marks.end(),
                                        [&](ControlFailure const& p_mark)
                                        { return p_mark.label == label; });
        next.push_back(known == marks.end() ? ControlFailure{ label, now }
                                            : *known);
    }
    marks = std::move(next);
    return marks;
}

//! @brief Draw the timeline of the skill runs on a common time axis.
void timeline(robotik::Simulation const& p_simulation)
{
    robotik::SkillScheduler const& skills = p_simulation.skills();
    std::span<robotik::SkillRun const> const runs = skills.trace();
    std::vector<ControlFailure> const& failures = controlFailures(p_simulation);
    double const span = std::max(p_simulation.time().value(), 1.0);
    ImVec2 const at = ImGui::GetCursorScreenPos();
    float const width = ImGui::GetContentRegionAvail().x;
    float label = 120.0f;
    for (ControlFailure const& failure : failures)
    {
        label = std::max(label,
                         ImGui::CalcTextSize(failure.label.c_str()).x + 12.0f);
    }
    float const row = 16.0f;
    float const scale = std::max(width - label - 10.0f, 10.0f);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    // Time ruler: a tick every 1, 2, 5, 10... seconds, about 80 px apart.
    double tick = 1.0;
    for (double const step : { 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0 })
    {
        tick = step;
        if (static_cast<double>(scale) * step / span >= 80.0)
        {
            break;
        }
    }
    ImU32 const muted = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    ImU32 const grid = ImGui::GetColorU32(ImGuiCol_Separator, 0.35f);
    float const rows_height =
        static_cast<float>(failures.size() + skills.size()) * row + row;
    for (double t = 0.0; t <= span + 1e-9; t += tick)
    {
        float const x = at.x + label + static_cast<float>(t / span) * scale;
        char text[16];
        std::snprintf(text, sizeof text, "%g s", t);
        draw->AddText(ImVec2(x + 2.0f, at.y), muted, text);
        draw->AddLine(ImVec2(x, at.y), ImVec2(x, at.y + rows_height), grid);
    }
    float const ruler = row;
    float const head = ruler + static_cast<float>(failures.size()) * row;
    ImU32 const red = ImGui::GetColorU32(RED);
    for (std::size_t i = 0; i < failures.size(); ++i)
    {
        float const y = at.y + ruler + static_cast<float>(i) * row;
        draw->AddText(ImVec2(at.x, y), red, failures[i].label.c_str());
        float const x0 =
            at.x + label + static_cast<float>(failures[i].time / span) * scale;
        float const x1 =
            at.x + label +
            static_cast<float>(p_simulation.time().value() / span) * scale;
        draw->AddRectFilled(ImVec2(x0, y + 3.0f),
                            ImVec2(std::max(x1, x0 + 6.0f), y + row - 3.0f),
                            red,
                            2.0f);
    }
    for (std::size_t id = 0; id < skills.size(); ++id)
    {
        float const y = at.y + head + static_cast<float>(id) * row;
        draw->AddText(ImVec2(at.x, y),
                      ImGui::GetColorU32(::colorOf(skills.state(id))),
                      skills.name(id).c_str());
    }
    for (robotik::SkillRun const& run : runs)
    {
        float const y = at.y + head + static_cast<float>(run.skill) * row;
        float const x0 =
            at.x + label + static_cast<float>(run.start.value() / span) * scale;
        float const x1 =
            at.x + label + static_cast<float>(run.end.value() / span) * scale;
        draw->AddRectFilled(ImVec2(x0, y + 3.0f),
                            ImVec2(std::max(x1, x0 + 2.0f), y + row - 3.0f),
                            ImGui::GetColorU32(::colorOf(run.state)),
                            2.0f);
    }
    ImGui::Dummy(
        ImVec2(width, head + static_cast<float>(skills.size()) * row + 4.0f));
}

//! @brief Names of the skills currently holding @p_id.
std::string holdersOf(robotik::SkillScheduler const& p_skills,
                             robotik::ResourceManager const& p_resources,
                             robotik::ResourceId p_id)
{
    std::uint16_t const shared = p_resources.users(p_id);
    if (shared > 0u)
    {
        std::string names;
        for (robotik::SkillId id = 0; id < p_skills.size(); ++id)
        {
            if (p_skills.state(id) != robotik::SkillState::Running)
            {
                continue;
            }
            for (robotik::ResourceRequirement const& need :
                 p_skills.description(id).resources)
            {
                if (need.id != p_id || need.access != robotik::Access::Shared)
                {
                    continue;
                }
                if (!names.empty())
                {
                    names += ", ";
                }
                names += p_skills.name(id);
            }
        }
        if (names.empty())
        {
            names = "?";
        }
        names += " (";
        names += std::to_string(shared);
        names += ")";
        return names;
    }
    if (p_resources.owner(p_id) != robotik::NO_OWNER)
    {
        return skillOwnerName(p_skills, p_resources.owner(p_id));
    }
    return "-";
}

//! @brief One resource as a chip: free (outline), held (orange), shared
//! (blue), in fault (red). A click injects or clears a fault.
void resourceChip(App& p_app, robotik::ResourceId p_id)
{
    robotik::Simulation& simulation = *p_app.simulation;
    robotik::ResourceManager& resources = simulation.robot().resources();
    robotik::SkillScheduler const& skills = simulation.skills();
    bool const available = resources.available(p_id);
    bool const shared = resources.users(p_id) > 0u;
    bool const exclusive = resources.owner(p_id) != robotik::NO_OWNER;
    std::string const holder = holdersOf(skills, resources, p_id);
    std::string text = resources.name(p_id);
    if (!available)
    {
        text += "  FAULT";
    }
    else if (exclusive || shared)
    {
        text += "  ·  " + holder;
    }
    ImVec4 const color = !available ? RED : exclusive ? ORANGE : shared ? BLUE : GREY;
    ImVec2 const pad(8.0f, 2.0f);
    ImVec2 const size(ImGui::CalcTextSize(text.c_str()).x + 2.0f * pad.x, ImGui::GetFrameHeight());
    if (ImGui::GetContentRegionAvail().x < size.x)
    {
        ImGui::NewLine();
    }
    ImGui::PushID(static_cast<int>(p_id));
    ImVec2 const at = ImGui::GetCursorScreenPos();
    bool const clicked = ImGui::InvisibleButton("##chip", size);
    bool const hovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec4 fill = color;
    fill.w = (available && !exclusive && !shared) ? (hovered ? 0.12f : 0.0f) : (hovered ? 0.45f : 0.30f);
    draw->AddRectFilled(at, ImVec2(at.x + size.x, at.y + size.y), ImGui::GetColorU32(fill), size.y * 0.5f);
    draw->AddRect(at, ImVec2(at.x + size.x, at.y + size.y), ImGui::GetColorU32(color), size.y * 0.5f, 0, 1.2f);
    draw->AddText(ImVec2(at.x + pad.x, at.y + (size.y - ImGui::GetTextLineHeight()) * 0.5f),
                  ImGui::GetColorU32(ImGuiCol_Text), text.c_str());
    if (hovered)
    {
        ImGui::SetTooltip("%s\n%s\nHolders: %s\nClick to %s.", resources.name(p_id).c_str(),
                          !available ? "In fault" : shared ? "Shared" : exclusive ? "Exclusive" : "Free",
                          holder.c_str(), available ? "inject a fault" : "restore it");
    }
    if (clicked)
    {
        if (available)
        {
            resources.fail(p_id);
        }
        else
        {
            resources.restore(p_id);
        }
    }
    ImGui::PopID();
    ImGui::SameLine(0.0f, 6.0f);
}

} // namespace

void treeView(App& p_app)
{
    bt::Tree const* tree = p_app.simulation ? p_app.simulation->tree() : nullptr;
    if (tree == nullptr)
    {
        emptyState(p_app.simulation ? "This scenario has no behavior tree:\nthe plugin mission drives the robot."
                                    : "No scenario loaded.");
        return;
    }
    ImGui::TextDisabled("%s", p_app.simulation->scenario().controller.behavior_tree.filename().string().c_str());
    ImGui::SameLine();
    helpMarker("Each node shows its last status. Actions are skills, run by the scheduler.");
    ImGui::TextColored(GREY, "idle");
    ImGui::SameLine();
    ImGui::TextColored(ORANGE, "running");
    ImGui::SameLine();
    ImGui::TextColored(GREEN, "success");
    ImGui::SameLine();
    ImGui::TextColored(RED, "failure");
    ImGui::Separator();
    if (ImGui::BeginChild("tree"))
    {
        treeNode(tree->getRoot());
    }
    ImGui::EndChild();
}

void treePanel(App& p_app, char const* p_key)
{
    std::string const title = panelTitle("Behavior tree", p_key);
    if (ImGui::Begin(title.c_str()))
    {
        treeView(p_app);
    }
    ImGui::End();
}

void timelinePanel(App& p_app)
{
    if (!ImGui::Begin("Timeline"))
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
    robotik::ResourceManager const& resources = p_app.simulation->robot().resources();
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("Resources");
    ImGui::SameLine();
    for (robotik::ResourceId id = 0; id < resources.size(); ++id)
    {
        resourceChip(p_app, id);
    }
    ImGui::NewLine();
    ImGui::Separator();
    if (ImGui::BeginChild("runs"))
    {
        timeline(*p_app.simulation);
    }
    ImGui::EndChild();
    ImGui::End();
}
