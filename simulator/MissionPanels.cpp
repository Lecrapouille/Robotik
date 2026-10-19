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

/// Color of a behavior-tree status.
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

/// @brief Root of the nested tree of a SubTree node, else null.
static bt::Node const* subtreeRoot(bt::Node const& p_node)
{
    auto const* subtree = dynamic_cast<bt::SubTreeNode const*>(&p_node);
    if (subtree == nullptr || !subtree->subtree().hasRoot())
    {
        return nullptr;
    }
    return &subtree->subtree().getRoot();
}

/// @brief Draw a tree node.
/// @param p_node The node.
static void treeNode(bt::Node const& p_node)
{
    ImGui::PushID(&p_node);
    auto const* composite = dynamic_cast<bt::Composite const*>(&p_node);
    auto const* decorator = dynamic_cast<bt::Decorator const*>(&p_node);
    bt::Node const* nested = subtreeRoot(p_node);
    bool const leaf = composite == nullptr && nested == nullptr &&
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
    bool const open =
        action ? ImGui::TreeNodeEx("##node", flags, "%s", p_node.name.c_str())
               : ImGui::TreeNodeEx("##node",
                                   flags,
                                   "%s  %s",
                                   p_node.typeName(),
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
        else if (nested != nullptr)
        {
            treeNode(*nested);
        }
        else
        {
            treeNode(decorator->childNode());
        }
        ImGui::TreePop();
    }
    ImGui::PopID();
}

/// Why the behavior tree stopped, shown above the nodes.
struct ControlFailure
{
    std::string label;
    double time = 0.0;
};

/// @brief Failed composites and decorators. Skill leaves already have a row.
static void collectControlFailures(bt::Node const& p_node,
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
    else if (bt::Node const* nested = subtreeRoot(p_node))
    {
        collectControlFailures(*nested, p_labels);
    }
}

/// @brief Remember when a control node first failed, so the mark stays put.
static std::vector<ControlFailure> const&
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

/// @brief Draw the timeline of the skill runs on a common time axis.
static void timeline(robotik::Simulation const& p_simulation)
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
    for (double const step :
         { 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0 })
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

/// @brief Names of the skills currently holding @p_id, exclusive or shared.
static std::vector<std::string>
holdersOf(robotik::SkillScheduler const& p_skills,
          robotik::ResourceManager const& p_resources,
          robotik::ResourceId p_id)
{
    std::vector<std::string> names;
    if (p_resources.owner(p_id) != robotik::NO_OWNER)
    {
        names.push_back(skillOwnerName(p_skills, p_resources.owner(p_id)));
        return names;
    }
    std::uint16_t const shared = p_resources.users(p_id);
    if (shared == 0u)
    {
        return names;
    }
    for (robotik::SkillId id = 0; id < p_skills.size(); ++id)
    {
        if (p_skills.state(id) != robotik::SkillState::Running)
        {
            continue;
        }
        for (robotik::ResourceRequirement const& need :
             p_skills.description(id).resources)
        {
            if (need.id == p_id && need.access == robotik::Access::Shared)
            {
                names.push_back(p_skills.name(id));
            }
        }
    }
    // Users that are not skills of the scheduler (a plugin, the teach pendant).
    while (names.size() < shared)
    {
        names.emplace_back("(external)");
    }
    return names;
}

/// @brief Skills that need @p_id, whether they run or not.
static std::vector<std::string>
neededBy(robotik::SkillScheduler const& p_skills, robotik::ResourceId p_id)
{
    std::vector<std::string> names;
    for (robotik::SkillId id = 0; id < p_skills.size(); ++id)
    {
        for (robotik::ResourceRequirement const& need :
             p_skills.description(id).resources)
        {
            if (need.id == p_id)
            {
                names.push_back(p_skills.name(id));
                break;
            }
        }
    }
    return names;
}

/// @brief "a, b, c", or @p_none when empty.
static std::string joined(std::vector<std::string> const& p_names,
                          char const* p_none)
{
    if (p_names.empty())
    {
        return p_none;
    }
    std::string text;
    for (std::string const& name : p_names)
    {
        text += text.empty() ? "" : ", ";
        text += name;
    }
    return text;
}

/// Short word for how a resource is held: free, shared, exclusive, fault.
static char const* accessOf(robotik::ResourceManager const& p_resources,
                            robotik::ResourceId p_id)
{
    return !p_resources.available(p_id)                   ? "in fault"
           : p_resources.owner(p_id) != robotik::NO_OWNER ? "exclusive"
           : p_resources.users(p_id) > 0u                 ? "shared"
                                                          : "free";
}

/// Color of a resource from its fault and its leases.
static ImVec4 colorOf(robotik::ResourceManager const& p_resources,
                      robotik::ResourceId p_id)
{
    return !p_resources.available(p_id)                   ? RED
           : p_resources.owner(p_id) != robotik::NO_OWNER ? ORANGE
           : p_resources.users(p_id) > 0u                 ? BLUE
                                                          : GREY;
}

/// @brief Tooltip of a resource: one fact per row.
static void resourceTooltip(robotik::SkillScheduler const& p_skills,
                            robotik::ResourceManager const& p_resources,
                            robotik::ResourceId p_id)
{
    if (!ImGui::BeginTooltip())
    {
        return;
    }
    ImGui::TextUnformatted(p_resources.name(p_id).c_str());
    ImGui::Separator();
    if (beginTable("tip", 2, ImGuiTableFlags_SizingFixedFit))
    {
        auto row = [](char const* p_key,
                      ImVec4 const& p_color,
                      std::string const& p_value)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", p_key);
            ImGui::TableNextColumn();
            ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 320.0f);
            ImGui::TextColored(p_color, "%s", p_value.c_str());
            ImGui::PopTextWrapPos();
        };
        ImVec4 const text = ImGui::GetStyleColorVec4(ImGuiCol_Text);
        row("State", colorOf(p_resources, p_id), accessOf(p_resources, p_id));
        row("Held by",
            text,
            joined(holdersOf(p_skills, p_resources, p_id), "nobody"));
        row("Needed by", text, joined(neededBy(p_skills, p_id), "no skill"));
        endTable();
    }
    ImGui::TextDisabled("Faults: Resources panel.");
    ImGui::EndTooltip();
}

/// @brief One resource as a chip: free (outline), held (orange), shared
/// (blue), in fault (red), with the number of holders. Its width is fixed and
/// the last holders stay shown a moment: the chip does not flicker when
/// skills take and release it within a few steps. The tooltip and the
/// Resources panel list the holders.
static void resourceChip(App& p_app, robotik::ResourceId p_id)
{
    constexpr double HOLD_S = 0.4;
    robotik::Simulation& simulation = *p_app.simulation;
    robotik::ResourceManager const& resources = simulation.robot().resources();
    robotik::SkillScheduler const& skills = simulation.skills();
    std::vector<ChipHold>& holds = p_app.ui.chip_hold;
    holds.resize(resources.size());
    ChipHold& hold = holds[p_id];
    double const now = ImGui::GetTime();
    std::size_t const holders = holdersOf(skills, resources, p_id).size();
    if (holders > 0u || now >= hold.until)
    {
        hold = { holders,
                 resources.owner(p_id) != robotik::NO_OWNER,
                 now + HOLD_S };
    }

    bool const available = resources.available(p_id);
    std::string const& name = resources.name(p_id);
    std::string text = name;
    if (!available)
    {
        text += "  FAULT";
    }
    else if (hold.holders > 0u)
    {
        text += "  " + std::to_string(hold.holders);
    }
    ImVec4 const color = !available           ? RED
                         : hold.holders == 0u ? GREY
                         : hold.exclusive     ? ORANGE
                                              : BLUE;
    ImVec2 const pad(8.0f, 2.0f);
    float const widest =
        std::max(ImGui::CalcTextSize((name + "  FAULT").c_str()).x,
                 ImGui::CalcTextSize((name + "  99").c_str()).x);
    ImVec2 const size(widest + 2.0f * pad.x, ImGui::GetFrameHeight());
    if (ImGui::GetContentRegionAvail().x < size.x)
    {
        ImGui::NewLine();
    }
    ImGui::PushID(static_cast<int>(p_id));
    ImVec2 const at = ImGui::GetCursorScreenPos();
    ImGui::Dummy(size);
    bool const hovered = ImGui::IsItemHovered();
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec4 fill = color;
    fill.w = (available && hold.holders == 0u) ? (hovered ? 0.12f : 0.0f)
                                               : (hovered ? 0.45f : 0.30f);
    draw->AddRectFilled(at,
                        ImVec2(at.x + size.x, at.y + size.y),
                        ImGui::GetColorU32(fill),
                        size.y * 0.5f);
    draw->AddRect(at,
                  ImVec2(at.x + size.x, at.y + size.y),
                  ImGui::GetColorU32(color),
                  size.y * 0.5f,
                  0,
                  1.2f);
    draw->AddText(ImVec2(at.x + pad.x,
                         at.y + (size.y - ImGui::GetTextLineHeight()) * 0.5f),
                  ImGui::GetColorU32(ImGuiCol_Text),
                  text.c_str());
    if (hovered)
    {
        resourceTooltip(skills, resources, p_id);
    }
    ImGui::PopID();
    ImGui::SameLine(0.0f, 6.0f);
}

/// @brief Who holds what: one row per resource.
static void resourceTable(App& p_app)
{
    robotik::Simulation& simulation = *p_app.simulation;
    robotik::ResourceManager const& resources = simulation.robot().resources();
    robotik::SkillScheduler const& skills = simulation.skills();
    if (!beginTable("resources",
                    4,
                    ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                        ImGuiTableFlags_SizingFixedFit))
    {
        return;
    }
    ImGui::TableSetupColumn("Resource");
    ImGui::TableSetupColumn("State");
    ImGui::TableSetupColumn("Held by", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Needed by", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableHeadersRow();
    for (robotik::ResourceId id = 0; id < resources.size(); ++id)
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(resources.name(id).c_str());
        ImGui::TableNextColumn();
        ImGui::TextColored(
            colorOf(resources, id), "%s", accessOf(resources, id));
        ImGui::TableNextColumn();
        ImGui::TextWrapped(
            "%s", joined(holdersOf(skills, resources, id), "-").c_str());
        ImGui::TableNextColumn();
        ImGui::PushStyleColor(ImGuiCol_Text,
                              ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
        ImGui::TextWrapped("%s", joined(neededBy(skills, id), "-").c_str());
        ImGui::PopStyleColor();
    }
    endTable();
}

/// @brief Restores the resources whose timed fault is over.
static void expireFaults(App& p_app)
{
    double const now = p_app.simulation->time().value();
    robotik::ResourceManager& resources = p_app.simulation->robot().resources();
    std::vector<ManualFault>& faults = p_app.ui.manual_faults;
    // A replay rewinds the clock: the faults of the old run are forgotten.
    std::erase_if(faults,
                  [&](ManualFault const& p_fault)
                  { return p_fault.since > now + 1e-9; });
    std::erase_if(faults,
                  [&](ManualFault const& p_fault)
                  {
                      if (p_fault.until > 0.0 && now >= p_fault.until)
                      {
                          resources.restore(p_fault.resource);
                          return true;
                      }
                      return false;
                  });
}

/// @brief Inject faults on purpose: tick one or more resources, then apply.
/// Nothing happens on a single click on a resource.
static void faultForm(App& p_app)
{
    robotik::Simulation& simulation = *p_app.simulation;
    robotik::ResourceManager& resources = simulation.robot().resources();
    std::vector<ManualFault>& manual = p_app.ui.manual_faults;
    std::vector<bool>& picked = p_app.ui.fault_pick;
    picked.resize(resources.size(), false);
    double const now = simulation.time().value();

    // One row per resource: tick, name, fault state, restore.
    if (beginTable("faults",
                   4,
                   ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                       ImGuiTableFlags_SizingFixedFit))
    {
        ImGui::TableSetupColumn("##pick", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("Resource");
        ImGui::TableSetupColumn("Fault", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("##restore", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();
        for (robotik::ResourceId id = 0; id < resources.size(); ++id)
        {
            std::string const& name = resources.name(id);
            bool const available = resources.available(id);
            auto const by_hand =
                std::find_if(manual.begin(),
                             manual.end(),
                             [&](ManualFault const& p_fault)
                             { return p_fault.resource == name; });
            // Keyed by name: an index would clash with the IDs of the header
            // cells (column index + label).
            ImGui::PushID(name.c_str());
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            bool value = picked[id];
            ImGui::BeginDisabled(!available);
            if (ImGui::Checkbox("##pick", &value))
            {
                picked[id] = value;
            }
            ImGui::EndDisabled();
            ImGui::TableNextColumn();
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(
                available ? ImGui::GetStyleColorVec4(ImGuiCol_Text) : RED,
                "%s",
                name.c_str());
            ImGui::TableNextColumn();
            if (available)
            {
                ImGui::TextDisabled("-");
            }
            else if (by_hand == manual.end())
            {
                ImGui::TextColored(RED, "by the scenario");
            }
            else if (by_hand->until > 0.0)
            {
                ImGui::TextColored(
                    RED, "by hand, %.1f s left", by_hand->until - now);
            }
            else
            {
                ImGui::TextColored(RED, "by hand");
            }
            ImGui::TableNextColumn();
            if (!available && ImGui::SmallButton("Restore"))
            {
                resources.restore(id);
                if (by_hand != manual.end())
                {
                    manual.erase(by_hand);
                }
            }
            ImGui::PopID();
        }
        endTable();
    }

    // Actions on the ticked resources.
    std::size_t const count = static_cast<std::size_t>(
        std::count(picked.begin(), picked.end(), true));
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("For");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    ImGui::InputFloat("##duration",
                      &p_app.ui.fault_duration,
                      0.5f,
                      1.0f,
                      p_app.ui.fault_duration > 0.0f ? "%.1f s" : "ever");
    p_app.ui.fault_duration = std::max(p_app.ui.fault_duration, 0.0f);
    ImGui::SetItemTooltip("Simulated time before the resources come back. 0: "
                          "until restored by hand.");
    ImGui::SameLine();
    ImGui::BeginDisabled(count == 0u);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.60f, 0.15f, 0.15f, 1.0f));
    char apply[48];
    std::snprintf(apply,
                  sizeof apply,
                  "Inject %zu fault%s",
                  count,
                  count > 1u ? "s" : "");
    if (ImGui::Button(apply))
    {
        double const duration = static_cast<double>(p_app.ui.fault_duration);
        for (robotik::ResourceId id = 0; id < resources.size(); ++id)
        {
            if (!picked[id])
            {
                continue;
            }
            resources.fail(id);
            std::erase_if(manual,
                          [&](ManualFault const& p_fault)
                          { return p_fault.resource == resources.name(id); });
            manual.push_back({ resources.name(id),
                               now,
                               duration > 0.0 ? now + duration : 0.0 });
            picked[id] = false;
        }
    }
    ImGui::PopStyleColor();
    ImGui::EndDisabled();
    ImGui::SameLine();
    bool any = false;
    for (robotik::ResourceId id = 0; id < resources.size(); ++id)
    {
        any = any || !resources.available(id);
    }
    ImGui::BeginDisabled(!any);
    if (ImGui::Button("Restore all"))
    {
        resources.restoreAll();
        manual.clear();
    }
    ImGui::EndDisabled();
}

/// @brief The fault plan of the scenario: past entries greyed out.
static void scenarioFaults(App& p_app)
{
    robotik::FaultInjector& injector = p_app.simulation->faults();
    if (injector.scheduled().empty() && injector.random().empty())
    {
        ImGui::TextDisabled("The scenario injects no fault.");
        return;
    }
    double const now = p_app.simulation->time().value();
    for (robotik::Fault const& fault : injector.scheduled())
    {
        bool const past = fault.at.value() <= now;
        ImGui::PushStyleColor(
            ImGuiCol_Text,
            past ? ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled)
                 : ImGui::GetStyleColorVec4(ImGuiCol_Text));
        ImGui::BulletText("%6.2f s  %s  %s",
                          fault.at.value(),
                          fault.disable ? "disable" : "restore",
                          fault.resource.c_str());
        ImGui::PopStyleColor();
    }
    for (robotik::RandomFault const& fault : injector.random())
    {
        ImGui::BulletText(
            "random  %s  %.3f / s", fault.resource.c_str(), fault.rate);
    }
}

void treeView(App& p_app)
{
    bt::Tree const* tree =
        p_app.simulation ? p_app.simulation->tree() : nullptr;
    if (tree == nullptr)
    {
        if (!p_app.simulation)
        {
            emptyState("No scenario loaded.");
        }
        else if (p_app.simulation->planner() != nullptr)
        {
            emptyState("The PDDL planner builds the tree of its plan\nwhen the "
                       "run starts "
                       "(see the Planner panel).");
        }
        else
        {
            emptyState("This scenario has no behavior tree:\nthe plugin "
                       "mission drives the robot.");
        }
        return;
    }
    ImGui::TextDisabled("%s",
                        p_app.simulation->scenario()
                            .controller.behavior_tree.filename()
                            .string()
                            .c_str());
    ImGui::SameLine();
    helpMarker("Each node shows its last status. Actions are skills, run by "
               "the scheduler.");
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
    robotik::ResourceManager const& resources =
        p_app.simulation->robot().resources();
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

void resourcesPanel(App& p_app)
{
    if (!ImGui::Begin("Resources"))
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
    expireFaults(p_app);
    ImGui::SeparatorText("Who holds what");
    resourceTable(p_app);
    ImGui::SeparatorText("Inject faults");
    faultForm(p_app);
    ImGui::SeparatorText("Faults of the scenario");
    scenarioFaults(p_app);
    ImGui::End();
}
