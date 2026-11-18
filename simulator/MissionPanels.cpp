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

/// Root of the nested tree of a SubTree node, else null.
bt::Node const* subtreeRoot(bt::Node const& p_node)
{
    auto const* subtree = dynamic_cast<bt::SubTreeNode const*>(&p_node);
    if (subtree == nullptr || !subtree->subtree().hasRoot())
    {
        return nullptr;
    }
    return &subtree->subtree().getRoot();
}

/// Draw a tree node. A click on an action that is a skill selects it.
/// @param p_node The node.
static void treeNode(App& p_app, bt::Node const& p_node)
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
    bool const skill =
        action && p_app.simulation->cell().skills().find(p_node.name) !=
                      robotik::NO_SKILL;
    if (skill && isSelected(p_app, SelectionKind::Skill, p_node.name))
    {
        flags |= ImGuiTreeNodeFlags_Selected;
    }
    bool const open =
        action ? ImGui::TreeNodeEx("##node", flags, "%s", p_node.name.c_str())
               : ImGui::TreeNodeEx("##node",
                                   flags,
                                   "%s  %s",
                                   p_node.typeName(),
                                   p_node.name.c_str());
    ImGui::PopStyleColor();
    if (skill && ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
    {
        selectItem(p_app, SelectionKind::Skill, p_node.name);
    }
    if (!leaf && open)
    {
        if (composite != nullptr)
        {
            for (std::size_t i = 0; i < composite->childIndices().size(); ++i)
            {
                treeNode(p_app, composite->childAt(i));
            }
        }
        else if (nested != nullptr)
        {
            treeNode(p_app, *nested);
        }
        else
        {
            treeNode(p_app, decorator->childNode());
        }
        ImGui::TreePop();
    }
    ImGui::PopID();
}

/// Names of the skills currently holding @p_id, exclusive or shared.
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
    if (shared == 0U)
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

/// Skills that need @p_id, whether they run or not.
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

/// "a, b, c", or @p_none when empty.
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
           : p_resources.users(p_id) > 0U                 ? "shared"
                                                          : "free";
}

/// Color of a resource from its fault and its leases.
static ImVec4 colorOf(robotik::ResourceManager const& p_resources,
                      robotik::ResourceId p_id)
{
    return !p_resources.available(p_id)                   ? RED
           : p_resources.owner(p_id) != robotik::NO_OWNER ? ORANGE
           : p_resources.users(p_id) > 0U                 ? BLUE
                                                          : GREY;
}

/// Who holds what: one row per resource.
static void resourceTable(App& p_app)
{
    robotik::Simulation& simulation = *p_app.simulation;
    robotik::ResourceManager const& resources = simulation.cell().robot().resources();
    robotik::SkillScheduler const& skills = simulation.cell().skills();
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
        itemLink(p_app, SelectionKind::Resource, resources.name(id));
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

/// Restores the resources whose timed fault is over.
static void expireFaults(App& p_app)
{
    double const now = p_app.simulation->time().value();
    robotik::ResourceManager& resources = p_app.simulation->cell().robot().resources();
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

/// Inject faults on purpose: tick one or more resources, then apply.
/// Nothing happens on a single click on a resource.
static void faultForm(App& p_app)
{
    robotik::Simulation& simulation = *p_app.simulation;
    robotik::ResourceManager& resources = simulation.cell().robot().resources();
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
                std::ranges::find_if(manual,
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
    std::size_t const count =
        static_cast<std::size_t>(std::ranges::count(picked, true));
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("For");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0F);
    ImGui::InputFloat("##duration",
                      &p_app.ui.fault_duration,
                      0.5F,
                      1.0F,
                      p_app.ui.fault_duration > 0.0F ? "%.1f s" : "ever");
    p_app.ui.fault_duration = std::max(p_app.ui.fault_duration, 0.0F);
    ImGui::SetItemTooltip("Simulated time before the resources come back. 0: "
                          "until restored by hand.");
    ImGui::SameLine();
    ImGui::BeginDisabled(count == 0U);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.60F, 0.15F, 0.15F, 1.0F));
    char apply[48];
    std::snprintf(apply,
                  sizeof apply,
                  "Inject %zu fault%s",
                  count,
                  count > 1U ? "s" : "");
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

/// The fault plan of the scenario: past entries greyed out.
static void scenarioFaults(App& p_app)
{
    robotik::FaultInjector& injector = p_app.simulation->cell().faults();
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
        p_app.simulation ? p_app.simulation->cell().tree() : nullptr;
    if (tree == nullptr)
    {
        if (!p_app.simulation)
        {
            emptyState("No scenario loaded.");
        }
        else if (p_app.simulation->cell().planner() != nullptr)
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
        treeNode(p_app, tree->getRoot());
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
