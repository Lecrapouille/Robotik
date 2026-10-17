// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The PDDL planner at work: its state, the plan and how far it went, what
// the robot believes, every plan made in the run, and the texts behind them
// (domain, generated problem, planner output, generated behavior tree).

#include "Ui.hpp"

#include "Robotik/Planning/PlanExecutor.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <imgui.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{

struct PlannerMemory
{
    robotik::Simulation const* simulation = nullptr;
    //!< Record shown in the text tabs; follows the last one when -1.
    int selected = -1;
    std::string saved;
};

ImVec4 colorOf(robotik::PlanState p_state)
{
    switch (p_state)
    {
        case robotik::PlanState::Succeeded:
            return GREEN;
        case robotik::PlanState::Failed:
            return RED;
        case robotik::PlanState::Waiting:
            return ORANGE;
        case robotik::PlanState::Planning:
        case robotik::PlanState::Executing:
            break;
    }
    return BLUE;
}

ImVec4 colorOf(robotik::Truth p_truth)
{
    switch (p_truth)
    {
        case robotik::Truth::True:
            return GREEN;
        case robotik::Truth::False:
            return RED;
        case robotik::Truth::Unknown:
            break;
    }
    return GREY;
}

//! @brief Writes the texts of every plan of the run in @p_folder.
std::string save(robotik::PlanExecutor const& p_executor,
                 std::filesystem::path const& p_folder)
{
    std::error_code error;
    std::filesystem::create_directories(p_folder, error);
    if (error)
    {
        return "Cannot create " + p_folder.string() + ": " + error.message();
    }
    std::ofstream(p_folder / p_executor.spec().domain.filename())
        << p_executor.domain();
    std::vector<robotik::PlanningRecord> const& history = p_executor.history();
    for (std::size_t i = 0u; i < history.size(); ++i)
    {
        std::string const n = std::to_string(i);
        std::ofstream(p_folder / ("problem_" + n + ".pddl"))
            << history[i].problem;
        std::ofstream(p_folder / ("plan_" + n + ".txt"))
            << "; " << history[i].cause << "\n"
            << history[i].result.report << history[i].result.diagnostics;
        if (!history[i].tree.empty())
        {
            std::ofstream(p_folder / ("plan_" + n + ".bt.yml"))
                << history[i].tree;
        }
    }
    return "Saved " + std::to_string(history.size()) + " plan(s) in " +
           p_folder.string();
}

void planTab(robotik::PlanExecutor const& p_executor)
{
    std::vector<robotik::GroundAction> const& plan = p_executor.plan();
    if (plan.empty())
    {
        emptyState(p_executor.state() == robotik::PlanState::Succeeded
                       ? "Nothing to do: the goal already holds."
                       : "No plan yet.");
        return;
    }
    ImGuiTableFlags const flags =
        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
        ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY;
    if (!beginTable("plan", 4, flags))
    {
        return;
    }
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthStretch, 1.0f);
    ImGui::TableSetupColumn("Skills", ImGuiTableColumnFlags_WidthStretch, 1.6f);
    ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableHeadersRow();
    bool const running = p_executor.state() == robotik::PlanState::Executing;
    for (std::size_t i = 0u; i < plan.size(); ++i)
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::Text("%zu", i + 1u);
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(
            robotik::Atom{ plan[i].name, plan[i].args }.toString().c_str());
        ImGui::TableNextColumn();
        std::string skills;
        for (std::string const& node : p_executor.nodes(plan[i]))
        {
            skills += (skills.empty() ? "" : ", ") + node;
        }
        ImGui::TextDisabled("%s", skills.c_str());
        ImGui::TableNextColumn();
        robotik::StepStatus const status = i < p_executor.steps().size()
                                               ? p_executor.steps()[i]
                                               : robotik::StepStatus::Pending;
        switch (status)
        {
            case robotik::StepStatus::Done:
                ImGui::TextColored(GREEN, "done");
                break;
            case robotik::StepStatus::Skipped:
                ImGui::TextColored(GREEN, "skipped");
                ImGui::SetItemTooltip("Its effects already held in the beliefs: "
                                      "nothing to do");
                break;
            case robotik::StepStatus::Failed:
                ImGui::TextColored(RED, "failed");
                ImGui::SetItemTooltip("%s", p_executor.failure().c_str());
                break;
            case robotik::StepStatus::Pending:
                if (i == p_executor.done() && running)
                {
                    ImGui::TextColored(BLUE, "running");
                }
                else
                {
                    ImGui::TextDisabled("pending");
                }
                break;
        }
    }
    endTable();
}

void beliefsTab(robotik::PlanExecutor const& p_executor,
                robotik::Simulation& p_simulation)
{
    ImGui::TextDisabled(
        "Atoms read now on the beliefs by the observers of the mission.");
    ImGui::SameLine();
    helpMarker("True atoms go into the :init of the next problem. Unknown "
               "counts as false for "
               "the planner, but an effect read as unknown is not a failure: "
               "not seen does not "
               "mean false.");
    std::vector<robotik::ObservedAtom> const atoms =
        p_executor.observe(p_simulation.context());
    ImGuiTableFlags const flags = ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_ScrollY;
    if (!beginTable("beliefs", 2, flags))
    {
        return;
    }
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Atom", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableHeadersRow();
    for (auto const& [atom, value] : atoms)
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(atom.toString().c_str());
        ImGui::TableNextColumn();
        ImGui::TextColored(colorOf(value), "%s", robotik::toString(value));
    }
    for (robotik::Literal const& fact : p_executor.spec().facts)
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(fact.atom.toString().c_str());
        ImGui::TableNextColumn();
        ImGui::TextDisabled("fact");
    }
    endTable();
}

void historyTab(robotik::PlanExecutor const& p_executor,
                PlannerMemory& p_memory)
{
    std::vector<robotik::PlanningRecord> const& history = p_executor.history();
    ImGuiTableFlags const flags =
        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
        ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY;
    if (!beginTable("history", 6, flags))
    {
        return;
    }
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Why", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Result", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Steps", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Search", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableHeadersRow();
    for (std::size_t i = 0u; i < history.size(); ++i)
    {
        robotik::PlanningRecord const& record = history[i];
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        bool const selected = p_memory.selected < 0
                                  ? i + 1u == history.size()
                                  : static_cast<int>(i) == p_memory.selected;
        char label[32];
        std::snprintf(label, sizeof label, "%zu", i);
        if (ImGui::Selectable(
                label, selected, ImGuiSelectableFlags_SpanAllColumns))
        {
            p_memory.selected =
                i + 1u == history.size() ? -1 : static_cast<int>(i);
        }
        ImGui::SetItemTooltip(
            "Show this plan in the Problem, Output and Tree tabs");
        ImGui::TableNextColumn();
        ImGui::Text("%.2f s", record.time.value());
        ImGui::TableNextColumn();
        ImGui::TextWrapped("%s", record.cause.c_str());
        ImGui::TableNextColumn();
        ImGui::TextColored(record.result.solved() ? GREEN : RED,
                           "%s",
                           robotik::toString(record.result.status));
        ImGui::TableNextColumn();
        ImGui::Text("%zu", record.result.steps.size());
        ImGui::TableNextColumn();
        ImGui::Text("%zu exp, %.1f ms",
                    record.result.expanded,
                    record.result.milliseconds);
    }
    endTable();
}

} // namespace

void plannerPanel(App& p_app)
{
    static PlannerMemory memory;
    if (!ImGui::Begin("Planner"))
    {
        ImGui::End();
        return;
    }
    robotik::PlanExecutor* executor =
        p_app.simulation ? p_app.simulation->planner() : nullptr;
    if (executor == nullptr)
    {
        emptyState("This scenario has no planner.\nA scenario with "
                   "controller.planner gets its "
                   "plan from a PDDL goal instead of a behavior tree.");
        ImGui::End();
        return;
    }
    robotik::Simulation& simulation = *p_app.simulation;
    if (memory.simulation != &simulation ||
        memory.selected >= static_cast<int>(executor->history().size()))
    {
        memory = PlannerMemory{};
        memory.simulation = &simulation;
        // A planned scenario: bring this panel in front of the Skills tab.
        ImGui::SetWindowFocus();
    }

    // --- State --------------------------------------------------------------
    badge(robotik::toString(executor->state()), colorOf(executor->state()));
    ImGui::SameLine();
    ImGui::Text("plans %zu   replans %u / %u   step %zu / %zu",
                executor->history().size(),
                executor->replans(),
                executor->spec().max_replans,
                executor->done(),
                executor->plan().size());
    if (!executor->failure().empty())
    {
        ImGui::PushStyleColor(
            ImGuiCol_Text,
            executor->state() == robotik::PlanState::Failed ? RED : ORANGE);
        bool const failed = executor->state() == robotik::PlanState::Failed;
        ImGui::TextWrapped(
            "%s%s", failed ? "" : "Last replan: ", executor->failure().c_str());
        ImGui::PopStyleColor();
    }
    std::string goal = executor->spec().goal_pddl;
    for (robotik::Literal const& literal : executor->spec().goal)
    {
        goal += (goal.empty() ? "" : ", ") + literal.toString();
    }
    ImGui::TextDisabled("Goal");
    ImGui::SameLine();
    ImGui::TextWrapped("%s", goal.c_str());

    bool const running = !simulation.finished();
    ImGui::BeginDisabled(!running);
    if (ImGui::Button("Replan"))
    {
        executor->replan("asked by the user");
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip(
        "Drop the current plan and plan again from the beliefs");
    ImGui::SameLine();
    std::filesystem::path const folder =
        std::filesystem::current_path() / "planner" / simulation.scenario().id;
    if (ImGui::Button("Save PDDL"))
    {
        memory.saved = save(*executor, folder);
    }
    ImGui::SetItemTooltip(
        "Write the domain, and each problem, plan and tree of the run, in %s",
        folder.string().c_str());
    if (!memory.saved.empty())
    {
        ImGui::SameLine();
        ImGui::TextDisabled("%s", memory.saved.c_str());
    }
    ImGui::Separator();

    // --- Tabs ---------------------------------------------------------------
    std::vector<robotik::PlanningRecord> const& history = executor->history();
    robotik::PlanningRecord const* record = nullptr;
    if (!history.empty())
    {
        record = memory.selected < 0
                     ? &history.back()
                     : &history[static_cast<std::size_t>(memory.selected)];
    }
    std::string const n = std::to_string(
        memory.selected < 0 ? history.size() - 1u
                            : static_cast<std::size_t>(memory.selected));
    if (ImGui::BeginTabBar("planner"))
    {
        if (ImGui::BeginTabItem("Plan"))
        {
            planTab(*executor);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Beliefs"))
        {
            beliefsTab(*executor, simulation);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("History"))
        {
            historyTab(*executor, memory);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Domain"))
        {
            textView(executor->spec().domain, executor->domain());
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Problem"))
        {
            if (record != nullptr)
            {
                textView(folder / ("problem_" + n + ".pddl"), record->problem);
            }
            else
            {
                // Not planned yet: what the planner would get now.
                textView(folder / "problem.pddl",
                         executor->problem(simulation.context()));
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Output"))
        {
            if (record != nullptr)
            {
                textView(folder / ("plan_" + n + ".txt"),
                         record->result.report + record->result.diagnostics);
            }
            else
            {
                emptyState("No plan yet.");
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Tree"))
        {
            if (record != nullptr && !record->tree.empty())
            {
                textView(folder / ("plan_" + n + ".bt.yml"), record->tree);
            }
            else
            {
                emptyState("No behavior tree for this plan.");
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}
