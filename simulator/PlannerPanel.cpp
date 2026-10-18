// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The PDDL planner at work. Three views:
// - plannerView: the goal, how far the plan went, what the robot believes,
//   every plan of the run (Planner panel, and the View panel of Plugin);
// - plannerTexts: the texts behind a plan: domain, generated problem,
//   planner output, generated behavior tree (Files > Planner);
// - beliefsView: the symbolic beliefs (also in the Autonomy workspace).

#include "Ui.hpp"

#include "Robotik/Planning/PlanExecutor.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <imgui.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

/// Color of a plan state.
static ImVec4 colorOf(robotik::PlanState p_state)
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

/// Color of a truth value.
static ImVec4 colorOf(robotik::Truth p_truth)
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

/// @brief Writes the texts of every plan of the run in @p_folder.
static std::string save(robotik::PlanExecutor const& p_executor,
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
            std::ofstream(p_folder / ("plan_" + n + ".bt.yaml"))
                << history[i].tree;
        }
    }
    return "Saved " + std::to_string(history.size()) + " plan(s) in " +
           p_folder.string();
}

/// @brief The planner of the loaded run, or null.
static robotik::PlanExecutor* plannerOf(App& p_app)
{
    return p_app.simulation ? p_app.simulation->planner() : nullptr;
}

/// @brief Folder where Save PDDL writes the texts of the run.
static std::filesystem::path saveFolder(App const& p_app)
{
    return std::filesystem::current_path() / "planner" / p_app.simulation->scenario().id;
}

/// @brief Index of the plan whose texts are shown: the one picked in
/// History, else the last one. History must not be empty.
static std::size_t shownRecord(App& p_app, robotik::PlanExecutor const& p_executor)
{
    std::size_t const count = p_executor.history().size();
    int& picked = p_app.ui.planner_record;
    if (picked >= static_cast<int>(count))
    {
        picked = -1;
    }
    return picked < 0 ? count - 1u : static_cast<std::size_t>(picked);
}

/// Placeholder when the scenario has no PDDL planner.
static void noPlanner()
{
    emptyState("This scenario has no planner.\nA scenario with controller.planner gets its "
               "plan from a PDDL goal instead of a behavior tree.");
}

/// @brief Goal and state. The goal stays on top: it is the order given to
/// the robot (one day written by a language model from a sentence).
static void goalHeader(App& p_app, robotik::PlanExecutor& p_executor)
{
    std::string goal = p_executor.spec().goal_pddl;
    for (robotik::Literal const& literal : p_executor.spec().goal)
    {
        goal += (goal.empty() ? "" : ", ") + literal.toString();
    }
    ImGui::TextDisabled("GOAL");
    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 1.2f);
    ImGui::TextWrapped("%s", goal.c_str());
    ImGui::PopFont();

    badge(robotik::toString(p_executor.state()), colorOf(p_executor.state()));
    ImGui::SameLine();
    std::size_t const steps = p_executor.plan().size();
    float const progress =
        steps == 0u ? (p_executor.state() == robotik::PlanState::Succeeded ? 1.0f : 0.0f)
                    : static_cast<float>(p_executor.done()) / static_cast<float>(steps);
    char overlay[48];
    std::snprintf(overlay, sizeof overlay, "step %zu / %zu", p_executor.done(), steps);
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram, colorOf(p_executor.state()));
    ImGui::ProgressBar(progress, ImVec2(160.0f, 0.0f), overlay);
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::TextDisabled("replans %u / %u", p_executor.replans(), p_executor.spec().max_replans);
    ImGui::SameLine();
    ImGui::BeginDisabled(p_app.simulation->finished());
    if (ImGui::SmallButton("Replan"))
    {
        p_executor.replan("asked by the user");
    }
    ImGui::EndDisabled();
    ImGui::SetItemTooltip("Drop the current plan and plan again from the beliefs");
    if (!p_executor.failure().empty())
    {
        bool const failed = p_executor.state() == robotik::PlanState::Failed;
        ImGui::PushStyleColor(ImGuiCol_Text, failed ? RED : ORANGE);
        ImGui::TextWrapped("%s%s", failed ? "" : "Last replan: ", p_executor.failure().c_str());
        ImGui::PopStyleColor();
    }
}

/// @brief The plan as a list of steps, the current one highlighted.
static void planSteps(robotik::PlanExecutor const& p_executor)
{
    std::vector<robotik::GroundAction> const& plan = p_executor.plan();
    if (plan.empty())
    {
        ImGui::TextDisabled(p_executor.state() == robotik::PlanState::Succeeded
                                ? "Nothing to do: the goal already holds."
                                : "No plan yet.");
        return;
    }
    bool const running = p_executor.state() == robotik::PlanState::Executing;
    if (!beginTable("plan", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit))
    {
        return;
    }
    ImGui::TableSetupColumn("##state", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Action", ImGuiTableColumnFlags_WidthStretch, 1.0f);
    ImGui::TableSetupColumn("Skills", ImGuiTableColumnFlags_WidthStretch, 1.4f);
    for (std::size_t i = 0u; i < plan.size(); ++i)
    {
        robotik::StepStatus const status =
            i < p_executor.steps().size() ? p_executor.steps()[i] : robotik::StepStatus::Pending;
        bool const current = status == robotik::StepStatus::Pending && i == p_executor.done() && running;
        ImGui::TableNextRow();
        if (current)
        {
            ImVec4 highlight = BLUE;
            highlight.w = 0.25f;
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg1, ImGui::GetColorU32(highlight));
        }
        ImGui::TableNextColumn();
        switch (status)
        {
            case robotik::StepStatus::Done:
                ImGui::TextColored(GREEN, "%2zu  done", i + 1u);
                break;
            case robotik::StepStatus::Skipped:
                ImGui::TextColored(GREEN, "%2zu  skip", i + 1u);
                ImGui::SetItemTooltip("Its effects already held in the beliefs: nothing to do");
                break;
            case robotik::StepStatus::Failed:
                ImGui::TextColored(RED, "%2zu  FAIL", i + 1u);
                ImGui::SetItemTooltip("%s", p_executor.failure().c_str());
                break;
            case robotik::StepStatus::Pending:
                if (current)
                {
                    ImGui::TextColored(BLUE, "%2zu  now", i + 1u);
                }
                else
                {
                    ImGui::TextDisabled("%2zu", i + 1u);
                }
                break;
        }
        ImGui::TableNextColumn();
        std::string const action = robotik::Atom{ plan[i].name, plan[i].args }.toString();
        if (current)
        {
            ImGui::TextUnformatted(action.c_str());
        }
        else
        {
            ImGui::TextColored(status == robotik::StepStatus::Pending ? GREY
                                                                       : ImGui::GetStyleColorVec4(ImGuiCol_Text),
                               "%s", action.c_str());
        }
        ImGui::TableNextColumn();
        std::string skills;
        for (std::string const& node : p_executor.nodes(plan[i]))
        {
            skills += (skills.empty() ? "" : ", ") + node;
        }
        ImGui::TextDisabled("%s", skills.c_str());
    }
    endTable();
}

/// @brief Every plan of the run. A click picks the plan whose texts
/// Files > Planner shows.
static void historyTable(App& p_app, robotik::PlanExecutor const& p_executor)
{
    std::vector<robotik::PlanningRecord> const& history = p_executor.history();
    if (history.empty())
    {
        ImGui::TextDisabled("No plan yet.");
        return;
    }
    std::size_t const shown = shownRecord(p_app, p_executor);
    if (!beginTable("history", 5, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                      ImGuiTableFlags_Resizable))
    {
        return;
    }
    ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Why", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Result", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Search", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableHeadersRow();
    for (std::size_t i = 0u; i < history.size(); ++i)
    {
        robotik::PlanningRecord const& record = history[i];
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        char label[32];
        std::snprintf(label, sizeof label, "%zu", i);
        if (ImGui::Selectable(label, i == shown, ImGuiSelectableFlags_SpanAllColumns))
        {
            p_app.ui.planner_record = i + 1u == history.size() ? -1 : static_cast<int>(i);
        }
        ImGui::SetItemTooltip("Pick this plan for Files > Planner (problem, output, tree)");
        ImGui::TableNextColumn();
        ImGui::Text("%.2f s", record.time.value());
        ImGui::TableNextColumn();
        ImGui::TextWrapped("%s", record.cause.c_str());
        ImGui::TableNextColumn();
        ImGui::TextColored(record.result.solved() ? GREEN : RED, "%s %zu",
                           robotik::toString(record.result.status), record.result.steps.size());
        ImGui::TableNextColumn();
        ImGui::TextDisabled("%zu exp, %.1f ms", record.result.expanded, record.result.milliseconds);
    }
    endTable();
}

void beliefsView(App& p_app)
{
    robotik::PlanExecutor* executor = plannerOf(p_app);
    if (executor == nullptr)
    {
        noPlanner();
        return;
    }
    bool& only_true = p_app.ui.beliefs_only_true;
    std::vector<robotik::ObservedAtom> const atoms = executor->observe(p_app.simulation->context());
    if (!beginTable("beliefs", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV))
    {
        return;
    }
    ImGui::TableSetupColumn("Atom", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthFixed);
    // Headers drawn by hand: a click on Value is the filter, so it takes no
    // line of its own.
    ImGui::TableNextRow(ImGuiTableRowFlags_Headers);
    ImGui::TableNextColumn();
    ImGui::TableHeader("Atom");
    ImGui::SetItemTooltip("Atoms read now on the beliefs by the observers of the mission. True "
                          "atoms go into the :init of the next problem. Unknown counts as false "
                          "for the planner, but an effect read as unknown is not a failure: not "
                          "seen does not mean false. Facts come from the scenario and never "
                          "change.");
    ImGui::TableNextColumn();
    ImGui::TableHeader(only_true ? "True only" : "Value");
    if (ImGui::IsItemClicked())
    {
        only_true = !only_true;
    }
    ImGui::SetItemTooltip(only_true ? "Click to show every atom" : "Click to show the true atoms only");
    for (auto const& [atom, value] : atoms)
    {
        if (only_true && value != robotik::Truth::True)
        {
            continue;
        }
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(atom.toString().c_str());
        ImGui::TableNextColumn();
        ImGui::TextColored(colorOf(value), "%s", robotik::toString(value));
    }
    for (robotik::Literal const& fact : executor->spec().facts)
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextDisabled("%s", fact.atom.toString().c_str());
        ImGui::TableNextColumn();
        ImGui::TextDisabled("fact");
    }
    endTable();
}

void plannerView(App& p_app)
{
    robotik::PlanExecutor* executor = plannerOf(p_app);
    if (executor == nullptr)
    {
        noPlanner();
        return;
    }
    goalHeader(p_app, *executor);
    if (ImGui::CollapsingHeader("Plan", ImGuiTreeNodeFlags_DefaultOpen))
    {
        planSteps(*executor);
    }
    if (ImGui::CollapsingHeader("Beliefs", ImGuiTreeNodeFlags_DefaultOpen))
    {
        beliefsView(p_app);
    }
    if (ImGui::CollapsingHeader("History"))
    {
        historyTable(p_app, *executor);
        if (ImGui::SmallButton("Show the texts"))
        {
            p_app.ui.workspace = Workspace::Plugin;
            p_app.ui.place_plugin_panels = true;
            p_app.ui.select_tab = FileTab::Planner;
        }
        ImGui::SetItemTooltip("Plugin > Files > Planner: domain, problem, output and tree of "
                              "the picked plan");
    }
}

void plannerTexts(App& p_app)
{
    robotik::PlanExecutor* executor = plannerOf(p_app);
    if (executor == nullptr)
    {
        noPlanner();
        return;
    }
    robotik::Simulation& simulation = *p_app.simulation;
    std::vector<robotik::PlanningRecord> const& history = executor->history();
    std::filesystem::path const folder = saveFolder(p_app);

    // Which plan: the last one, or one picked in History.
    robotik::PlanningRecord const* record = nullptr;
    std::string n;
    if (!history.empty())
    {
        std::size_t const index = shownRecord(p_app, *executor);
        record = &history[index];
        n = std::to_string(index);
        char preview[160];
        std::snprintf(preview, sizeof preview, "plan %zu  (%.2f s, %s)%s", index, record->time.value(),
                      record->cause.c_str(), p_app.ui.planner_record < 0 ? "  last" : "");
        ImGui::SetNextItemWidth(std::max(ImGui::GetContentRegionAvail().x - 110.0f, 120.0f));
        if (ImGui::BeginCombo("##record", preview))
        {
            for (std::size_t i = 0u; i < history.size(); ++i)
            {
                char item[160];
                std::snprintf(item, sizeof item, "plan %zu  (%.2f s, %s)", i, history[i].time.value(),
                              history[i].cause.c_str());
                if (ImGui::Selectable(item, i == index))
                {
                    p_app.ui.planner_record = i + 1u == history.size() ? -1 : static_cast<int>(i);
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
    }
    else
    {
        ImGui::TextDisabled("Not planned yet: the problem is what the planner would get now.");
        ImGui::SameLine();
    }
    if (ImGui::Button("Save PDDL"))
    {
        p_app.ui.planner_saved = save(*executor, folder);
    }
    ImGui::SetItemTooltip("Write the domain, and each problem, plan and tree of the run, in %s",
                          folder.string().c_str());
    if (!p_app.ui.planner_saved.empty())
    {
        ImGui::TextDisabled("%s", p_app.ui.planner_saved.c_str());
    }

    if (!ImGui::BeginTabBar("planner texts"))
    {
        return;
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
            textView(folder / "problem.pddl", executor->problem(simulation.context()));
        }
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Output"))
    {
        if (record != nullptr)
        {
            textView(folder / ("plan_" + n + ".txt"), record->result.report + record->result.diagnostics);
        }
        else
        {
            emptyState("No plan yet.");
        }
        ImGui::EndTabItem();
    }
    if (ImGui::BeginTabItem("Generated tree"))
    {
        if (record != nullptr && !record->tree.empty())
        {
            textView(folder / ("plan_" + n + ".bt.yaml"), record->tree);
        }
        else
        {
            emptyState("No behavior tree for this plan.");
        }
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
}

void plannerPanel(App& p_app)
{
    static robotik::Simulation const* seen = nullptr;
    if (!ImGui::Begin("Planner"))
    {
        ImGui::End();
        return;
    }
    robotik::Simulation const* current = p_app.simulation.get();
    if (current != seen)
    {
        seen = current;
        p_app.ui.planner_record = -1;
        p_app.ui.planner_saved.clear();
        // A planned scenario: bring this panel in front of the Skills tab.
        if (plannerOf(p_app) != nullptr)
        {
            ImGui::SetWindowFocus();
        }
    }
    plannerView(p_app);
    ImGui::End();
}
