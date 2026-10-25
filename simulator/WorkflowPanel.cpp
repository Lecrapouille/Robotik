// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The workflow of the cell (x.workflow.yaml files of the loaded package): its
// cycle time, its throughput, the operations of the critical circuit (the
// bottleneck), the earliest schedule as a Gantt chart, and the timed event
// graph with the critical circuit in red. The durations can be the declared
// ones or the ones measured in the current run.

#include "Colors.hpp"
#include "Ui.hpp"

#include "Robotik/Package/Plugin/PluginSession.hpp"
#include "Robotik/Package/Scenario/FileRole.hpp"
#include "Robotik/Package/Scenario/Workflow.hpp"
#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Task/Workflow/WorkflowAnalysis.hpp"

#include "Editor/NetViewer.hpp"
#include "PetriNet/PetriNet.hpp"

#include <imgui.h>
#include <implot.h>

#include <algorithm>
#include <filesystem>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

/// What the panel shows, computed when the file or the durations change.
struct WorkflowState
{
    std::filesystem::path package;
    std::vector<std::filesystem::path> files;
    std::size_t selected = 0U;
    /// Scenario the selection was made for.
    std::string scenario;
    bool measured = false;
    int cycles = 3;
    bool dirty = true;
    std::string error;
    robotik::WorkflowDescription declared;
    robotik::WorkflowDescription used;
    robotik::WorkflowAnalysis analysis;
    std::vector<tpne::Arc const*> critical_arcs;
    tpne::NetViewer viewer;
};

/// Workflow files of the package, sorted.
static std::vector<std::filesystem::path> workflowFiles(std::filesystem::path const& p_root)
{
    std::vector<std::filesystem::path> files;
    std::error_code error;
    for (auto const& entry : std::filesystem::recursive_directory_iterator(p_root, error))
    {
        if (entry.is_regular_file() &&
            robotik::fileRole(entry.path()) == robotik::FileRole::Workflow)
        {
            files.push_back(entry.path());
        }
    }
    std::ranges::sort(files);
    return files;
}

/// Reads the selected file and analyses it.
static void analyse(WorkflowState& p_state, robotik::Simulation const* p_simulation)
{
    p_state.dirty = false;
    p_state.error.clear();
    p_state.critical_arcs.clear();
    p_state.analysis = {};
    auto workflow = robotik::loadWorkflow(p_state.files[p_state.selected]);
    if (!workflow)
    {
        p_state.error = workflow.error();
        return;
    }
    p_state.declared = workflow.take();
    p_state.used = p_state.declared;
    if (p_state.measured && p_simulation != nullptr)
    {
        (void)robotik::useMeasuredDurations(
            p_state.used, robotik::measuredDurations(p_simulation->cell().skills()));
    }
    auto analysis = robotik::analyseWorkflow(p_state.used, static_cast<std::size_t>(p_state.cycles));
    if (!analysis)
    {
        p_state.error = analysis.error();
        return;
    }
    p_state.analysis = analysis.take();
    auto const& arcs = p_state.analysis.graph->arcs();
    for (std::size_t index : p_state.analysis.critical_arcs)
    {
        p_state.critical_arcs.push_back(&arcs[index]);
    }
    p_state.viewer.fit();
}

/// Durations declared and used, one line per operation.
static void durationsTable(WorkflowState const& p_state)
{
    if (!beginTable("##durations", 4))
    {
        return;
    }
    ImGui::TableSetupColumn("Product");
    ImGui::TableSetupColumn("Operation");
    ImGui::TableSetupColumn("Declared");
    ImGui::TableSetupColumn("Used");
    ImGui::TableHeadersRow();
    for (std::size_t p = 0U; p < p_state.declared.products.size(); ++p)
    {
        auto const& declared = p_state.declared.products[p];
        auto const& used = p_state.used.products[p];
        for (std::size_t o = 0U; o < declared.route.size(); ++o)
        {
            double const before = declared.route[o].duration.value();
            double const after = used.route[o].duration.value();
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(declared.id.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%s (%s)", declared.route[o].name.c_str(), declared.route[o].on.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%.2f s", before);
            ImGui::TableNextColumn();
            if (after != before)
            {
                ImGui::TextColored(BLUE, "%.2f s (measured)", after);
            }
            else
            {
                ImGui::Text("%.2f s", after);
            }
        }
    }
    endTable();
}

/// Earliest schedule: one row per operator, one color per product.
static void gantt(WorkflowState const& p_state)
{
    std::vector<std::string> rows;
    for (auto const& op : p_state.declared.operators)
    {
        rows.push_back(op.id);
    }
    std::map<std::string, int> colors;
    for (auto const& product : p_state.declared.products)
    {
        colors.emplace(product.id, static_cast<int>(colors.size()));
    }
    double end = 1.0;
    for (auto const& bar : p_state.analysis.bars)
    {
        end = std::max(end, bar.end.value());
    }
    if (!ImPlot::BeginPlot("##gantt", ImVec2(-1, 60.0F + 30.0F * static_cast<float>(rows.size())),
                           ImPlotFlags_NoLegend))
    {
        return;
    }
    std::vector<double> ticks(rows.size());
    std::vector<char const*> labels(rows.size());
    for (std::size_t i = 0U; i < rows.size(); ++i)
    {
        ticks[i] = static_cast<double>(i);
        labels[i] = rows[i].c_str();
    }
    ImPlot::SetupAxes("time (s)", nullptr, 0, ImPlotAxisFlags_Invert);
    ImPlot::SetupAxisTicks(ImAxis_Y1, ticks.data(), static_cast<int>(rows.size()), labels.data());
    ImPlot::SetupAxesLimits(0.0, end, -0.6, static_cast<double>(rows.size()) - 0.4,
                            ImPlotCond_Always);
    ImDrawList* draw = ImPlot::GetPlotDrawList();
    ImPlot::PushPlotClipRect();
    for (auto const& bar : p_state.analysis.bars)
    {
        auto const row = std::ranges::find(rows, bar.on) - rows.begin();
        double const y = static_cast<double>(row);
        ImVec2 const a = ImPlot::PlotToPixels(bar.start.value(), y - 0.35);
        ImVec2 const b = ImPlot::PlotToPixels(bar.end.value(), y + 0.35);
        draw->AddRectFilled(a, b, ImGui::GetColorU32(ImPlot::GetColormapColor(colors[bar.product])),
                            3.0F);
        std::string const label = bar.product + " #" + std::to_string(bar.cycle + 1U);
        ImVec2 const size = ImGui::CalcTextSize(label.c_str());
        if (b.x - a.x > size.x + 4.0F)
        {
            draw->AddText(ImVec2(a.x + 3.0F, (a.y + b.y - size.y) / 2.0F),
                          IM_COL32(0, 0, 0, 255), label.c_str());
        }
        if (ImGui::IsMouseHoveringRect(ImVec2(std::min(a.x, b.x), std::min(a.y, b.y)),
                                       ImVec2(std::max(a.x, b.x), std::max(a.y, b.y))))
        {
            ImGui::SetTooltip("%s: %s on %s\n%.2f s .. %.2f s", bar.product.c_str(),
                              bar.operation.c_str(), bar.on.c_str(), bar.start.value(),
                              bar.end.value());
        }
    }
    ImPlot::PopPlotClipRect();
    ImPlot::EndPlot();
}

/// GitHub's viewer highlights arcs by index in Net::arcs(). The local
/// robotik-integration viewer highlights Arc pointers. One of the two
/// assignments is compiled.
template <typename Options>
static void highlightCritical(Options& p_options, WorkflowState const& p_state)
{
    using Highlighted = std::remove_pointer_t<decltype(p_options.highlighted_arcs)>;
    if constexpr (std::is_same_v<typename Highlighted::value_type, std::size_t>)
        p_options.highlighted_arcs = &p_state.analysis.critical_arcs;
    else
        p_options.highlighted_arcs = &p_state.critical_arcs;
}

void workflowPanel(App& p_app)
{
    static WorkflowState state;

    if (!ImGui::Begin("Workflow"))
    {
        ImGui::End();
        return;
    }
    robotik::PluginPackage const* package =
        p_app.plugins.catalog().matchScenario(p_app.scenario_path).package;
    std::filesystem::path const root = (package != nullptr) ? package->root : std::filesystem::path();
    if (root != state.package)
    {
        state.package = root;
        state.files = root.empty() ? std::vector<std::filesystem::path>{} : workflowFiles(root);
        state.selected = 0U;
        state.dirty = true;
        state.scenario.clear();
    }
    // The workflow named after the scenario, when there is one
    // (factory.scenario.yaml -> factory.workflow.yaml).
    std::string const scenario = p_app.simulation ? p_app.simulation->scenario().id : std::string();
    if (scenario != state.scenario)
    {
        state.scenario = scenario;
        for (std::size_t i = 0U; i < state.files.size(); ++i)
        {
            if (robotik::roleStem(state.files[i]) == scenario + "_workflow")
            {
                state.selected = i;
                state.dirty = true;
            }
        }
    }
    if (state.files.empty())
    {
        emptyState("No workflow (x.workflow.yaml) in this package.");
        ImGui::End();
        return;
    }

    // File, durations, cycles.
    std::string const name = state.files[state.selected].filename().string();
    ImGui::SetNextItemWidth(-80.0F);
    if (ImGui::BeginCombo("##file", name.c_str()))
    {
        for (std::size_t i = 0U; i < state.files.size(); ++i)
        {
            if (ImGui::Selectable(state.files[i].filename().string().c_str(), i == state.selected))
            {
                state.selected = i;
                state.dirty = true;
            }
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    state.dirty |= ImGui::Button("Analyse");
    state.dirty |= ImGui::Checkbox("Measured durations", &state.measured);
    ImGui::SameLine();
    helpMarker("Replace the declared duration of an operation by the mean "
               "duration of its skill in the current run (Analyse again "
               "after the run).");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(100.0F);
    state.dirty |= ImGui::SliderInt("Cycles", &state.cycles, 1, 10);
    if (state.dirty)
    {
        analyse(state, p_app.simulation.get());
    }
    if (!state.error.empty())
    {
        ImGui::TextColored(RED, "%s", state.error.c_str());
        ImGui::End();
        return;
    }

    // Results.
    robotik::WorkflowAnalysis const& analysis = state.analysis;
    ImGui::Text("Cycle time");
    ImGui::SameLine();
    badge((std::to_string(analysis.cycle_time.value()).substr(0, 5) + " s").c_str(), ORANGE);
    ImGui::SameLine();
    ImGui::Text("Throughput");
    ImGui::SameLine();
    badge((std::to_string(analysis.throughput * 60.0).substr(0, 5) + " /min").c_str(), GREEN);
    ImGui::TextUnformatted("Critical circuit (bottleneck):");
    for (std::string const& operation : analysis.critical)
    {
        ImGui::BulletText("%s", operation.c_str());
    }

    if (ImGui::BeginTabBar("##workflow"))
    {
        if (ImGui::BeginTabItem("Gantt"))
        {
            gantt(state);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Durations"))
        {
            durationsTable(state);
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Event graph"))
        {
            tpne::NetViewer::Options options;
            highlightCritical(options, state);
            state.viewer.draw("##graph", *analysis.graph, options);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}
