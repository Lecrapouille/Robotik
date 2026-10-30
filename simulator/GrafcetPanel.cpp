// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The GRAFCET controller at work (controller.grafcet of the scenario): the
// operator inputs as buttons (the desk of a GEMMA: Dcy, AU ...), the active
// steps, and one tab per graph drawn by the net viewer of
// TimedPetriNetEditor (active steps outlined, running actions in green). A
// GRAFCET generated from the workflow can be saved to open it in the editor.
// "Check deadlocks" looks for the situations where the steps and the
// resources of their skills block each other, before the run.

#include "Colors.hpp"
#include "Ui.hpp"

#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Task/Grafcet/GrafcetController.hpp"
#include "Robotik/Task/Grafcet/GrafcetDeadlocks.hpp"

#include "Editor/NetViewer.hpp"
#include "PetriNet/GrafcetEngine.hpp"
#include "PetriNet/PetriNet.hpp"

#include <imgui.h>

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

/// Data given to the viewer to know which actions run.
struct GraphActions
{
    tpne::GrafcetEngine const* engine;
    tpne::Net const* net;
};

/// True when the action of the step runs (drawn green).
static bool actionRuns(size_t p_place, size_t p_action, void* p_data)
{
    GraphActions const& graph = *static_cast<GraphActions const*>(p_data);
    return graph.engine->actionState(*graph.net, p_place, p_action).active;
}

/// Operator inputs: a click is a push button (1 during one cycle: Dcy,
/// Rearm), a right click latches or releases it (AU). Green while 1.
static void operatorInputs(robotik::GrafcetController& p_grafcet)
{
    std::map<std::string, int> const inputs = p_grafcet.operatorInputs();
    if (inputs.empty())
    {
        return;
    }
    ImGui::TextUnformatted("Operator");
    ImGui::SameLine();
    helpMarker("Inputs no skill nor observer gives: the buttons of the "
               "operator desk. Click: push button (1 during one cycle of the "
               "GRAFCET, for Dcy, Rearm). Right click: latch or release (for "
               "an emergency stop AU).");
    for (auto const& [name, value] : inputs)
    {
        ImGui::SameLine();
        bool const on = (value != 0);
        if (on)
        {
            ImGui::PushStyleColor(ImGuiCol_Button, GREEN);
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.0F, 0.0F, 0.0F, 1.0F));
        }
        if (ImGui::SmallButton(name.c_str()))
        {
            p_grafcet.pulse(name);
        }
        if (ImGui::IsItemClicked(ImGuiMouseButton_Right))
        {
            p_grafcet.setInput(name, on ? 0 : 1);
        }
        if (on)
        {
            ImGui::PopStyleColor(2);
        }
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s = %d\nclick: push, right click: %s", name.c_str(), value,
                              on ? "release" : "latch");
        }
    }
}

/// The file a generated GRAFCET is saved to: next to its workflow,
/// x.workflow.yaml -> x.grafcet.json.
static std::filesystem::path savedFile(std::filesystem::path const& p_workflow)
{
    std::string name = p_workflow.filename().string();
    std::string const suffix = ".workflow.yaml";
    if (name.ends_with(suffix))
    {
        name.resize(name.size() - suffix.size());
    }
    return p_workflow.parent_path() / (name + ".grafcet.json");
}

/// Saves a generated GRAFCET, and says where (or what went wrong).
static void saveButton(robotik::GrafcetController const& p_grafcet,
                       std::filesystem::path const& p_workflow, std::string& p_message)
{
    if (!p_grafcet.generated())
    {
        return;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Save .grafcet.json"))
    {
        std::filesystem::path const file = savedFile(p_workflow);
        auto const saved = p_grafcet.save(file);
        p_message = saved ? "Saved " + file.string() : saved.error();
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("This GRAFCET is generated from the workflow of the cell.\n"
                          "Save it to open it in TimedPetriNetEditor.");
    }
}

void grafcetPanel(App& p_app)
{
    // One viewer per graph, kept while the same simulation is shown (pan
    // and zoom of the user).
    static robotik::Simulation const* seen = nullptr;
    static std::vector<tpne::NetViewer> viewers;
    static std::string message;
    static std::optional<robotik::GrafcetDeadlocks> deadlocks;
    static tpne::NetViewer resources_viewer;

    if (!ImGui::Begin("GRAFCET"))
    {
        ImGui::End();
        return;
    }
    robotik::Simulation* simulation = p_app.simulation.get();
    robotik::GrafcetController* grafcet =
        (simulation != nullptr) ? simulation->cell().grafcet() : nullptr;
    if (grafcet == nullptr)
    {
        emptyState("The controller of this scenario is not a GRAFCET.");
        ImGui::End();
        return;
    }
    std::vector<tpne::Net> const& graphs = grafcet->graphs();
    if (simulation != seen)
    {
        seen = simulation;
        viewers.assign(graphs.size(), tpne::NetViewer{});
        message.clear();
        deadlocks.reset();
        ImGui::SetWindowFocus();
    }

    operatorInputs(*grafcet);
    std::string steps;
    for (std::string const& step : grafcet->activeSteps())
    {
        steps += (steps.empty() ? "" : "  ") + step;
    }
    ImGui::TextColored(ORANGE, "%s", steps.c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("(%zu firings)", grafcet->firings());
    saveButton(*grafcet, simulation->cell().description().controller.grafcet->file, message);
    if (!message.empty())
    {
        ImGui::TextDisabled("%s", message.c_str());
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Check deadlocks"))
    {
        deadlocks = robotik::findDeadlocks(*grafcet, simulation->cell().skills());
    }
    if (ImGui::IsItemHovered())
    {
        ImGui::SetTooltip("Looks, before the run, for the situations where steps\n"
                          "and the resources of their skills block each other\n"
                          "(receptivities taken as true some day).");
    }
    if (deadlocks)
    {
        std::string const summary = std::to_string(deadlocks->deadlocks.size()) + " deadlock(s) in " +
                                    std::to_string(deadlocks->states) + " markings" +
                                    (deadlocks->complete ? "" : " (not all)");
        ImGui::TextColored(deadlocks->deadlocks.empty() ? GREEN : RED, "%s", summary.c_str());
        for (std::string const& deadlock : deadlocks->deadlocks)
        {
            ImGui::BulletText("%s", deadlock.c_str());
        }
    }

    tpne::GrafcetEngine const& engine = grafcet->engine();
    if (ImGui::BeginTabBar("##graphs"))
    {
        for (size_t i = 0U; i < graphs.size(); ++i)
        {
            if (!ImGui::BeginTabItem(graphs[i].name.c_str()))
            {
                continue;
            }
            GraphActions actions{ &engine, &graphs[i] };
            tpne::NetViewer::Options options;
            options.initial_marking = &engine.initialMarking(graphs[i]);
            options.is_action_active = &actionRuns;
            options.data = &actions;
            ImGui::TextDisabled("Drag to pan, wheel to zoom, double click to fit.");
            viewers[i].draw("##net", graphs[i], options);
            ImGui::EndTabItem();
        }
        if (deadlocks && deadlocks->net && ImGui::BeginTabItem("Steps and resources"))
        {
            ImGui::TextDisabled("The net checked: the steps, and one place per resource.");
            resources_viewer.draw("##resources", *deadlocks->net, tpne::NetViewer::Options{});
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::End();
}
