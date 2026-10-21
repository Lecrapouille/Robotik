// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The GRAFCET controller at work (controller.grafcet of the scenario): the
// operator inputs as buttons (the desk of a GEMMA: Dcy, AU ...), the active
// steps, and one tab per graph drawn by the net viewer of
// TimedPetriNetEditor (active steps outlined, running actions in green).

#include "Colors.hpp"
#include "Ui.hpp"

#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Task/Grafcet/GrafcetController.hpp"

#include "Editor/NetViewer.hpp"
#include "PetriNet/GrafcetEngine.hpp"
#include "PetriNet/PetriNet.hpp"

#include <imgui.h>

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

void grafcetPanel(App& p_app)
{
    // One viewer per graph, kept while the same simulation is shown (pan
    // and zoom of the user).
    static robotik::Simulation const* seen = nullptr;
    static std::vector<tpne::NetViewer> viewers;

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
        ImGui::EndTabBar();
    }
    ImGui::End();
}
