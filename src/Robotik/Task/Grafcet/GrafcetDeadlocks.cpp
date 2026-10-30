// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Grafcet/GrafcetDeadlocks.hpp"

#include "Robotik/Task/Grafcet/GrafcetController.hpp"
#include "Robotik/Task/Scheduler.hpp"

#include "Analysis/Analysis.hpp"
#include "PetriNet/GrafcetEngine.hpp"
#include "PetriNet/PetriNet.hpp"

#include <algorithm>
#include <optional>
#include <set>
#include <utility>

namespace robotik
{

/// Horizontal space between the graphs in the net drawn.
static constexpr float GRAPH_SPACE = 1500.0F;

/// The graphs of a GEMMA that loop for ever: left out.
static bool loops(tpne::Net const& p_graph)
{
    return p_graph.name == "Safety" || p_graph.name == "Control";
}

// ****************************************************************************
/// Writes down the steps a receptivity reads (the steps of the success).
// ****************************************************************************
class StepsRead final: public tpne::Receptivity::Resolver
{
public:

    explicit StepsRead(std::vector<tpne::Net> const& p_graphs) : m_graphs(p_graphs) {}

    int const* input(std::string const& /*p_name*/) override
    {
        return &m_zero;
    }

    tpne::Place const* step(std::string const& p_graph, size_t p_id) override
    {
        for (std::size_t g = 0U; g < m_graphs.size(); ++g)
        {
            if (p_graph.empty() || m_graphs[g].name == p_graph)
            {
                // Step ids are their indexes.
                if (p_id < m_graphs[g].places().size())
                {
                    steps.emplace_back(g, p_id);
                    return &m_graphs[g].places()[p_id];
                }
            }
        }
        return nullptr;
    }

    /// Graph index and step id of each step read.
    std::vector<std::pair<std::size_t, std::size_t>> steps;

private:

    std::vector<tpne::Net> const& m_graphs;
    int m_zero = 0;
};

/// "Production:X12 (M1.Grasp(p1a))": a step in words.
static std::string stepName(tpne::Net const& p_graph, tpne::Place const& p_step)
{
    std::string name = p_graph.name + ":X" + std::to_string(p_step.id);
    if (!p_step.caption.empty())
    {
        name += " (" + p_step.caption + ")";
    }
    return name;
}

GrafcetDeadlocks findDeadlocks(std::vector<tpne::Net> const& p_graphs,
                               std::map<std::string, std::vector<std::string>> const& p_resources,
                               std::string const& p_success, std::size_t p_max_states)
{
    auto net = std::make_shared<tpne::Net>(tpne::TypeOfNet::PetriNet);
    net->name = "Steps and resources";

    // One place per exclusive resource of an action, free at the start.
    std::map<std::string, std::size_t> resource;
    float x = 0.0F;
    for (auto const& [action, names] : p_resources)
    {
        (void)action;
        for (std::string const& name : names)
        {
            if (!resource.contains(name))
            {
                tpne::Place& place = net->addPlace(1U, name);
                net->position(place) = tpne::Point{ x, -120.0F };
                x += 140.0F;
                resource[name] = place.id;
            }
        }
    }

    // The steps: where a token waits (step), and where it holds the
    // resources of the actions of the step (holds), when it has some.
    struct Step
    {
        std::size_t graph = 0U;
        std::size_t id = 0U;
        std::size_t place = 0U;
        std::optional<std::size_t> holds;
        std::vector<std::size_t> resources;
    };
    std::vector<Step> steps;
    std::map<std::pair<std::size_t, std::size_t>, std::size_t> index;
    for (std::size_t g = 0U; g < p_graphs.size(); ++g)
    {
        tpne::Net const& graph = p_graphs[g];
        if (loops(graph))
        {
            continue;
        }
        float const shift = GRAPH_SPACE * static_cast<float>(g);
        for (tpne::Place const& place : graph.places())
        {
            Step step{ g, place.id, 0U, std::nullopt, {} };
            std::set<std::size_t> held;
            for (tpne::Action const& action : place.actions)
            {
                auto const found = p_resources.find(action.name);
                if (found == p_resources.end() || action.qualifier != tpne::Action::Qualifier::N)
                {
                    continue;
                }
                for (std::string const& name : found->second)
                {
                    held.insert(resource.at(name));
                }
            }
            tpne::Point const at = graph.position(place);
            tpne::Place& added = net->addPlace(place.tokens, stepName(graph, place));
            net->position(added) = tpne::Point{ at.x + shift, at.y };
            step.place = added.id;
            if (!held.empty())
            {
                step.resources.assign(held.begin(), held.end());
                std::string names;
                for (std::size_t const r : step.resources)
                {
                    names += (names.empty() ? "" : ", ") + net->places()[r].caption;
                }
                tpne::Place& holds = net->addPlace(0U, graph.name + ":X" + std::to_string(place.id) +
                                                           " holds " + names);
                net->position(holds) = tpne::Point{ at.x + shift + 160.0F, at.y };
                step.holds = holds.id;
                tpne::Transition& takes = net->addTransition(graph.name + ":X" +
                                                             std::to_string(place.id) + " takes " + names);
                net->position(takes) = tpne::Point{ at.x + shift + 80.0F, at.y };
                (void)net->addArc(net->places()[step.place], takes);
                for (std::size_t const r : step.resources)
                {
                    (void)net->addArc(net->places()[r], net->transitions()[takes.id]);
                }
                (void)net->addArc(net->transitions()[takes.id], net->places()[*step.holds]);
            }
            index[{ g, place.id }] = steps.size();
            steps.push_back(std::move(step));
        }
    }

    // The transitions: a step with resources leaves from where it holds
    // them, and gives them back.
    for (std::size_t g = 0U; g < p_graphs.size(); ++g)
    {
        tpne::Net const& graph = p_graphs[g];
        if (loops(graph))
        {
            continue;
        }
        float const shift = GRAPH_SPACE * static_cast<float>(g);
        for (tpne::Transition const& transition : graph.transitions())
        {
            tpne::Point const at = graph.position(transition);
            tpne::Transition& added = net->addTransition(graph.name + ":T" + std::to_string(transition.id) +
                                                         " " + transition.caption);
            net->position(added) = tpne::Point{ at.x + shift, at.y };
            std::size_t const t = added.id;
            for (tpne::Arc const& arc : graph.arcsIn(transition))
            {
                Step const& step = steps[index.at({ g, arc.from.id })];
                (void)net->addArc(net->places()[step.holds.value_or(step.place)], net->transitions()[t]);
                for (std::size_t const r : step.resources)
                {
                    (void)net->addArc(net->transitions()[t], net->places()[r]);
                }
            }
            for (tpne::Arc const& arc : graph.arcsOut(transition))
            {
                (void)net->addArc(net->transitions()[t], net->places()[steps[index.at({ g, arc.to.id })].place]);
            }
        }
    }

    // The normal end: the steps of the success, else the last steps.
    StepsRead read(p_graphs);
    tpne::Receptivity success;
    if (!p_success.empty())
    {
        (void)success.compile(p_success, read);
    }
    std::vector<std::size_t> final_places;
    for (auto const& [g, id] : read.steps)
    {
        auto const found = index.find({ g, id });
        if (found != index.end())
        {
            final_places.push_back(found->second);
        }
    }

    GrafcetDeadlocks result;
    tpne::ReachabilityGraph const graph = tpne::reachabilityGraph(*net, p_max_states);
    result.states = graph.states.size();
    result.complete = graph.complete;
    for (std::size_t const state : graph.deadlocks)
    {
        tpne::Marking const& marking = graph.states[state];
        auto const active = [&marking, &steps](std::size_t p_step)
        {
            Step const& step = steps[p_step];
            return marking[step.place] > 0U || (step.holds && marking[*step.holds] > 0U);
        };
        bool normal = false;
        if (!final_places.empty())
        {
            normal = std::ranges::all_of(final_places, active);
        }
        else
        {
            normal = true;
            for (std::size_t s = 0U; s < steps.size(); ++s)
            {
                tpne::Net const& owner = p_graphs[steps[s].graph];
                if (active(s) && !owner.arcsOut(owner.places()[steps[s].id]).empty())
                {
                    normal = false;
                }
            }
        }
        if (normal)
        {
            continue;
        }
        std::string text;
        for (std::size_t s = 0U; s < steps.size(); ++s)
        {
            Step const& step = steps[s];
            tpne::Net const& owner = p_graphs[step.graph];
            if (!active(s))
            {
                continue;
            }
            text += text.empty() ? "" : "; ";
            text += stepName(owner, owner.places()[step.id]);
            if (step.holds && marking[*step.holds] > 0U)
            {
                text += " holds";
            }
            else if (!step.resources.empty())
            {
                text += " waits for";
            }
            if (!step.resources.empty())
            {
                for (std::size_t const r : step.resources)
                {
                    text += " " + net->places()[r].caption;
                }
            }
        }
        result.deadlocks.push_back(text);
    }
    result.net = std::move(net);
    return result;
}

GrafcetDeadlocks findDeadlocks(GrafcetController const& p_grafcet, SkillScheduler const& p_skills,
                               std::size_t p_max_states)
{
    // The exclusive resources of each skill, by name.
    std::map<std::string, std::vector<std::string>> resources;
    ResourceManager const& manager = p_skills.resources();
    for (std::size_t i = 0U; i < p_skills.size(); ++i)
    {
        auto const id = static_cast<SkillId>(i);
        std::vector<std::string> names;
        for (ResourceRequirement const& need : p_skills.description(id).resources)
        {
            if (need.access == Access::Exclusive)
            {
                names.push_back(manager.name(need.id));
            }
        }
        if (!names.empty())
        {
            resources[p_skills.name(id)] = std::move(names);
        }
    }

    // The graphs in their initial situation.
    std::vector<tpne::Net> graphs = p_grafcet.graphs();
    for (std::size_t i = 0U; i < graphs.size(); ++i)
    {
        (void)graphs[i].tokens(p_grafcet.engine().initialMarking(p_grafcet.graphs()[i]));
    }
    return findDeadlocks(graphs, resources, p_grafcet.endCondition(), p_max_states);
}

} // namespace robotik
