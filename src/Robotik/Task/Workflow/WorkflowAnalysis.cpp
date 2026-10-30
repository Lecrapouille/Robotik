// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Workflow/WorkflowAnalysis.hpp"

// Only this folder and Task/Grafcet include TimedPetriNetEditor.
#include "MaxPlus/Algorithms.hpp"
#include "MaxPlus/Flowshop.hpp"
#include "PetriNet/PetriNet.hpp"

#include <algorithm>

namespace robotik
{

/// Caption of the transition of an operation (see tpne::buildFlowshop).
static std::string captionOf(std::string const& p_operator, std::string const& p_product)
{
    return p_operator + "_" + p_product;
}

/// Where an operation is in the workflow, by the caption of its transition.
struct Where
{
    WorkflowProduct const* product;
    WorkflowOperation const* operation;
};

/// The flowshop of a workflow (machines: the operators, conveyors: the
/// product kinds), and where each operation is.
static tpne::Flowshop flowshopOf(WorkflowDescription const& p_workflow,
                                 std::map<std::string, Where>& p_operations)
{
    tpne::Flowshop shop;
    shop.nmachines = p_workflow.operators.size();
    shop.nconveyors = p_workflow.products.size();
    for (WorkflowOperator const& op : p_workflow.operators)
    {
        shop.machineNames.push_back(op.id);
        shop.nm.push_back(op.capacity);
    }
    for (WorkflowProduct const& product : p_workflow.products)
    {
        shop.conveyorNames.push_back(product.id);
        shop.np.push_back(product.count);
    }
    shop.PT.assign(shop.nmachines,
                   std::vector<float>(shop.nconveyors, tpne::flowshopNoOperation()));
    for (std::size_t p = 0U; p < p_workflow.products.size(); ++p)
    {
        WorkflowProduct const& product = p_workflow.products[p];
        for (WorkflowOperation const& operation : product.route)
        {
            for (std::size_t m = 0U; m < p_workflow.operators.size(); ++m)
            {
                if (p_workflow.operators[m].id == operation.on)
                {
                    shop.PT[m][p] = static_cast<float>(operation.duration.value());
                }
            }
            p_operations[captionOf(operation.on, product.id)] = { &product, &operation };
        }
    }
    return shop;
}

Result<WorkflowAnalysis> analyseWorkflow(WorkflowDescription const& p_workflow,
                                         std::size_t p_cycles)
{
    std::map<std::string, Where> operations;
    tpne::Flowshop const shop = flowshopOf(p_workflow, operations);
    std::string const error = tpne::checkFlowshop(shop);
    if (!error.empty())
    {
        return failure(p_workflow.file.string() + ": " + error);
    }

    auto net = std::make_shared<tpne::Net>(tpne::TypeOfNet::TimedPetriNet);
    tpne::buildFlowshop(*net, shop);
    net->name = p_workflow.id;

    WorkflowAnalysis analysis;
    tpne::CriticalCycleResult const cycle = tpne::findCriticalCycle(*net);
    if (!cycle.success)
    {
        return failure(p_workflow.file.string() + ": " + cycle.message.str());
    }
    double lambda = 0.0;
    for (double const duration : cycle.durations)
    {
        lambda = std::max(lambda, duration);
    }
    analysis.cycle_time = Seconds(lambda);
    analysis.throughput =
        lambda > 0.0 ? static_cast<double>(p_workflow.products.size()) / lambda : 0.0;

    // Critical operations: transitions at the start of a critical arc.
    analysis.critical_arcs = cycle.arcs;
    for (std::size_t const a : cycle.arcs)
    {
        tpne::Arc const& arc = net->arcs()[a];
        if (!arc.from.isTransition())
        {
            continue;
        }
        auto const found = operations.find(net->transitions()[arc.from.id].caption);
        if (found == operations.end())
        {
            continue;
        }
        std::string const text = found->second.product->id + ": " +
                                 found->second.operation->name + " (" +
                                 found->second.operation->on + ")";
        if (std::ranges::find(analysis.critical, text) == analysis.critical.end())
        {
            analysis.critical.push_back(text);
        }
        std::string const& on = found->second.operation->on;
        if (std::ranges::find(analysis.critical_operators, on) == analysis.critical_operators.end())
        {
            analysis.critical_operators.push_back(on);
        }
    }

    // Earliest schedule: an operation transition fires when it starts.
    std::vector<std::vector<double>> dates;
    std::string const late = tpne::earliestDates(*net, p_cycles, dates);
    if (!late.empty())
    {
        return failure(p_workflow.file.string() + ": " + late);
    }
    auto const& transitions = net->transitions();
    for (std::size_t t = 0U; t < transitions.size(); ++t)
    {
        auto const found = operations.find(transitions[t].caption);
        if (found == operations.end())
        {
            continue;
        }
        WorkflowOperation const& operation = *found->second.operation;
        for (std::size_t k = 0U; k < p_cycles; ++k)
        {
            analysis.bars.push_back({ operation.on, found->second.product->id,
                                      operation.name, k, Seconds(dates[t][k]),
                                      Seconds(dates[t][k]) + operation.duration });
        }
    }
    std::ranges::sort(analysis.bars, {}, [](WorkflowBar const& p_bar)
                      { return p_bar.start.value(); });
    analysis.graph = std::move(net);
    return analysis;
}

std::map<std::string, Seconds> measuredDurations(SkillScheduler const& p_skills)
{
    std::map<std::string, std::pair<double, std::size_t>> sums;
    for (SkillRun const& run : p_skills.trace())
    {
        if (run.state == SkillState::Succeeded)
        {
            auto& sum = sums[p_skills.name(run.skill)];
            sum.first += (run.end - run.start).value();
            ++sum.second;
        }
    }
    std::map<std::string, Seconds> means;
    for (auto const& [name, sum] : sums)
    {
        means[name] = Seconds(sum.first / static_cast<double>(sum.second));
    }
    return means;
}

std::size_t useMeasuredDurations(WorkflowDescription& p_workflow,
                                 std::map<std::string, Seconds> const& p_measured)
{
    std::size_t changed = 0U;
    for (WorkflowProduct& product : p_workflow.products)
    {
        for (WorkflowOperation& operation : product.route)
        {
            auto const found = p_measured.find(operation.skill);
            if (!operation.skill.empty() && found != p_measured.end())
            {
                operation.duration = found->second;
                ++changed;
            }
        }
    }
    return changed;
}

std::vector<std::size_t> eventGraphMarking(tpne::Net const& p_graph,
                                           std::map<std::string, std::size_t> const& p_started)
{
    auto const firings = [&p_graph, &p_started](std::size_t p_transition)
    {
        auto const found = p_started.find(p_graph.transitions()[p_transition].caption);
        return found == p_started.end() ? 0L : static_cast<long>(found->second);
    };
    std::vector<std::size_t> marking;
    marking.reserve(p_graph.places().size());
    for (tpne::Place const& place : p_graph.places())
    {
        long tokens = static_cast<long>(place.tokens);
        for (tpne::Arc const& arc : p_graph.arcsIn(place))
        {
            tokens += firings(arc.from.id);
        }
        for (tpne::Arc const& arc : p_graph.arcsOut(place))
        {
            tokens -= firings(arc.to.id);
        }
        marking.push_back(static_cast<std::size_t>(std::max(0L, tokens)));
    }
    return marking;
}

Result<void> writeFlowshop(WorkflowDescription const& p_workflow, std::ostream& p_out)
{
    std::map<std::string, Where> operations;
    tpne::Flowshop const shop = flowshopOf(p_workflow, operations);
    std::string const error = tpne::checkFlowshop(shop);
    if (!error.empty())
    {
        return failure(p_workflow.file.string() + ": " + error);
    }
    tpne::writeFlowshopCSV(p_out, shop);
    return success();
}

} // namespace robotik
