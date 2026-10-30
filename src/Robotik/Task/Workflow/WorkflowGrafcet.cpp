// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Workflow/WorkflowGrafcet.hpp"

#include "PetriNet/Gemma.hpp"
#include "PetriNet/PetriNet.hpp"

#include <algorithm>
#include <cstddef>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace robotik
{

/// Horizontal space between the columns of two operators.
static constexpr float COLUMN = 420.0F;
/// Vertical space between two steps of a column.
static constexpr float ROW = 70.0F;
/// First row of the sequences, under the shared steps.
static constexpr float TOP = 200.0F;

/// One part to make: the product kind and the world entity.
struct Job
{
    std::size_t product = 0U;
    std::string part;
};

/// The net being built, and where the next step of each column goes.
struct Builder
{
    tpne::Net& net;
    /// Inputs of the next transition (steps).
    std::vector<std::size_t> inputs;
    /// Receptivity of the next transition.
    std::string code;
    /// More outputs of the next transition (steps given back).
    std::vector<std::size_t> extra;
    /// Column and row of the next step.
    float x = 0.0F;
    float y = TOP;
};

/// Adds a step at (x, y) and returns its number.
static std::size_t addStep(tpne::Net& p_net, std::string const& p_caption, std::size_t p_tokens,
                           float p_x, float p_y)
{
    tpne::Place& place = p_net.addPlace(p_tokens, p_caption);
    p_net.position(place) = tpne::Point{ p_x, p_y };
    return place.id;
}

/// Links two nodes of the net; a refused arc is a bug of this generator.
static void link(tpne::Net& p_net, tpne::Node& p_from, tpne::Node& p_to)
{
    std::string const error = p_net.addArc(p_from, p_to);
    if (!error.empty())
    {
        throw std::logic_error("workflowGrafcet: " + error);
    }
}

/// Fires the pending transition (inputs, code) into the given steps; the
/// steps given back (extra) are outputs too. Returns the transition number.
static std::size_t fire(Builder& p_builder, std::vector<std::size_t> const& p_outputs)
{
    tpne::Net& net = p_builder.net;
    tpne::Transition& created = net.addTransition(p_builder.code);
    net.position(created) = tpne::Point{ p_builder.x, p_builder.y - ROW / 2.0F };
    std::size_t const transition = created.id;
    for (std::size_t const step : p_builder.inputs)
    {
        link(net, *net.findPlace(step), *net.findTransition(transition));
    }
    std::vector<std::size_t> outputs = p_outputs;
    outputs.insert(outputs.end(), p_builder.extra.begin(), p_builder.extra.end());
    for (std::size_t const step : outputs)
    {
        link(net, *net.findTransition(transition), *net.findPlace(step));
    }
    p_builder.extra.clear();
    return transition;
}

/// Fires the pending transition into a new step of the column, which becomes
/// the only input of the next transition.
static std::size_t advance(Builder& p_builder, std::string const& p_caption,
                           std::string const& p_action = {})
{
    std::size_t const step = addStep(p_builder.net, p_caption, 0U, p_builder.x, p_builder.y);
    p_builder.y += ROW;
    if (!p_action.empty())
    {
        tpne::Action action;
        action.name = p_action;
        p_builder.net.findPlace(step)->actions.push_back(action);
    }
    (void)fire(p_builder, { step });
    p_builder.inputs = { step };
    return step;
}

/// Replaces every @p_from by @p_to in @p_text.
static std::string replaced(std::string p_text, std::string const& p_from, std::string const& p_to)
{
    for (std::size_t at = p_text.find(p_from); at != std::string::npos;
         at = p_text.find(p_from, at + p_to.size()))
    {
        p_text.replace(at, p_from.size(), p_to);
    }
    return p_text;
}

/// Seconds as a receptivity writes them: "2", "1.5".
static std::string seconds(double p_value)
{
    std::ostringstream text;
    text << p_value;
    return text.str();
}

/// Runs a recipe: one step per skill call, its receptivity done(skill) and
/// what follows " . " in the recipe.
static void runRecipe(Builder& p_builder, std::vector<std::string> const& p_recipe,
                      std::string const& p_part, std::string const& p_to, std::string const& p_on,
                      std::string const& p_where)
{
    for (std::string const& entry : p_recipe)
    {
        if (p_to.empty() && entry.find("{to}") != std::string::npos)
        {
            throw std::runtime_error(p_where + ": '" + entry +
                                     "' gives the part {to} nowhere (set workflow.output)");
        }
        std::string const text =
            replaced(replaced(replaced(entry, "{part}", p_part), "{to}", p_to), "{on}", p_on);
        std::size_t const dot = text.find(" . ");
        std::string const action = (dot == std::string::npos) ? text : text.substr(0U, dot);
        (void)advance(p_builder, action, action);
        p_builder.code = "done(" + action + ")";
        if (dot != std::string::npos)
        {
            p_builder.code += text.substr(dot);
        }
    }
}

/// The shared steps of a buffer: its zone, and one place per product kind.
struct BufferSteps
{
    WorkflowBuffer const* buffer = nullptr;
    /// Step "zone free", or none.
    std::optional<std::size_t> zone;
    /// Step "place free" of each product kind.
    std::map<std::string, std::size_t> places;
    /// World entity of the place of each product kind.
    std::map<std::string, std::string> entities;
    /// Steps where the operator taking parts waits for the zone.
    std::vector<std::size_t> takers;
    /// Transitions where the operator giving parts enters the zone.
    std::vector<std::size_t> givers;
};

/// Checks what the generator needs and lists the parts, lot after lot.
static std::vector<Job> jobsOf(WorkflowDescription const& p_workflow)
{
    std::vector<Job> jobs;
    std::size_t lots = 0U;
    for (WorkflowProduct const& product : p_workflow.products)
    {
        lots = std::max(lots, product.parts.size());
    }
    for (std::size_t lot = 0U; lot < lots; ++lot)
    {
        for (std::size_t p = 0U; p < p_workflow.products.size(); ++p)
        {
            if (lot < p_workflow.products[p].parts.size())
            {
                jobs.push_back({ p, p_workflow.products[p].parts[lot] });
            }
        }
    }
    if (jobs.empty())
    {
        throw std::runtime_error("no product has parts: nothing to make (products[].parts)");
    }
    for (WorkflowOperator const& op : p_workflow.operators)
    {
        if (op.capacity > 1U)
        {
            throw std::runtime_error("operator " + op.id +
                                     ": capacity above 1 cannot be generated (a GRAFCET step "
                                     "holds one token): declare one operator per unit");
        }
    }
    return jobs;
}

/// Index of the operator, of the buffer after it.
static std::size_t operatorIndex(WorkflowDescription const& p_workflow, std::string const& p_id)
{
    auto const found = std::ranges::find(p_workflow.operators, p_id, &WorkflowOperator::id);
    return static_cast<std::size_t>(found - p_workflow.operators.begin());
}

/// What the production graph tells the other graphs.
struct Production
{
    /// Step End.
    std::size_t end = 0U;
    /// Steps where each operator waits between two parts (idle, done).
    std::vector<std::vector<std::size_t>> idle;
};

/// Builds the production graph; a part starts only when @p_start is true
/// ("1": always). Throws what is missing in the workflow.
static Production build(WorkflowDescription const& p_workflow, tpne::Net& p_net,
                        std::string const& p_start)
{
    Production production;
    std::vector<Job> const jobs = jobsOf(p_workflow);
    std::size_t const operators = p_workflow.operators.size();

    // Shared steps on the first row: zones and places of the buffers.
    std::vector<BufferSteps> buffers(operators);
    float x = COLUMN / 2.0F;
    for (WorkflowBuffer const& buffer : p_workflow.buffers)
    {
        BufferSteps& steps = buffers[operatorIndex(p_workflow, buffer.after)];
        steps.buffer = &buffer;
        if (buffer.zone)
        {
            steps.zone = addStep(p_net, buffer.id + " free", 1U, x, 40.0F);
            x += 120.0F;
        }
        for (auto const& [product, entity] : buffer.places)
        {
            steps.places[product] = addStep(p_net, entity + " free", 1U, x, 120.0F);
            steps.entities[product] = entity;
            x += 120.0F;
        }
        if (!buffer.place.empty())
        {
            // One place for every product kind.
            std::size_t const shared = addStep(p_net, buffer.place + " free", 1U, x, 120.0F);
            x += 120.0F;
            for (WorkflowProduct const& product : p_workflow.products)
            {
                steps.places[product.id] = shared;
                steps.entities[product.id] = buffer.place;
            }
        }
    }

    // Step "part done by the operator": given to the next operator.
    std::map<std::pair<std::size_t, std::size_t>, std::size_t> given;
    std::vector<std::size_t> done;
    production.idle.resize(operators);
    for (std::size_t k = 0U; k < operators; ++k)
    {
        WorkflowOperator const& op = p_workflow.operators[k];
        Builder builder{ p_net, {}, {}, {}, 80.0F + COLUMN * static_cast<float>(k), TOP };
        std::optional<std::size_t> idle;
        for (std::size_t j = 0U; j < jobs.size(); ++j)
        {
            WorkflowProduct const& product = p_workflow.products[jobs[j].product];
            std::string const& part = jobs[j].part;
            auto const at = std::ranges::find(product.route, op.id, &WorkflowOperation::on);
            if (at == product.route.end())
            {
                continue;
            }
            std::size_t const r = static_cast<std::size_t>(at - product.route.begin());
            std::string const where = "operator " + op.id + ", part " + part;
            if (!idle)
            {
                idle = addStep(p_net, op.id + " idle", 1U, builder.x, 40.0F);
                production.idle[k].push_back(*idle);
            }

            // The buffer it takes the part from, the one it gives it to.
            BufferSteps* from = nullptr;
            BufferSteps* to = nullptr;
            if (r > 0U)
            {
                from = &buffers[operatorIndex(p_workflow, product.route[r - 1U].on)];
            }
            if (r + 1U < product.route.size())
            {
                to = &buffers[k];
                if (to->buffer == nullptr)
                {
                    throw std::runtime_error(where + ": product " + product.id + " goes from " +
                                             op.id + " to " + product.route[r + 1U].on +
                                             ": declare a buffer after " + op.id);
                }
                if (!to->places.contains(product.id))
                {
                    throw std::runtime_error(where + ": buffer " + to->buffer->id +
                                             " has no place for " + product.id);
                }
            }
            std::string const destination =
                (to != nullptr) ? to->entities.at(product.id) : p_workflow.output;

            // Start: idle, the part given by the previous operator, its place
            // in the next buffer free.
            builder.inputs = { *idle };
            builder.code = p_start;
            if (from != nullptr)
            {
                builder.inputs.push_back(given.at({ j, r - 1U }));
            }
            if (to != nullptr)
            {
                builder.inputs.push_back(to->places.at(product.id));
            }
            if (from != nullptr && from->zone)
            {
                from->takers.push_back(
                    advance(builder, op.id + " waits " + from->buffer->id + " for " + part));
                builder.inputs.push_back(*from->zone);
            }
            runRecipe(builder, op.take, part, destination, op.id, where);
            if (from != nullptr)
            {
                // Taken: its place and the zone are free again.
                builder.extra.push_back(from->places.at(product.id));
                if (from->zone)
                {
                    builder.extra.push_back(*from->zone);
                }
            }
            // The work: a skill (a belt runs) for its duration or until done,
            // or a duration alone (the operator holds the part).
            double const duration = at->duration.value();
            if (duration > 0.0 || !at->skill.empty())
            {
                std::string caption = op.id + " " + at->name + " " + product.id;
                if (duration > 0.0)
                {
                    caption += " (" + seconds(duration) + " s)";
                }
                std::size_t const work = advance(builder, caption, at->skill);
                builder.code = (duration > 0.0)
                                   ? seconds(duration) + "s/X" + std::to_string(work)
                                   : "done(" + at->skill + ")";
            }
            if (to != nullptr && to->zone)
            {
                (void)advance(builder, op.id + " waits " + to->buffer->id);
                builder.inputs.push_back(*to->zone);
                builder.code = "1";
                // The transition entering the zone is the next one.
                to->givers.push_back(p_net.transitions().size());
            }
            runRecipe(builder, op.give, part, destination, op.id, where);

            // End: idle again (or done), the part given to the next operator.
            bool const last = std::none_of(jobs.begin() + static_cast<std::ptrdiff_t>(j) + 1,
                                           jobs.end(),
                                           [&p_workflow, &op](Job const& p_job)
                                           {
                                               auto const& route =
                                                   p_workflow.products[p_job.product].route;
                                               return std::ranges::find(route, op.id,
                                                                        &WorkflowOperation::on) !=
                                                      route.end();
                                           });
            std::vector<std::size_t> outputs;
            idle = addStep(p_net, op.id + (last ? " done" : " idle"), 0U, builder.x, builder.y);
            outputs.push_back(*idle);
            production.idle[k].push_back(*idle);
            if (to != nullptr)
            {
                std::size_t const handed =
                    addStep(p_net, part + " done by " + op.id, 0U, builder.x + 160.0F, builder.y);
                given[{ j, r }] = handed;
                outputs.push_back(handed);
            }
            if (to != nullptr && to->zone)
            {
                builder.extra.push_back(*to->zone);
            }
            (void)fire(builder, outputs);
            builder.y += ROW;
            if (last)
            {
                done.push_back(*idle);
            }
        }
    }

    // In a zone the operator taking parts goes first.
    for (BufferSteps const& steps : buffers)
    {
        if (steps.takers.empty())
        {
            continue;
        }
        std::string guard = "/(";
        for (std::size_t i = 0U; i < steps.takers.size(); ++i)
        {
            guard += (i == 0U ? "X" : " + X") + std::to_string(steps.takers[i]);
        }
        guard += ")";
        for (std::size_t const transition : steps.givers)
        {
            p_net.findTransition(transition)->caption = guard;
        }
    }

    // All the operators done: the end.
    Builder end{ p_net, done, "1", {}, 80.0F, 0.0F };
    for (std::size_t const step : done)
    {
        end.y = std::max(end.y, p_net.position(*p_net.findPlace(step)).y + ROW * 2.0F);
    }
    production.end = advance(end, "End");
    return production;
}

/// Receptivity of another graph: Production at rest, every operator between
/// two parts ("(Production:X1 + Production:X12) . (...)").
static std::string atRest(Production const& p_production)
{
    std::string code;
    for (std::vector<std::size_t> const& steps : p_production.idle)
    {
        if (steps.empty())
        {
            continue;
        }
        code += code.empty() ? "(" : " . (";
        for (std::size_t i = 0U; i < steps.size(); ++i)
        {
            code += (i == 0U ? "Production:X" : " + Production:X") + std::to_string(steps[i]);
        }
        code += ")";
    }
    return code;
}

/// Adds the Safety and Control graphs of the default GEMMA: the production
/// starts a part only in F1; Control leaves A2 (end of cycle) when every
/// operator is between two parts. Returns the End step of Production.
static std::size_t addGemma(WorkflowDescription const& p_workflow, std::vector<tpne::Net>& p_graphs)
{
    tpne::Gemma const gemma = tpne::defaultGemma();
    tpne::Net safety(tpne::TypeOfNet::GRAFCET);
    tpne::Net control(tpne::TypeOfNet::GRAFCET);
    tpne::generateGemma(gemma, safety, control);
    std::map<std::string, std::size_t> const steps = tpne::gemmaSteps(control);

    tpne::Net& production = p_graphs.back();
    Production const built =
        build(p_workflow, production, "Control:X" + std::to_string(steps.at("F1")));
    for (tpne::Transition& transition : control.transitions())
    {
        if (transition.caption == "Production:X0")
        {
            transition.caption = atRest(built);
        }
    }
    p_graphs.insert(p_graphs.begin(), { std::move(safety), std::move(control) });
    return built.end;
}

Result<std::string> workflowGrafcet(WorkflowDescription const& p_workflow,
                                    std::vector<tpne::Net>& p_graphs)
{
    // The border of this generator: what is missing in the workflow becomes
    // a message.
    try
    {
        p_graphs.clear();
        p_graphs.emplace_back(tpne::TypeOfNet::GRAFCET);
        tpne::Net& net = p_graphs.back();
        net.name = "Production";
        net.description = "Generated from the workflow " + p_workflow.id;
        std::size_t const end = p_workflow.gemma.empty() ? build(p_workflow, net, "1").end
                                                         : addGemma(p_workflow, p_graphs);
        return "Production:X" + std::to_string(end);
    }
    catch (std::runtime_error const& error)
    {
        return failure(p_workflow.file.string() + ": " + error.what());
    }
}

} // namespace robotik
