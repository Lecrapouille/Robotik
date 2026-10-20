// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Planning/PlanExecutor.hpp"
#include "Robotik/Common/Must.hpp"
#include "Robotik/Task/Perception/WorldModel.hpp"
#include "Robotik/Task/Planning/PlanGraph.hpp"

#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Task/RobotContext.hpp"
#include "Robotik/Task/Scheduler.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <functional>
#include <set>
#include <sstream>
#include <stdexcept>

namespace robotik
{

/// @brief A PDDL name: a letter, then letters, digits, '_' or '-'.
static bool pddlName(std::string_view p_name)
{
    if (p_name.empty() ||
        std::isalpha(static_cast<unsigned char>(p_name.front())) == 0)
    {
        return false;
    }
    return std::all_of(p_name.begin(),
                       p_name.end(),
                       [](char p_c)
                       {
                           return std::isalnum(
                                      static_cast<unsigned char>(p_c)) != 0 ||
                                  p_c == '_' || p_c == '-';
                       });
}

/// @brief YAML double-quoted scalar.
static std::string yamlQuoted(std::string_view p_text)
{
    std::string result = "\"";
    for (char const c : p_text)
    {
        if (c == '"' || c == '\\')
        {
            result += '\\';
        }
        result += c;
    }
    return result + "\"";
}

/// Reads a whole file, or throws when it cannot be opened.
static std::string readFile(std::filesystem::path const& p_file)
{
    std::ifstream in(p_file);
    if (!in)
    {
        throw std::runtime_error("cannot read the PDDL domain " +
                                 p_file.string());
    }
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

/// @brief "pick(red_cube)" for a step.
static std::string label(GroundAction const& p_step)
{
    return Atom{ p_step.name, p_step.args }.toString();
}

char const* toString(PlanState p_state)
{
    switch (p_state)
    {
        case PlanState::Planning:
            return "planning";
        case PlanState::Waiting:
            return "waiting";
        case PlanState::Executing:
            return "executing";
        case PlanState::Succeeded:
            return "succeeded";
        case PlanState::Failed:
            return "failed";
    }
    return "?";
}

char const* toString(StepStatus p_status)
{
    switch (p_status)
    {
        case StepStatus::Pending:
            return "pending";
        case StepStatus::Skipped:
            return "skipped";
        case StepStatus::Done:
            return "done";
        case StepStatus::Failed:
            return "failed";
    }
    return "?";
}

// ----------------------------------------------------------------------------
PlanExecutor::PlanExecutor(PlannerSpec p_spec,
                           WorldDescription const& p_world,
                           PlanningDomain const& p_domain,
                           std::string const& p_name,
                           bt::NodeFactory& p_factory,
                           SkillScheduler& p_skills,
                           std::unique_ptr<PlanningEngine> p_engine)
    : m_spec(std::move(p_spec)),
      m_domain(p_domain),
      m_factory(p_factory),
      m_skills(p_skills),
      m_engine(std::move(p_engine))
{
    for (char const c : lowerCase(p_name))
    {
        m_name += std::isalnum(static_cast<unsigned char>(c)) != 0 ? c : '_';
    }
    if (m_name.empty() ||
        std::isalpha(static_cast<unsigned char>(m_name.front())) == 0)
    {
        m_name = "p_" + m_name;
    }
    load(p_world, p_domain);
}

PlanExecutor::~PlanExecutor() = default;

// ----------------------------------------------------------------------------
void PlanExecutor::load(WorldDescription const& p_world,
                        PlanningDomain const& p_domain)
{
    std::string const file = m_spec.domain.string();
    m_domain_text = readFile(m_spec.domain);
    try
    {
        m_schema = must(DomainSchema::parse(m_domain_text));
    }
    catch (std::runtime_error const& error)
    {
        throw std::runtime_error(file + ": " + error.what());
    }
    if (std::string const diagnostics = m_engine->load(m_domain_text, file);
        !diagnostics.empty())
    {
        throw std::runtime_error("PDDL domain rejected by the planner:\n" +
                                 diagnostics);
    }

    std::vector<std::string> errors;
    auto known = [&](std::string_view p_type)
    { return p_type == "object" || m_schema.types.contains(p_type); };

    // --- Objects: the entities with a kind ----------------------------------
    std::set<std::string> names;
    for (auto const& [constant, type] : m_schema.constants)
    {
        names.insert(constant);
    }
    for (EntitySpec const& entity : p_world.entities)
    {
        if (entity.kind.empty())
        {
            continue;
        }
        PlanningObject object{ entity.id, lowerCase(entity.id), entity.kind };
        if (!pddlName(object.name))
        {
            errors.push_back(
                "entity '" + entity.id +
                "': its id is not a PDDL name (letters, digits, '_', '-')");
        }
        else if (!names.insert(object.name).second)
        {
            errors.push_back("entity '" + entity.id +
                             "': same PDDL name as another entity or constant "
                             "(case is ignored)");
        }
        if (!known(object.type))
        {
            errors.push_back("entity '" + entity.id + "': kind '" +
                             object.type + "' is not a type of the domain");
        }
        m_objects.push_back(std::move(object));
    }
    if (m_objects.empty() && m_schema.constants.empty())
    {
        errors.push_back(
            "no world entity has a kind: the problem has no object");
    }

    // --- Mission: observers and bindings ------------------------------------
    std::set<std::string> fact_predicates;
    for (Literal const& fact : m_spec.facts)
    {
        fact_predicates.insert(fact.atom.predicate);
    }
    for (auto const& [name, predicate] : m_schema.predicates)
    {
        if (!predicate.derived && p_domain.observer(name) == nullptr &&
            !fact_predicates.contains(name))
        {
            errors.push_back(
                "predicate '" + name +
                "' has no observer (PlanningDomain::observe in the mission) "
                "and no fact (planner.facts)");
        }
    }
    for (auto const& [name, observer] : p_domain.observers())
    {
        if (!m_schema.predicates.contains(name))
        {
            errors.push_back("observer '" + name +
                             "' is not a predicate of the domain");
        }
    }
    for (auto const& [name, binding] : p_domain.bindings())
    {
        if (!m_schema.actions.contains(name))
        {
            errors.push_back("binding '" + name +
                             "' is not an action of the domain");
        }
    }
    for (auto const& [name, action] : m_schema.actions)
    {
        ActionBinding const* binding = p_domain.binding(name);
        if (binding == nullptr)
        {
            errors.push_back(
                "action '" + name +
                "' has no binding (PlanningDomain::bind in the mission)");
            continue;
        }
        std::vector<std::string> const args(action.parameters.size(), "x");
        try
        {
            for (std::string const& node : binding->nodes)
            {
                (void)expand(node, args);
            }
            for (auto const& [key, value] : binding->parameters)
            {
                (void)expand(value, args);
            }
        }
        catch (std::out_of_range const& error)
        {
            errors.push_back("binding of '" + name + "': " + error.what());
        }
    }

    // --- Scenario: facts and goal -------------------------------------------
    auto typeOf = [&](std::string const& p_name) -> std::string const*
    {
        if (auto const constant = m_schema.constants.find(p_name);
            constant != m_schema.constants.end())
        {
            return &constant->second;
        }
        for (PlanningObject const& object : m_objects)
        {
            if (object.name == p_name)
            {
                return &object.type;
            }
        }
        return nullptr;
    };
    auto checkLiteral =
        [&](Literal const& p_literal, std::string const& p_where)
    {
        auto const predicate =
            m_schema.predicates.find(p_literal.atom.predicate);
        if (predicate == m_schema.predicates.end())
        {
            errors.push_back(p_where + " " + p_literal.toString() +
                             ": unknown predicate");
            return;
        }
        if (predicate->second.types.size() != p_literal.atom.args.size())
        {
            errors.push_back(p_where + " " + p_literal.toString() +
                             ": expects " +
                             std::to_string(predicate->second.types.size()) +
                             " argument(s)");
            return;
        }
        for (std::size_t i = 0u; i < p_literal.atom.args.size(); ++i)
        {
            std::string const* type = typeOf(p_literal.atom.args[i]);
            if (type == nullptr)
            {
                errors.push_back(
                    p_where + " " + p_literal.toString() + ": '" +
                    p_literal.atom.args[i] +
                    "' is neither an entity with a kind nor a constant");
            }
            else if (!m_schema.isA(*type, predicate->second.types[i]))
            {
                errors.push_back(p_where + " " + p_literal.toString() + ": '" +
                                 p_literal.atom.args[i] + "' is a " + *type +
                                 ", not a " + predicate->second.types[i]);
            }
        }
    };
    for (Literal const& fact : m_spec.facts)
    {
        checkLiteral(fact, "planner.facts:");
        if (!fact.positive)
        {
            errors.push_back(
                "planner.facts: " + fact.toString() +
                ": facts are true atoms (what is not listed is false)");
        }
    }
    for (Literal const& goal : m_spec.goal)
    {
        checkLiteral(goal, "planner.goal:");
    }

    if (!errors.empty())
    {
        std::string text = "the planner setup has " +
                           std::to_string(errors.size()) + " problem(s):";
        for (std::string const& error : errors)
        {
            text += "\n  - " + error;
        }
        throw std::runtime_error(text);
    }
}

// ----------------------------------------------------------------------------
void PlanExecutor::reset(Seed p_seed)
{
    m_seed = p_seed.value;
    m_state = PlanState::Planning;
    m_resume = Seconds{};
    m_replans = 0u;
    m_failure.clear();
    m_history.clear();
    m_plan.clear();
    m_done = 0u;
    m_steps.clear();
    m_after.clear();
    m_order.clear();
    m_started.clear();
    m_check_failure.clear();
    m_tree.reset();
}

// ----------------------------------------------------------------------------
void PlanExecutor::replan(std::string p_cause)
{
    if (m_tree)
    {
        m_tree->halt();
    }
    m_skills.cancelAll();
    m_failure = std::move(p_cause);
    m_state = PlanState::Planning;
}

// ----------------------------------------------------------------------------
bt::Status PlanExecutor::tick(RobotContext const& p_context)
{
    m_context = &p_context;
    if (m_state == PlanState::Waiting && p_context.time >= m_resume)
    {
        m_state = PlanState::Planning;
    }
    if (m_state == PlanState::Planning)
    {
        plan(p_context, m_history.empty() ? std::string("start") : m_failure);
    }
    if (m_state == PlanState::Executing)
    {
        m_check_failure.clear();
        switch (m_tree->tick())
        {
            case bt::Status::SUCCESS:
                if (std::string const missing = goalReached(p_context);
                    missing.empty())
                {
                    m_state = PlanState::Succeeded;
                }
                else
                {
                    retry(p_context, missing);
                }
                break;
            case bt::Status::FAILURE:
            {
                std::string cause = m_check_failure.empty()
                                        ? stepFailure(p_context)
                                        : m_check_failure;
                if (std::size_t const failed = failingStep();
                    failed < m_steps.size())
                {
                    m_steps[failed] = StepStatus::Failed;
                }
                m_tree->halt();
                m_skills.cancelAll();
                retry(p_context, std::move(cause));
                break;
            }
            default:
                break;
        }
    }
    m_context = nullptr;
    switch (m_state)
    {
        case PlanState::Succeeded:
            return bt::Status::SUCCESS;
        case PlanState::Failed:
            return bt::Status::FAILURE;
        default:
            return bt::Status::RUNNING;
    }
}

// ----------------------------------------------------------------------------
void PlanExecutor::plan(RobotContext const& p_context, std::string p_cause)
{
    PlanningRecord record;
    record.time = p_context.time;
    record.cause = std::move(p_cause);
    record.observed = observe(p_context);
    record.problem = problemOf(record.observed);

    PlanningOptions options;
    options.search = m_spec.search;
    options.seed = m_seed;
    options.timeout = std::chrono::milliseconds(static_cast<std::int64_t>(
        std::max(0.001, m_spec.timeout.value()) * 1000.0));
    record.result = m_engine->solve(record.problem, options);

    m_plan.clear();
    m_done = 0u;
    m_tree.reset();
    if (record.result.solved())
    {
        for (GroundAction const& step : record.result.steps)
        {
            m_plan.push_back(
                { step.name, ids(step.args), step.start, step.duration });
        }
    }
    m_steps.assign(m_plan.size(), StepStatus::Pending);
    m_history.push_back(std::move(record));
    PlanningResult const& result = m_history.back().result;

    if (!result.solved())
    {
        std::string why = std::string("planner: ") + toString(result.status);
        if (std::size_t const end = result.diagnostics.find('\n');
            end != 0u && !result.diagnostics.empty())
        {
            why += " (" + result.diagnostics.substr(0u, end) + ")";
        }
        // A wrong problem will not get better by waiting.
        if (result.status == PlanningResult::Status::Invalid)
        {
            fail(std::move(why));
        }
        else
        {
            retry(p_context, std::move(why));
        }
        return;
    }
    if (m_plan.empty())
    {
        m_state = PlanState::Succeeded;
        return;
    }
    compile();
}

// ----------------------------------------------------------------------------
void PlanExecutor::compile()
{
    m_order = partialOrder();
    m_started.assign(m_plan.size(), false);
    m_start_times.assign(m_plan.size(), Seconds(0.0));
    m_after.assign(m_plan.size(), {});
    for (std::size_t k = 0u; k < m_order.size(); ++k)
    {
        for (PlanEvent const& event : m_order[k].start_after)
        {
            m_after[k].push_back(event.step);
        }
    }
    bool const chain = sequential(m_order);
    // "1" for the end of step 1, "start of 1" for its start.
    auto events = [](std::vector<PlanEvent> const& p_events)
    {
        std::string text;
        for (PlanEvent const& event : p_events)
        {
            text += (text.empty() ? "" : ", ") +
                    std::string(event.end ? "" : "start of ") +
                    std::to_string(event.step + 1u);
        }
        return text;
    };
    std::ostringstream yaml;
    if (chain)
    {
        yaml << "BehaviorTree:\n  Sequence:\n    name: \"Plan\"\n    "
                "children:\n";
    }
    else
    {
        // Every step at once; each waits for its predecessors, runs once,
        // and the first failure fails the plan.
        yaml << "BehaviorTree:\n  Parallel:\n    name: \"Plan\"\n"
             << "    success_on_all: true\n    fail_on_all: false\n    "
                "children:\n";
    }
    for (std::size_t k = 0u; k < m_plan.size(); ++k)
    {
        GroundAction const& step = m_plan[k];
        std::string const title = std::to_string(k + 1u) + ". " + label(step);
        std::string indent = "      ";
        if (!chain)
        {
            std::string const wait =
                m_order[k].start_after.empty()
                    ? "Start " + title
                    : "After " + events(m_order[k].start_after) + ": " + title;
            m_factory.registerNode(
                wait,
                [this, k]()
                {
                    return std::make_unique<bt::CallbackLeaf>(
                        [this, k]() { return ready(k); });
                });
            yaml << indent << "- RunOnce:\n"
                 << indent << "    child:\n"
                 << indent << "      - Sequence:\n"
                 << indent << "          name: " << yamlQuoted(title) << "\n"
                 << indent << "          children:\n"
                 << indent << "            - Action:\n"
                 << indent << "                name: " << yamlQuoted(wait)
                 << "\n";
            indent += "            ";
        }
        yaml << indent << "- SubTree:\n"
             << indent << "    name: " << yamlQuoted(title) << "\n"
             << indent << "    reference: step_" << k << "\n";
        ActionBinding const* binding = m_domain.binding(step.name);
        if (binding != nullptr && !binding->parameters.empty())
        {
            yaml << indent << "    parameters:\n";
            for (auto const& [key, value] : binding->parameters)
            {
                yaml << indent << "      " << key << ": "
                     << yamlQuoted(expand(value, step.args)) << "\n";
            }
        }
    }
    yaml << "SubTrees:\n";
    for (std::size_t k = 0u; k < m_plan.size(); ++k)
    {
        GroundAction const& step = m_plan[k];
        std::string const title = std::to_string(k + 1u) + ". " + label(step);
        // A step whose effects already hold is skipped: the beliefs may have
        // changed since the plan (an object seen on the way).
        std::string const skip = "Done already? " + title;
        m_factory.registerNode(
            skip,
            [this, k]()
            {
                return std::make_unique<bt::CallbackLeaf>(
                    [this, k]()
                    {
                        return m_context != nullptr ? this->skip(k, *m_context)
                                                    : bt::Status::FAILURE;
                    });
            });
        yaml << "  step_" << k
             << ":\n    Selector:\n      name: " << yamlQuoted(title)
             << "\n      children:\n        - Action:\n            name: "
             << yamlQuoted(skip)
             << "\n        - Sequence:\n            name: \"Do\"\n"
             << "            children:\n";
        for (std::string const& node : nodes(step))
        {
            yaml << "              - Action:\n                  name: "
                 << yamlQuoted(node) << "\n";
        }
        // A step of a temporal plan may have to last until others end (a
        // tray held while the other arm works).
        if (!m_order[k].end_after.empty())
        {
            std::string const until =
                "Until " + events(m_order[k].end_after) + ": " + title;
            m_factory.registerNode(
                until,
                [this, k]()
                {
                    return std::make_unique<bt::CallbackLeaf>(
                        [this, k]() { return mayEnd(k); });
                });
            yaml << "              - Action:\n                  name: "
                 << yamlQuoted(until) << "\n";
        }
        // The check node marks the step done: always there, it only reads
        // the beliefs when verify is on.
        std::string const check = "Check " + title;
        m_factory.registerNode(
            check,
            [this, k]()
            {
                return std::make_unique<bt::CallbackLeaf>(
                    [this, k]()
                    {
                        return m_context != nullptr ? this->check(k, *m_context)
                                                    : bt::Status::FAILURE;
                    });
            });
        yaml << "              - Action:\n                  name: "
             << yamlQuoted(check) << "\n";
    }

    std::string const text = yaml.str();
    m_history.back().tree = text;
    m_blackboard = std::make_shared<bt::Blackboard>();
    auto built = bt::Builder::fromText(m_factory, text, m_blackboard);
    if (!built)
    {
        fail("behavior tree of the plan: " + built.getError() +
             " (a binding names a node that is not a skill?)");
        return;
    }
    m_tree = std::move(built.getValue());
    m_tree->clock(m_clock);
    m_state = PlanState::Executing;
}

// ----------------------------------------------------------------------------
std::vector<std::string> PlanExecutor::nodes(GroundAction const& p_step) const
{
    std::vector<std::string> result;
    if (ActionBinding const* binding = m_domain.binding(p_step.name))
    {
        for (std::string const& node : binding->nodes)
        {
            result.push_back(expand(node, p_step.args));
        }
    }
    return result;
}

// ----------------------------------------------------------------------------
std::vector<Literal> PlanExecutor::effects(std::size_t p_step,
                                           bool& p_partial) const
{
    std::vector<Literal> result;
    p_partial = true;
    if (p_step >= m_plan.size())
    {
        return result;
    }
    GroundAction const& step = m_plan[p_step];
    auto const action = m_schema.actions.find(step.name);
    if (action == m_schema.actions.end())
    {
        return result;
    }
    p_partial = action->second.partial;
    auto const& parameters = action->second.parameters;
    for (Literal const& effect : action->second.effects)
    {
        Literal ground{ { effect.atom.predicate, {} }, effect.positive };
        for (std::string const& arg : effect.atom.args)
        {
            auto const found =
                std::find_if(parameters.begin(),
                             parameters.end(),
                             [&](Parameter const& p_parameter)
                             { return "?" + p_parameter.name == arg; });
            std::size_t const index =
                static_cast<std::size_t>(found - parameters.begin());
            ground.atom.args.push_back(
                index < step.args.size() ? lowerCase(step.args[index]) : arg);
        }
        result.push_back(std::move(ground));
    }
    return result;
}

// ----------------------------------------------------------------------------
bt::Status PlanExecutor::check(std::size_t p_step,
                               RobotContext const& p_context)
{
    if (p_step >= m_plan.size())
    {
        return bt::Status::FAILURE;
    }
    if (m_spec.verify)
    {
        bool partial = false;
        for (Literal const& effect : effects(p_step, partial))
        {
            Truth const value = evaluate(effect.atom, p_context);
            if (m_spec.observed && !seenSince(effect.atom, p_context, m_start_times[p_step]))
            {
                m_check_failure = std::to_string(p_step + 1u) + ". " +
                                  label(m_plan[p_step]) + ": " + effect.toString() +
                                  " expected, not observed since the step started";
                return bt::Status::FAILURE;
            }
            if (value != Truth::Unknown &&
                (value == Truth::True) != effect.positive)
            {
                m_check_failure =
                    std::to_string(p_step + 1u) + ". " + label(m_plan[p_step]) +
                    ": " + effect.toString() + " expected, the beliefs say " +
                    toString(value);
                return bt::Status::FAILURE;
            }
        }
    }
    finish(p_step, StepStatus::Done);
    return bt::Status::SUCCESS;
}

// ----------------------------------------------------------------------------
bt::Status PlanExecutor::skip(std::size_t p_step, RobotContext const& p_context)
{
    bool partial = true;
    std::vector<Literal> const expected = effects(p_step, partial);
    if (partial || expected.empty())
    {
        return bt::Status::FAILURE;
    }
    // Every effect must be known to hold: unknown is not enough to skip.
    for (Literal const& effect : expected)
    {
        Truth const value = evaluate(effect.atom, p_context);
        if (value == Truth::Unknown ||
            (value == Truth::True) != effect.positive)
        {
            return bt::Status::FAILURE;
        }
    }
    finish(p_step, StepStatus::Skipped);
    return bt::Status::SUCCESS;
}

// ----------------------------------------------------------------------------
bool PlanExecutor::reached(PlanEvent const& p_event) const
{
    StepStatus const status = m_steps[p_event.step];
    bool const over =
        status == StepStatus::Done || status == StepStatus::Skipped;
    return p_event.end ? over : over || m_started[p_event.step];
}

// ----------------------------------------------------------------------------
bool PlanExecutor::running(std::size_t p_step) const
{
    if (m_state != PlanState::Executing || p_step >= m_steps.size() ||
        p_step >= m_order.size() || m_steps[p_step] != StepStatus::Pending)
    {
        return false;
    }
    return std::all_of(m_order[p_step].start_after.begin(),
                       m_order[p_step].start_after.end(),
                       [this](PlanEvent const& p_event)
                       { return reached(p_event); });
}

// ----------------------------------------------------------------------------
bt::Status PlanExecutor::ready(std::size_t p_step)
{
    for (PlanEvent const& event : m_order[p_step].start_after)
    {
        if (!reached(event))
        {
            return bt::Status::RUNNING;
        }
    }
    m_started[p_step] = true;
    m_start_times[p_step] = m_context != nullptr ? m_context->time : Seconds(0.0);
    return bt::Status::SUCCESS;
}

// ----------------------------------------------------------------------------
bt::Status PlanExecutor::mayEnd(std::size_t p_step) const
{
    for (PlanEvent const& event : m_order[p_step].end_after)
    {
        if (!reached(event))
        {
            return bt::Status::RUNNING;
        }
    }
    return bt::Status::SUCCESS;
}

// ----------------------------------------------------------------------------
void PlanExecutor::finish(std::size_t p_step, StepStatus p_status)
{
    m_steps[p_step] = p_status;
    m_done = static_cast<std::size_t>(std::count_if(
        m_steps.begin(),
        m_steps.end(),
        [](StepStatus p_s)
        { return p_s == StepStatus::Done || p_s == StepStatus::Skipped; }));
}

// ----------------------------------------------------------------------------
std::size_t PlanExecutor::failingStep() const
{
    std::size_t first = m_plan.size();
    for (std::size_t k = 0u; k < m_plan.size(); ++k)
    {
        if (m_steps[k] != StepStatus::Pending)
        {
            continue;
        }
        for (std::string const& node : nodes(m_plan[k]))
        {
            SkillId const id = m_skills.find(node);
            if (id != NO_SKILL && (m_skills.state(id) == SkillState::Failed ||
                                   m_skills.state(id) == SkillState::Cancelled))
            {
                return k;
            }
        }
        bool const going =
            k < m_order.size() &&
            std::all_of(
                m_order[k].start_after.begin(),
                m_order[k].start_after.end(),
                [this](PlanEvent const& p_event) { return reached(p_event); });
        if (first == m_plan.size() && going)
        {
            first = k;
        }
    }
    return first;
}

// ----------------------------------------------------------------------------
std::vector<StepOrder> PlanExecutor::partialOrder() const
{
    std::vector<StepOrder> chain(m_plan.size());
    for (std::size_t k = 1u; k < m_plan.size(); ++k)
    {
        chain[k].start_after = { { k - 1u, true } };
    }
    if (!m_spec.parallel)
    {
        return chain;
    }
    std::vector<StepFootprint> prints;
    for (GroundAction const& step : m_plan)
    {
        auto const action = m_schema.actions.find(step.name);
        if (action == m_schema.actions.end())
        {
            return chain; // Unknown to RobotIK: keep the plan as it is.
        }
        StepFootprint print = footprint(action->second, step);
        for (std::string const& node : nodes(step))
        {
            if (SkillId const id = m_skills.find(node); id != NO_SKILL)
            {
                std::vector<ResourceRequirement> const& needed =
                    m_skills.description(id).resources;
                print.resources.insert(
                    print.resources.end(), needed.begin(), needed.end());
            }
        }
        prints.push_back(std::move(print));
    }
    return orderSteps(prints);
}

// ----------------------------------------------------------------------------
bool PlanExecutor::seenSince(Atom const& p_atom,
                             RobotContext const& p_context,
                             Seconds p_since) const
{
    if (evaluate(p_atom, p_context) == Truth::Unknown)
    {
        return false; // the beliefs cannot tell
    }
    // Every object of the atom the robot has beliefs about must have been
    // observed again since the step started.
    for (std::string const& id : ids(p_atom.args))
    {
        WorldObject const* belief = p_context.world_model.find(id);
        if (belief != nullptr && belief->seen < p_since)
        {
            return false;
        }
    }
    return true;
}

// ----------------------------------------------------------------------------
Truth PlanExecutor::evaluate(Atom const& p_atom,
                             RobotContext const& p_context) const
{
    if (Observer const* observer = m_domain.observer(p_atom.predicate))
    {
        std::vector<std::string> const args = ids(p_atom.args);
        // A belief too old says nothing any more.
        for (std::string const& id : args)
        {
            WorldObject const* belief = p_context.world_model.find(id);
            if (belief != nullptr && belief->observed() &&
                p_context.time - belief->seen > m_max_age)
            {
                return Truth::Unknown;
            }
        }
        return (*observer)(p_context, args);
    }
    auto const predicate = m_schema.predicates.find(p_atom.predicate);
    if (predicate == m_schema.predicates.end() || predicate->second.derived)
    {
        return Truth::Unknown;
    }
    // A static predicate: true when listed in the facts.
    bool const listed = std::any_of(m_spec.facts.begin(),
                                    m_spec.facts.end(),
                                    [&](Literal const& p_fact)
                                    { return p_fact.atom == p_atom; });
    return truth(listed);
}

// ----------------------------------------------------------------------------
std::string PlanExecutor::goalReached(RobotContext const& p_context) const
{
    for (Literal const& goal : m_spec.goal)
    {
        Truth const value = evaluate(goal.atom, p_context);
        if (value != Truth::Unknown && (value == Truth::True) != goal.positive)
        {
            return "plan done but goal " + goal.toString() + " not reached";
        }
    }
    return {};
}

// ----------------------------------------------------------------------------
std::string PlanExecutor::stepFailure(RobotContext const& p_context) const
{
    std::size_t const failed = failingStep();
    if (failed >= m_plan.size())
    {
        return "the plan failed";
    }
    GroundAction const& step = m_plan[failed];
    std::string const title = std::to_string(failed + 1u) + ". " + label(step);
    for (std::string const& node : nodes(step))
    {
        SkillId const id = m_skills.find(node);
        if (id == NO_SKILL)
        {
            continue;
        }
        SkillState const state = m_skills.state(id);
        if (state != SkillState::Failed && state != SkillState::Cancelled)
        {
            continue;
        }
        SkillReason const reason = m_skills.reason(id);
        std::string text = title + ": skill " + node + " ";
        if (reason == SkillReason::Failed)
        {
            std::string const why = m_skills.skill(id).why();
            return text + (why.empty() ? "failed (it gave no reason)"
                                       : "failed: " + why);
        }
        text += toString(state) + std::string(" (") + toString(reason);
        if (ResourceId const blocker = m_skills.blocker(id);
            blocker != NO_RESOURCE)
        {
            text += ": " + p_context.robot.resources().name(blocker);
        }
        return text + ")";
    }
    return title + " failed";
}

// ----------------------------------------------------------------------------
void PlanExecutor::fail(std::string p_why)
{
    m_failure = std::move(p_why);
    m_state = PlanState::Failed;
}

// ----------------------------------------------------------------------------
void PlanExecutor::retry(RobotContext const& p_context, std::string p_cause)
{
    if (m_replans >= m_spec.max_replans)
    {
        fail(p_cause + " (no replan left)");
        return;
    }
    ++m_replans;
    m_failure = std::move(p_cause);
    m_resume = p_context.time + m_spec.replan_delay;
    m_state = PlanState::Waiting;
}

// ----------------------------------------------------------------------------
std::vector<std::string>
PlanExecutor::ids(std::vector<std::string> const& p_names) const
{
    std::vector<std::string> result;
    result.reserve(p_names.size());
    for (std::string const& name : p_names)
    {
        auto const object = std::find_if(m_objects.begin(),
                                         m_objects.end(),
                                         [&](PlanningObject const& p_object)
                                         { return p_object.name == name; });
        result.push_back(object != m_objects.end() ? object->id : name);
    }
    return result;
}

// ----------------------------------------------------------------------------
std::vector<ObservedAtom>
PlanExecutor::observe(RobotContext const& p_context) const
{
    std::vector<ObservedAtom> result;
    for (auto const& [name, observer] : m_domain.observers())
    {
        auto const predicate = m_schema.predicates.find(name);
        if (predicate == m_schema.predicates.end())
        {
            continue;
        }
        // Candidates of each argument: objects and constants of its type.
        std::vector<std::vector<std::string>> candidates;
        for (std::string const& type : predicate->second.types)
        {
            std::vector<std::string> names;
            for (PlanningObject const& object : m_objects)
            {
                if (m_schema.isA(object.type, type))
                {
                    names.push_back(object.name);
                }
            }
            for (auto const& [constant, constant_type] : m_schema.constants)
            {
                if (m_schema.isA(constant_type, type))
                {
                    names.push_back(constant);
                }
            }
            candidates.push_back(std::move(names));
        }
        std::vector<std::string> tuple(candidates.size());
        std::function<void(std::size_t)> visit = [&](std::size_t p_index)
        {
            if (p_index == candidates.size())
            {
                std::vector<std::string> const args = ids(tuple);
                result.push_back(
                    { Atom{ name, tuple }, observer(p_context, args) });
                return;
            }
            for (std::string const& candidate : candidates[p_index])
            {
                tuple[p_index] = candidate;
                visit(p_index + 1u);
            }
        };
        visit(0u);
    }
    return result;
}

// ----------------------------------------------------------------------------
std::string PlanExecutor::problem(RobotContext const& p_context) const
{
    return problemOf(observe(p_context));
}

// ----------------------------------------------------------------------------
std::string
PlanExecutor::problemOf(std::vector<ObservedAtom> const& p_observed) const
{
    std::ostringstream text;
    text << "; Generated by RobotIK from the world, the scenario and the "
            "beliefs.\n"
         << "(define (problem " << m_name << ")\n  (:domain " << m_schema.name
         << ")\n";
    if (!m_objects.empty())
    {
        text << "  (:objects\n";
        for (PlanningObject const& object : m_objects)
        {
            text << "    " << object.name << " - " << object.type << "\n";
        }
        text << "  )\n";
    }
    text << "  (:init\n";
    std::set<std::string> written;
    if (!m_spec.facts.empty())
    {
        text << "    ; facts of the scenario\n";
        for (Literal const& fact : m_spec.facts)
        {
            if (written.insert(fact.atom.toPddl()).second)
            {
                text << "    " << fact.atom.toPddl() << "\n";
            }
        }
    }
    text << "    ; observed on the beliefs\n";
    for (auto const& [atom, value] : p_observed)
    {
        if (value == Truth::True && written.insert(atom.toPddl()).second)
        {
            text << "    " << atom.toPddl() << "\n";
        }
    }
    text << "  )\n";
    if (!m_spec.goal_pddl.empty())
    {
        text << "  (:goal " << m_spec.goal_pddl << ")\n";
    }
    else
    {
        text << "  (:goal (and\n";
        for (Literal const& goal : m_spec.goal)
        {
            text << "    " << goal.toPddl() << "\n";
        }
        text << "  ))\n";
    }
    text << ")\n";
    return text.str();
}

} // namespace robotik
