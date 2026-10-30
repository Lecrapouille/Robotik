// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Grafcet/GrafcetController.hpp"

#include "Robotik/Task/CellDescription.hpp"
#include "Robotik/Task/Planning/PlanningDomain.hpp"
#include "Robotik/Task/RobotContext.hpp"
#include "Robotik/Task/Scheduler.hpp"
#include "Robotik/Task/Workflow/WorkflowGrafcet.hpp"

// Only this folder includes TimedPetriNetEditor (make check-layers).
#include "IO/Exports/Exports.hpp"
#include "IO/Imports/Imports.hpp"
#include "PetriNet/GrafcetEngine.hpp"
#include "PetriNet/PetriNet.hpp"

#include <span>

namespace robotik
{

// ****************************************************************************
/// Where the value of an input of the GRAFCET comes from.
// ****************************************************************************
struct GrafcetInput
{
    enum class Kind : std::uint8_t
    {
        Done,     ///< done(S): the run of skill S succeeded.
        Failed,   ///< failed(S): the run of skill S failed or was cancelled.
        Running,  ///< running(S): skill S waits or runs.
        Predicate, ///< predicate(args) of the planning domain.
        Operator, ///< set by the operator (button), the scenario or the mission.
    };

    std::string name;
    Kind kind = Kind::Operator;
    SkillId skill = NO_SKILL;
    Observer const* observer = nullptr;
    std::vector<std::string> args;
};

// ****************************************************************************
/// An action of the GRAFCET: a skill requested while the action is true.
// ****************************************************************************
struct GrafcetAction
{
    std::string name;
    SkillId skill = NO_SKILL;
    /// Output at the previous cycle (rising and falling edges).
    bool previous = false;
};

// ****************************************************************************
/// Gives the end conditions of the scenario (success, failure) the values of
/// the inputs and the steps of the graphs.
// ****************************************************************************
class EndConditions : public tpne::Receptivity::Resolver
{
public:

    explicit EndConditions(std::vector<tpne::Net>& p_graphs) : m_graphs(p_graphs) {}

    int const* input(std::string const& p_name) override
    {
        return &values.emplace(p_name, 0).first->second;
    }

    tpne::Place const* step(std::string const& p_graph, size_t p_id) override
    {
        for (tpne::Net& net : m_graphs)
        {
            if (p_graph.empty() || net.name == p_graph)
            {
                if (tpne::Place const* place = net.findPlace(p_id))
                {
                    return place;
                }
                if (!p_graph.empty())
                {
                    break;
                }
            }
        }
        errors.push_back("unknown step " + (p_graph.empty() ? "" : p_graph + ":") +
                         "X" + std::to_string(p_id));
        return nullptr;
    }

    /// Input values read by the conditions, copied from the engine.
    std::map<std::string, int> values;
    std::vector<std::string> errors;

private:

    std::vector<tpne::Net>& m_graphs;
};

struct GrafcetController::Impl
{
    Impl(GrafcetSpec const& p_spec, SkillScheduler& p_skills,
         PlanningDomain const& p_domain)
        : spec(p_spec), skills(p_skills), domain(p_domain), conditions(graphs)
    {}

    GrafcetSpec spec;
    SkillScheduler& skills;
    PlanningDomain const& domain;
    /// Graphs of the file. Not resized after the engine got their address.
    std::vector<tpne::Net> graphs;
    tpne::GrafcetEngine engine;
    std::vector<GrafcetInput> inputs;
    std::vector<GrafcetAction> actions;
    EndConditions conditions;
    tpne::Receptivity success;
    tpne::Receptivity failure;
    /// Errors met while running (unstable graphs ...), kept for the report.
    std::vector<std::string> warnings;
    /// Next event of the scenario to apply (GrafcetSpec::events).
    std::size_t next_event = 0U;
    /// Push buttons pressed: back to 0 after the next cycle.
    std::vector<std::string> pulses;
};

/// Splits "name(a, b)" into "name" and its arguments ("a", "b"). The
/// arguments keep their own parentheses: done(Grasp(cube)) -> Grasp(cube).
static std::string splitCall(std::string const& p_text, std::vector<std::string>& p_args)
{
    p_args.clear();
    std::size_t const open = p_text.find('(');
    if (open == std::string::npos || p_text.back() != ')')
    {
        return p_text;
    }
    std::string const inside = p_text.substr(open + 1, p_text.size() - open - 2);
    int depth = 0;
    std::string arg;
    for (char const c : inside)
    {
        if (c == ',' && depth == 0)
        {
            p_args.push_back(arg);
            arg.clear();
            continue;
        }
        depth += (c == '(') ? 1 : (c == ')') ? -1 : 0;
        if (c != ' ')
        {
            arg += c;
        }
    }
    if (!arg.empty())
    {
        p_args.push_back(arg);
    }
    return p_text.substr(0, open);
}

/// Finds a skill by name, ignoring spaces: the receptivity parser drops the
/// spaces between arguments ("Convey(p1,feeder,zone)" for the skill
/// "Convey(p1, feeder, zone)").
static SkillId findSkill(SkillScheduler const& p_skills, std::string const& p_name)
{
    SkillId const exact = p_skills.find(p_name);
    if (exact != NO_SKILL)
    {
        return exact;
    }
    auto const compact = [](std::string p_text)
    {
        std::erase(p_text, ' ');
        return p_text;
    };
    std::string const wanted = compact(p_name);
    for (SkillId id = 0U; id < p_skills.size(); ++id)
    {
        if (compact(p_skills.name(id)) == wanted)
        {
            return id;
        }
    }
    return NO_SKILL;
}

/// Finds where each input of the engine comes from.
static Result<void> bindInputs(GrafcetController::Impl& p_impl)
{
    std::string errors;
    for (auto const& [name, value] : p_impl.engine.inputs())
    {
        (void)value;
        GrafcetInput input;
        input.name = name;
        std::string const head = splitCall(name, input.args);
        if ((head == "done" || head == "failed" || head == "running") &&
            input.args.size() == 1U)
        {
            input.kind = (head == "done")     ? GrafcetInput::Kind::Done
                         : (head == "failed") ? GrafcetInput::Kind::Failed
                                              : GrafcetInput::Kind::Running;
            input.skill = findSkill(p_impl.skills, input.args.front());
            if (input.skill == NO_SKILL)
            {
                errors += "\n  " + name + ": unknown skill '" + input.args.front() + "'";
            }
        }
        else if (Observer const* observer = p_impl.domain.observer(head))
        {
            input.kind = GrafcetInput::Kind::Predicate;
            input.observer = observer;
        }
        p_impl.inputs.push_back(std::move(input));
    }
    if (!errors.empty())
    {
        return failure(p_impl.spec.file.string() + ":" + errors);
    }
    return success();
}

/// Finds the skill of each action.
static Result<void> bindActions(GrafcetController::Impl& p_impl)
{
    std::string errors;
    for (auto const& [name, value] : p_impl.engine.outputs())
    {
        (void)value;
        SkillId const skill = findSkill(p_impl.skills, name);
        if (skill == NO_SKILL)
        {
            errors += "\n  action '" + name + "' is not a skill";
        }
        p_impl.actions.push_back({ name, skill, false });
    }
    if (!errors.empty())
    {
        return failure(p_impl.spec.file.string() + ":" + errors);
    }
    return success();
}

/// Starts the engine and gives the initial operator inputs.
static Result<void> start(GrafcetController::Impl& p_impl)
{
    if (!p_impl.engine.start())
    {
        std::string errors;
        for (std::string const& error : p_impl.engine.errors())
        {
            errors += "\n  " + error;
        }
        return failure(p_impl.spec.file.string() + ":" + errors);
    }
    for (auto const& [name, value] : p_impl.spec.inputs)
    {
        p_impl.engine.setInput(name, value);
    }
    for (GrafcetAction& action : p_impl.actions)
    {
        action.previous = false;
    }
    p_impl.next_event = 0U;
    p_impl.pulses.clear();
    return success();
}

/// Compiles an end condition of the scenario.
static Result<void> compileEnd(GrafcetController::Impl& p_impl,
                               tpne::Receptivity& p_receptivity,
                               std::string const& p_code, char const* p_key)
{
    p_impl.conditions.errors.clear();
    if (!p_receptivity.compile(p_code, p_impl.conditions))
    {
        return failure(std::string("grafcet.") + p_key + ": " + p_receptivity.error());
    }
    if (!p_impl.conditions.errors.empty())
    {
        return failure(std::string("grafcet.") + p_key + ": " +
                       p_impl.conditions.errors.front());
    }
    return success();
}

Result<std::unique_ptr<GrafcetController>>
GrafcetController::load(GrafcetSpec const& p_spec, SkillScheduler& p_skills,
                        PlanningDomain const& p_domain)
{
    std::unique_ptr<GrafcetController> controller(new GrafcetController());
    controller->m_impl = std::make_unique<Impl>(p_spec, p_skills, p_domain);
    Impl& impl = *controller->m_impl;

    if (p_spec.workflow)
    {
        // Generated from the workflow: it ends with its End step unless the
        // scenario says otherwise.
        std::string end;
        ROBOTIK_TRY_ASSIGN(end, workflowGrafcet(*p_spec.workflow, impl.graphs));
        if (impl.spec.success.empty())
        {
            impl.spec.success = end;
        }
    }
    else
    {
        std::string const error = tpne::importAllNetsFromJSON(impl.graphs, p_spec.file.string());
        if (!error.empty())
        {
            return failure(error);
        }
    }
    if (impl.graphs.empty() || impl.graphs.front().type() != tpne::TypeOfNet::GRAFCET)
    {
        return failure(p_spec.file.string() + ": not a GRAFCET (\"type\": \"GRAFCET\")");
    }

    // The end conditions are read like receptivities: their inputs are
    // computed with the others.
    for (std::string const* code : { &impl.spec.success, &impl.spec.failure })
    {
        for (std::string const& name : tpne::Receptivity::inputs(*code))
        {
            impl.graphs.front().inputs.emplace(name, 0);
        }
    }

    std::vector<tpne::Net*> graphs;
    for (tpne::Net& net : impl.graphs)
    {
        graphs.push_back(&net);
    }
    impl.engine.graphs(graphs);
    impl.engine.onWarning = [&impl](std::string const& p_warning)
    { impl.warnings.push_back(p_warning); };

    ROBOTIK_TRY(start(impl));
    ROBOTIK_TRY(bindInputs(impl));
    ROBOTIK_TRY(bindActions(impl));
    ROBOTIK_TRY(compileEnd(impl, impl.success, impl.spec.success, "success"));
    ROBOTIK_TRY(compileEnd(impl, impl.failure, p_spec.failure, "failure"));
    for (auto const& [name, value] : p_spec.inputs)
    {
        (void)value;
        if (!impl.engine.inputs().contains(name))
        {
            return failure("grafcet.inputs." + name + ": no receptivity reads it");
        }
    }
    for (GrafcetSpec::Event const& event : p_spec.events)
    {
        if (!impl.engine.inputs().contains(event.input))
        {
            return failure("grafcet.events: no receptivity reads '" + event.input + "'");
        }
    }
    return controller;
}

GrafcetController::GrafcetController() = default;

GrafcetController::~GrafcetController() = default;

void GrafcetController::reset()
{
    m_impl->engine.stop();
    // The file was checked by load(): it compiles again.
    (void)start(*m_impl);
    m_impl->warnings.clear();
    std::string const& success = m_impl->spec.success;
    std::string const& failure = m_impl->spec.failure;
    (void)compileEnd(*m_impl, m_impl->success, success, "success");
    (void)compileEnd(*m_impl, m_impl->failure, failure, "failure");
}

/// Value of an input computed from the beliefs, or of an operator input.
static int valueOf(GrafcetInput const& p_input, GrafcetController::Impl const& p_impl,
                   RobotContext const& p_context)
{
    // A skill that is an action: done and failed only tell about the run the
    // GRAFCET requested, not about an older run (else the next step would see
    // the result of the previous use of the skill).
    auto const requested = [&p_impl](SkillId p_skill)
    {
        for (GrafcetAction const& action : p_impl.actions)
        {
            if (action.skill == p_skill)
            {
                return action.previous;
            }
        }
        return true;
    };

    switch (p_input.kind)
    {
        case GrafcetInput::Kind::Done:
            return (p_impl.skills.state(p_input.skill) == SkillState::Succeeded) &&
                   requested(p_input.skill);
        case GrafcetInput::Kind::Failed:
        {
            SkillState const state = p_impl.skills.state(p_input.skill);
            return (state == SkillState::Failed || state == SkillState::Cancelled) &&
                   requested(p_input.skill);
        }
        case GrafcetInput::Kind::Running:
        {
            SkillState const state = p_impl.skills.state(p_input.skill);
            return state == SkillState::Waiting || state == SkillState::Running;
        }
        case GrafcetInput::Kind::Predicate:
            return (*p_input.observer)(p_context, std::span<std::string const>(p_input.args)) ==
                   Truth::True;
        case GrafcetInput::Kind::Operator:
            break;
    }
    return p_impl.engine.inputValue(p_input.name);
}

bt::Status GrafcetController::tick(RobotContext const& p_context)
{
    Impl& impl = *m_impl;
    float const dt = static_cast<float>(p_context.dt.value());

    // 1. Inputs: events of the scenario (buttons), beliefs and skills.
    while (impl.next_event < impl.spec.events.size() &&
           impl.spec.events[impl.next_event].at.value() <= p_context.time.value())
    {
        GrafcetSpec::Event const& event = impl.spec.events[impl.next_event++];
        impl.engine.setInput(event.input, event.pulse ? 1 : event.value);
        if (event.pulse)
        {
            impl.pulses.push_back(event.input);
        }
    }
    for (GrafcetInput const& input : impl.inputs)
    {
        if (input.kind != GrafcetInput::Kind::Operator)
        {
            impl.engine.setInput(input.name, valueOf(input, impl, p_context));
        }
    }

    // 2. Evolution of the graphs, then actions. Push buttons are released
    // after one cycle: their rising edge has been seen.
    impl.engine.step(dt);
    for (std::string const& name : impl.pulses)
    {
        impl.engine.setInput(name, 0);
    }
    impl.pulses.clear();

    // 3. Actions to skills: request on the rising edge, cancel on the falling
    // edge if the skill still waits or runs.
    for (GrafcetAction& action : impl.actions)
    {
        bool const output = impl.engine.output(action.name);
        if (output && !action.previous)
        {
            impl.skills.request(action.skill);
        }
        else if (!output && action.previous)
        {
            impl.skills.cancel(action.skill);
        }
        action.previous = output;
    }

    // 4. End of the run, as the scenario says.
    for (auto const& [name, value] : impl.engine.inputs())
    {
        auto const found = impl.conditions.values.find(name);
        if (found != impl.conditions.values.end())
        {
            found->second = value;
        }
    }
    impl.success.tick(dt);
    impl.failure.tick(dt);
    bool const failed = !impl.spec.failure.empty() && impl.failure.evaluate();
    bool const succeeded = !impl.spec.success.empty() && impl.success.evaluate();
    impl.success.endOfInstant();
    impl.failure.endOfInstant();
    impl.success.endOfCycle();
    impl.failure.endOfCycle();
    if (failed)
    {
        return bt::Status::FAILURE;
    }
    return succeeded ? bt::Status::SUCCESS : bt::Status::RUNNING;
}

bool GrafcetController::setInput(std::string const& p_name, int p_value)
{
    for (GrafcetInput const& input : m_impl->inputs)
    {
        if (input.name == p_name && input.kind == GrafcetInput::Kind::Operator)
        {
            m_impl->engine.setInput(p_name, p_value);
            return true;
        }
    }
    return false;
}

bool GrafcetController::pulse(std::string const& p_name)
{
    if (!setInput(p_name, 1))
    {
        return false;
    }
    m_impl->pulses.push_back(p_name);
    return true;
}

std::map<std::string, int> GrafcetController::operatorInputs() const
{
    std::map<std::string, int> inputs;
    for (GrafcetInput const& input : m_impl->inputs)
    {
        if (input.kind == GrafcetInput::Kind::Operator)
        {
            inputs[input.name] = m_impl->engine.inputValue(input.name);
        }
    }
    return inputs;
}

std::vector<std::string> GrafcetController::activeSteps() const
{
    std::vector<std::string> steps;
    for (tpne::Net const& net : m_impl->graphs)
    {
        for (tpne::Place const& place : net.places())
        {
            if (place.tokens > 0U)
            {
                steps.push_back(net.name + ":X" + std::to_string(place.id));
            }
        }
    }
    return steps;
}

std::size_t GrafcetController::firings() const
{
    return m_impl->engine.firingCount();
}

std::vector<tpne::Net> const& GrafcetController::graphs() const
{
    return m_impl->graphs;
}

tpne::GrafcetEngine const& GrafcetController::engine() const
{
    return m_impl->engine;
}

bool GrafcetController::generated() const
{
    return m_impl->spec.workflow.has_value();
}

Result<void> GrafcetController::save(std::filesystem::path const& p_file) const
{
    // The engine knows the graphs by their address: the initial marking is
    // read on the originals and given to the copies.
    std::vector<tpne::Net> graphs = m_impl->graphs;
    for (std::size_t i = 0U; i < graphs.size(); ++i)
    {
        (void)graphs[i].tokens(m_impl->engine.initialMarking(m_impl->graphs[i]));
    }
    std::string const error = tpne::exportAllNetsToJSON(graphs, p_file.string());
    if (!error.empty())
    {
        return failure(p_file.string() + ": " + error);
    }
    return success();
}

} // namespace robotik
