// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/Scenario/Scenario.hpp"
#include "Robotik/Common/Must.hpp"

#include "Robotik/Package/Scenario/FileRole.hpp"
#include "Robotik/Package/Scenario/PackagePath.hpp"
#include "Robotik/Package/Scenario/Workflow.hpp"

#include <algorithm>
#include <stdexcept>

namespace robotik
{

/// Reads the faults a scenario injects on resources.
static void readFaults(Scenario& p_scenario, ConfigNode const& p_node)
{
    for (ConfigNode const& fault : p_node.items())
    {
        std::string resource = fault.text("resource");
        if (resource.empty())
        {
            throw std::runtime_error("a fault has no resource");
        }
        if (fault.has("rate"))
        {
            p_scenario.random_faults.push_back(
                { std::move(resource), fault.number("rate", 0.0) });
            continue;
        }
        std::string const action = fault.text("action");
        if (action != "disable" && action != "restore")
        {
            throw std::runtime_error("fault on '" + resource +
                                     "': action must be disable or restore");
        }
        p_scenario.faults.push_back({ Seconds(fault.number("at", 0.0)),
                                      std::move(resource),
                                      action == "disable" });
    }
}

/// Reads planner literals such as in(a, b).
static std::vector<Literal> readLiterals(ConfigNode const& p_node,
                                         std::string const& p_key)
{
    std::vector<Literal> literals;
    for (ConfigNode const& item : p_node.at(p_key).items())
    {
        if (!item.isScalar())
        {
            throw std::runtime_error(
                "planner." + p_key +
                ": each item is a literal such as in(a, b)");
        }
        try
        {
            literals.push_back(must(parseLiteral(item.scalar())));
        }
        catch (std::runtime_error const& error)
        {
            throw std::runtime_error(
                "planner." + p_key + ": " + error.what() +
                " (write one literal per line: a flow list [a(x, y)] "
                "splits at the comma)");
        }
    }
    return literals;
}

/// Reads the planner block: domain file, goal and replan limits.
static PlannerSpec readPlanner(std::filesystem::path const& p_root,
                               ConfigNode const& p_node)
{
    PlannerSpec spec;
    spec.domain = must(packageFile(p_root, p_node.text("domain")));
    if (spec.domain.empty())
    {
        throw std::runtime_error(
            "planner.domain is required (a PDDL file of the package)");
    }
    spec.facts = readLiterals(p_node, "facts");
    spec.goal = readLiterals(p_node, "goal");
    spec.goal_pddl = p_node.text("goal_pddl");
    if (spec.goal.empty() == spec.goal_pddl.empty())
    {
        throw std::runtime_error(
            "planner: give a goal (list of literals) or a goal_pddl");
    }
    ConfigNode const& replan = p_node.at("replan");
    double const max = replan.number("max", spec.max_replans);
    if (max < 0.0)
    {
        throw std::runtime_error("planner.replan.max must be positive");
    }
    spec.max_replans = static_cast<std::uint32_t>(max);
    spec.replan_delay =
        Seconds(replan.number("delay", spec.replan_delay.value()));
    if (p_node.text("verify") == "observed")
    {
        spec.verify = true;
        spec.observed = true;
    }
    else
    {
        spec.verify = p_node.flag("verify", spec.verify);
    }
    spec.parallel = p_node.flag("parallel", spec.parallel);
    spec.search = p_node.text("search", spec.search);
    spec.timeout = Seconds(p_node.number("timeout", spec.timeout.value()));
    return spec;
}

/// Reads the grafcet block: a file name, or a map with the file (or the
/// workflow to generate it from), the end conditions and the initial
/// operator inputs.
static GrafcetSpec readGrafcet(std::filesystem::path const& p_root,
                               ConfigNode const& p_node)
{
    GrafcetSpec spec;
    if (p_node.isMap() && p_node.has("workflow"))
    {
        if (p_node.has("file"))
        {
            throw std::runtime_error("grafcet: a file or a workflow, not both");
        }
        spec.file = must(packageFile(p_root, p_node.text("workflow")));
        if (fileRole(spec.file) != FileRole::Workflow)
        {
            throw std::runtime_error("grafcet.workflow: '" + spec.file.filename().string() +
                                     "' is not a .workflow.yaml file");
        }
        spec.workflow = must(loadWorkflow(spec.file));
    }
    else
    {
        spec.file = must(packageFile(
            p_root, p_node.isScalar() ? p_node.scalar() : p_node.text("file")));
        if (spec.file.empty())
        {
            throw std::runtime_error(
                "grafcet.file is required (a .grafcet.json file of the package)");
        }
        if (fileRole(spec.file) != FileRole::Grafcet)
        {
            throw std::runtime_error("grafcet: '" + spec.file.filename().string() +
                                     "' is not a .grafcet.json file");
        }
    }
    if (p_node.isMap())
    {
        spec.success = p_node.text("success");
        spec.failure = p_node.text("failure");
        for (auto const& [name, value] : p_node.at("inputs").entries())
        {
            if (!value.isScalar())
            {
                throw std::runtime_error("grafcet.inputs." + name +
                                         " must be a number");
            }
            spec.inputs[name] =
                static_cast<int>(p_node.at("inputs").number(name, 0.0));
        }
        for (ConfigNode const& item : p_node.at("events").items())
        {
            GrafcetSpec::Event event;
            event.at = Seconds(item.number("at", -1.0));
            event.input = item.text("input");
            event.value = static_cast<int>(item.number("value", 1.0));
            event.pulse = item.flag("pulse", false);
            if (event.at.value() < 0.0 || event.input.empty())
            {
                throw std::runtime_error(
                    "grafcet.events: each event needs 'at' (s) and 'input'");
            }
            spec.events.push_back(event);
        }
        std::ranges::stable_sort(spec.events, {}, [](GrafcetSpec::Event const& p_event)
                                 { return p_event.at.value(); });
    }
    return spec;
}

Result<ScenarioSource> loadScenarioSource(std::filesystem::path const& p_scenario)
{
    // The border of this reader: whatever stops it inside becomes a message.
    try
    {
        ScenarioSource source;
        source.scenario_file =
            std::filesystem::absolute(p_scenario).lexically_normal();
        source.package = must(packageRoot(source.scenario_file));
        source.scenario = must(ConfigNode::load(source.scenario_file));
        ConfigNode const& scenario = source.scenario.at("scenario");
        if (!scenario.isMap())
        {
            throw std::runtime_error(p_scenario.string() +
                                     ": no 'scenario:' section");
        }
        std::string const world = scenario.text("world");
        if (world.empty())
        {
            throw std::runtime_error(p_scenario.string() +
                                     ": scenario.world is required");
        }
        try
        {
            source.world_file = must(packageFile(source.package, world));
        }
        catch (std::runtime_error const& error)
        {
            throw std::runtime_error(p_scenario.string() + ": scenario.world " +
                                     error.what());
        }
        source.world = must(ConfigNode::load(source.world_file));
        return source;
    }
    catch (std::exception const& error)
    {
        return failure(error.what());
    }
}

Result<void> overrideValue(ScenarioSource& p_source, std::string_view p_path, ConfigNode p_value)
{
    if (p_path.starts_with("world."))
    {
        return p_source.world.set(p_path, std::move(p_value));
    }
    if (p_path.starts_with("scenario."))
    {
        return p_source.scenario.set(p_path, std::move(p_value));
    }
    return failure("Override '" + std::string(p_path) + "' must start with scenario. or world.");
}

/// Maps the words abstract and rigid onto the fidelity enum.
static Fidelity readFidelity(std::string const& p_text,
                             std::string const& p_where)
{
    if (p_text == "abstract")
    {
        return Fidelity::Abstract;
    }
    if (p_text == "rigid")
    {
        return Fidelity::Rigid;
    }
    if (p_text == "contact")
    {
        throw std::runtime_error(
            p_where + ": contact is not available yet (abstract or rigid)");
    }
    throw std::runtime_error(p_where + " must be abstract, rigid or contact");
}

/// Reads the fidelity and the per-entity overrides under physics.
static void readPhysics(Scenario& p_scenario, ConfigNode const& p_node)
{
    if (!p_node.isMap())
    {
        return;
    }
    p_scenario.physics.fidelity =
        readFidelity(p_node.text("fidelity", "abstract"), "physics.fidelity");
    for (auto const& [id, level] : p_node.at("overrides").entries())
    {
        if (!level.isScalar())
        {
            throw std::runtime_error("physics.overrides." + id +
                                     " must be a level");
        }
        p_scenario.physics.overrides.emplace(
            id, readFidelity(level.scalar(), "physics.overrides." + id));
    }
}

/// Every override names an entity or an actuator of the world.
static void checkPhysics(Scenario const& p_scenario)
{
    for (auto const& [id, level] : p_scenario.physics.overrides)
    {
        bool known = findEntity(p_scenario.world, id) != nullptr;
        for (EntitySpec const& entity : p_scenario.world.entities)
        {
            for (ActuatorSpec const& actuator : entity.actuators)
            {
                known = known || actuator.name == id;
            }
        }
        if (!known)
        {
            throw std::runtime_error(
                "physics.overrides: '" + id +
                "' is no entity nor actuator of the world");
        }
    }
}


// ----------------------------------------------------------------------------
/// Reads a perception level name.
/// @param p_text "sensed", "noisy_truth" or "truth".
/// @param p_where Field, for the error message.
/// @return The level.
// ----------------------------------------------------------------------------
static PerceptionFidelity readPerceptionLevel(std::string const& p_text,
                                              std::string const& p_where)
{
    if (p_text == "sensed")
    {
        return PerceptionFidelity::Sensed;
    }
    if (p_text == "noisy_truth")
    {
        return PerceptionFidelity::NoisyTruth;
    }
    if (p_text == "truth")
    {
        return PerceptionFidelity::Truth;
    }
    throw std::runtime_error(p_where + " must be sensed, noisy_truth or truth, not '" +
                             p_text + "'");
}

// ----------------------------------------------------------------------------
/// Reads the @c perception: section. The older @c oracle: key still works:
/// @c always is @c fidelity: truth, @c when_blind is @c blind: truth.
/// @param p_scenario Scenario to fill.
/// @param p_node The @c perception: node (may be null).
// ----------------------------------------------------------------------------
static void readPerception(Scenario& p_scenario, ConfigNode const& p_node)
{
    PerceptionSpec& perception = p_scenario.perception;
    if (p_node.has("oracle"))
    {
        std::string const oracle = p_node.text("oracle");
        if (oracle == "always")
        {
            perception.fidelity = PerceptionFidelity::Truth;
        }
        else if (oracle == "when_blind")
        {
            perception.blind = PerceptionFidelity::Truth;
        }
        else if (oracle != "never")
        {
            throw std::runtime_error("perception.oracle must be never, when_blind or always");
        }
    }
    if (p_node.has("fidelity"))
    {
        perception.fidelity = readPerceptionLevel(p_node.text("fidelity"), "perception.fidelity");
    }
    if (p_node.has("blind"))
    {
        perception.blind = readPerceptionLevel(p_node.text("blind"), "perception.blind");
    }
    for (auto const& [id, level] : p_node.at("overrides").entries())
    {
        perception.overrides.emplace(
            id, readPerceptionLevel(level.scalar(), "perception.overrides." + id));
    }
    ConfigNode const& noise = p_node.at("noise");
    perception.noise.sigma = noise.number("sigma", perception.noise.sigma);
    perception.noise.dropout = noise.number("dropout", perception.noise.dropout);
    perception.noise.range = noise.number("range", perception.noise.range);
    perception.max_age = Seconds(p_node.number("max_age", perception.max_age.value()));
}

Result<Scenario> readScenario(ScenarioSource const& p_source, AssetResolver const& p_assets)
{
    // The border of this reader: whatever stops it inside becomes a message.
    try
    {
        ConfigNode const& node = p_source.scenario.at("scenario");
        std::filesystem::path const& root = p_source.package;
        std::string const where = p_source.scenario_file.string();

        Scenario scenario;
        scenario.assets = p_assets;
        scenario.file = p_source.scenario_file;
        try
        {
            scenario.id = node.text("id");
            if (scenario.id.empty())
            {
                throw std::runtime_error("scenario.id is required");
            }
            scenario.description = node.text("description");
            scenario.seed = static_cast<std::uint64_t>(node.number("seed", 0.0));
            ConfigNode const& controller = node.at("controller");
            scenario.controller.behavior_tree =
                must(packageFile(root, controller.text("behavior_tree")));
            if (controller.has("planner"))
            {
                scenario.controller.planner =
                    readPlanner(root, controller.at("planner"));
            }
            if (controller.has("grafcet"))
            {
                scenario.controller.grafcet =
                    readGrafcet(root, controller.at("grafcet"));
            }
            int const controllers =
                int(!scenario.controller.behavior_tree.empty()) +
                int(scenario.controller.planner.has_value()) +
                int(scenario.controller.grafcet.has_value());
            if (controllers > 1)
            {
                throw std::runtime_error("controller: give one of behavior_tree, "
                                         "planner or grafcet, not several");
            }
            for (auto const& [name, file] : node.at("trajectories").entries())
            {
                if (!file.isScalar())
                {
                    throw std::runtime_error("trajectories." + name +
                                             " must be a file name");
                }
                scenario.trajectories.emplace(
                    name, must(loadTrajectory(must(packageFile(root, file.scalar())))));
            }
            readFaults(scenario, node.at("faults"));
            readPerception(scenario, node.at("perception"));
            readPhysics(scenario, node.at("physics"));
            scenario.timeout = Seconds(node.number("timeout", 0.0));
            scenario.stop_when = node.text("stop_when");
            scenario.parameters = node.at("parameters");
            for (ConfigNode const& item : node.at("assertions").items())
            {
                scenario.assertions.push_back(must(readAssertion(item)));
            }
        }
        catch (std::runtime_error const& error)
        {
            throw std::runtime_error(where + ": " + error.what());
        }

        ConfigNode const& world = p_source.world.at("world");
        if (!world.isMap())
        {
            throw std::runtime_error(p_source.world_file.string() +
                                     ": no 'world:' section");
        }
        try
        {
            scenario.world = must(readWorld(world, root, p_assets));
        }
        catch (std::runtime_error const& error)
        {
            throw std::runtime_error(p_source.world_file.string() + ": " +
                                     error.what());
        }
        try
        {
            checkPhysics(scenario);
        }
        catch (std::runtime_error const& error)
        {
            throw std::runtime_error(where + ": " + error.what());
        }
        return scenario;
    }
    catch (std::exception const& error)
    {
        return failure(error.what());
    }
}

} // namespace robotik
