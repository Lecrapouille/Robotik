// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Scenario/Scenario.hpp"

#include "Robotik/Scenario/PackagePath.hpp"

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
        p_scenario.faults.push_back(
            { Seconds(fault.number("at", 0.0)), std::move(resource), action == "disable" });
    }
}

/// Reads planner literals such as in(a, b).
static std::vector<Literal> readLiterals(ConfigNode const& p_node, std::string const& p_key)
{
    std::vector<Literal> literals;
    for (ConfigNode const& item : p_node.at(p_key).items())
    {
        if (!item.isScalar())
        {
            throw std::runtime_error("planner." + p_key + ": each item is a literal such as in(a, b)");
        }
        try
        {
            literals.push_back(Literal::parse(item.scalar()));
        }
        catch (std::runtime_error const& error)
        {
            throw std::runtime_error("planner." + p_key + ": " + error.what() +
                                     " (write one literal per line: a flow list [a(x, y)] "
                                     "splits at the comma)");
        }
    }
    return literals;
}

/// Reads the planner block: domain file, goal and replan limits.
static PlannerSpec readPlanner(std::filesystem::path const& p_root, ConfigNode const& p_node)
{
    PlannerSpec spec;
    spec.domain = packageFile(p_root, p_node.text("domain"));
    if (spec.domain.empty())
    {
        throw std::runtime_error("planner.domain is required (a PDDL file of the package)");
    }
    spec.facts = readLiterals(p_node, "facts");
    spec.goal = readLiterals(p_node, "goal");
    spec.goal_pddl = p_node.text("goal_pddl");
    if (spec.goal.empty() == spec.goal_pddl.empty())
    {
        throw std::runtime_error("planner: give a goal (list of literals) or a goal_pddl");
    }
    ConfigNode const& replan = p_node.at("replan");
    double const max = replan.number("max", spec.max_replans);
    if (max < 0.0)
    {
        throw std::runtime_error("planner.replan.max must be positive");
    }
    spec.max_replans = static_cast<std::uint32_t>(max);
    spec.replan_delay = Seconds(replan.number("delay", spec.replan_delay.value()));
    spec.verify = p_node.flag("verify", spec.verify);
    spec.parallel = p_node.flag("parallel", spec.parallel);
    spec.search = p_node.text("search", spec.search);
    spec.timeout = Seconds(p_node.number("timeout", spec.timeout.value()));
    return spec;
}

ScenarioSource ScenarioSource::load(std::filesystem::path const& p_scenario)
{
    ScenarioSource source;
    source.scenario_file = std::filesystem::absolute(p_scenario).lexically_normal();
    source.package = packageRoot(source.scenario_file);
    source.scenario = ConfigNode::load(source.scenario_file);
    ConfigNode const& scenario = source.scenario.at("scenario");
    if (!scenario.isMap())
    {
        throw std::runtime_error(p_scenario.string() + ": no 'scenario:' section");
    }
    std::string const world = scenario.text("world");
    if (world.empty())
    {
        throw std::runtime_error(p_scenario.string() + ": scenario.world is required");
    }
    try
    {
        source.world_file = packageFile(source.package, world);
    }
    catch (std::runtime_error const& error)
    {
        throw std::runtime_error(p_scenario.string() + ": scenario.world " + error.what());
    }
    source.world = ConfigNode::load(source.world_file);
    return source;
}

void ScenarioSource::override(std::string_view p_path, ConfigNode p_value)
{
    if (p_path.starts_with("world."))
    {
        world.set(p_path, std::move(p_value));
    }
    else if (p_path.starts_with("scenario."))
    {
        scenario.set(p_path, std::move(p_value));
    }
    else
    {
        throw std::runtime_error("Override '" + std::string(p_path) +
                                 "' must start with scenario. or world.");
    }
}

/// Maps the words abstract and rigid onto the fidelity enum.
static Fidelity readFidelity(std::string const& p_text, std::string const& p_where)
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
        throw std::runtime_error(p_where + ": contact is not available yet (abstract or rigid)");
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
    p_scenario.physics.fidelity = readFidelity(p_node.text("fidelity", "abstract"), "physics.fidelity");
    for (auto const& [id, level] : p_node.at("overrides").entries())
    {
        if (!level.isScalar())
        {
            throw std::runtime_error("physics.overrides." + id + " must be a level");
        }
        p_scenario.physics.overrides.emplace(id, readFidelity(level.scalar(), "physics.overrides." + id));
    }
}

/// Every override names an entity or an actuator of the world.
static void checkPhysics(Scenario const& p_scenario)
{
    for (auto const& [id, level] : p_scenario.physics.overrides)
    {
        bool known = p_scenario.world.find(id) != nullptr;
        for (EntitySpec const& entity : p_scenario.world.entities)
        {
            for (ActuatorSpec const& actuator : entity.actuators)
            {
                known = known || actuator.name == id;
            }
        }
        if (!known)
        {
            throw std::runtime_error("physics.overrides: '" + id +
                                     "' is no entity nor actuator of the world");
        }
    }
}

char const* toString(Fidelity p_fidelity)
{
    switch (p_fidelity)
    {
        case Fidelity::Abstract:
            return "abstract";
        case Fidelity::Rigid:
            return "rigid";
        case Fidelity::Contact:
            return "contact";
    }
    return "?";
}

Scenario Scenario::read(ScenarioSource const& p_source, AssetResolver const& p_assets)
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
        scenario.controller.behavior_tree = packageFile(root, controller.text("behavior_tree"));
        if (controller.has("planner"))
        {
            if (!scenario.controller.behavior_tree.empty())
            {
                throw std::runtime_error("controller: give a behavior_tree or a planner, not both");
            }
            scenario.controller.planner = readPlanner(root, controller.at("planner"));
        }
        for (auto const& [name, file] : node.at("trajectories").entries())
        {
            if (!file.isScalar())
            {
                throw std::runtime_error("trajectories." + name + " must be a file name");
            }
            scenario.trajectories.emplace(name, Trajectory::load(packageFile(root, file.scalar())));
        }
        readFaults(scenario, node.at("faults"));
        std::string const oracle = node.at("perception").text("oracle", "never");
        if (oracle == "never")
        {
            scenario.oracle = OraclePolicy::Never;
        }
        else if (oracle == "when_blind")
        {
            scenario.oracle = OraclePolicy::WhenBlind;
        }
        else if (oracle == "always")
        {
            scenario.oracle = OraclePolicy::Always;
        }
        else
        {
            throw std::runtime_error("perception.oracle must be never, when_blind or always");
        }
        readPhysics(scenario, node.at("physics"));
        scenario.timeout = Seconds(node.number("timeout", 0.0));
        scenario.stop_when = node.text("stop_when");
        scenario.parameters = node.at("parameters");
        for (ConfigNode const& item : node.at("assertions").items())
        {
            scenario.assertions.push_back(AssertionSpec::read(item));
        }
    }
    catch (std::runtime_error const& error)
    {
        throw std::runtime_error(where + ": " + error.what());
    }

    ConfigNode const& world = p_source.world.at("world");
    if (!world.isMap())
    {
        throw std::runtime_error(p_source.world_file.string() + ": no 'world:' section");
    }
    try
    {
        scenario.world =
            WorldDescription::read(world, root, p_assets);
    }
    catch (std::runtime_error const& error)
    {
        throw std::runtime_error(p_source.world_file.string() + ": " + error.what());
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

} // namespace robotik
