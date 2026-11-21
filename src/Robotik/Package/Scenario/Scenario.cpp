// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/Scenario/Scenario.hpp"

#include "Robotik/Package/Scenario/PackagePath.hpp"

namespace robotik
{

/// @p_result, its error prefixed with @p_where.
template <typename T>
static Result<T> prefixed(std::string const& p_where, Result<T> p_result)
{
    if (!p_result)
    {
        return failure(p_where + ": " + p_result.error());
    }
    return p_result;
}

/// Reads the faults a scenario injects on resources.
static Result<void> readFaults(Scenario& p_scenario, ConfigNode const& p_node)
{
    for (ConfigNode const& fault : p_node.items())
    {
        ROBOTIK_TRY(
            fault.checkKeys({ "resource", "rate", "action", "at" }, "fault"));
        std::string resource = fault.text("resource");
        if (resource.empty())
        {
            return failure("a fault has no resource");
        }
        if (fault.has("rate"))
        {
            double rate = 0.0;
            ROBOTIK_TRY_ASSIGN(rate, fault.number("rate", 0.0));
            p_scenario.random_faults.push_back({ std::move(resource), rate });
            continue;
        }
        std::string const action = fault.text("action");
        if (action != "disable" && action != "restore")
        {
            return failure("fault on '" + resource +
                           "': action must be disable or restore");
        }
        double at = 0.0;
        ROBOTIK_TRY_ASSIGN(at, fault.number("at", 0.0));
        p_scenario.faults.push_back(
            { Seconds(at), std::move(resource), action == "disable" });
    }
    return success();
}

/// Reads planner literals such as (in a b).
static Result<std::vector<Literal>> readLiterals(ConfigNode const& p_node,
                                                 std::string const& p_key)
{
    std::vector<Literal> literals;
    std::string const where = "planner." + p_key;
    for (ConfigNode const& item : p_node.at(p_key).items())
    {
        if (!item.isScalar())
        {
            return failure(where + ": each item is a literal such as (in a b)");
        }
        Literal literal;
        ROBOTIK_TRY_ASSIGN(literal,
                           prefixed(where, parseLiteral(item.scalar())));
        literals.push_back(std::move(literal));
    }
    return literals;
}

/// Reads the planner block: domain file, goal and replan limits.
static Result<PlannerSpec> readPlanner(std::filesystem::path const& p_root,
                                       ConfigNode const& p_node)
{
    ROBOTIK_TRY(p_node.checkKeys({ "domain",
                                   "method",
                                   "facts",
                                   "goal",
                                   "goal_pddl",
                                   "replan",
                                   "verify",
                                   "parallel",
                                   "search",
                                   "timeout" },
                                 "planner"));
    PlannerSpec spec;
    ROBOTIK_TRY_ASSIGN(spec.domain, packageFile(p_root, p_node.text("domain")));
    if (spec.domain.empty())
    {
        return failure(
            "planner.domain is required (a PDDL file of the package)");
    }
    if (std::string const method = p_node.text("method", "search");
        method == "pabt")
    {
        spec.method = PlannerMethod::Backchaining;
    }
    else if (method != "search")
    {
        return failure("planner.method: '" + method +
                       "' is not search or pabt");
    }
    ROBOTIK_TRY_ASSIGN(spec.facts, readLiterals(p_node, "facts"));
    ROBOTIK_TRY_ASSIGN(spec.goal, readLiterals(p_node, "goal"));
    spec.goal_pddl = p_node.text("goal_pddl");
    if (spec.goal.empty() == spec.goal_pddl.empty())
    {
        return failure(
            "planner: give a goal (list of literals) or a goal_pddl");
    }
    ConfigNode const& replan = p_node.at("replan");
    ROBOTIK_TRY(replan.checkKeys({ "max", "delay" }, "planner.replan"));
    double max = 0.0;
    ROBOTIK_TRY_ASSIGN(max, replan.number("max", spec.max_replans));
    if (max < 0.0)
    {
        return failure("planner.replan.max must be positive");
    }
    spec.max_replans = static_cast<std::uint32_t>(max);
    double delay = 0.0;
    ROBOTIK_TRY_ASSIGN(delay,
                       replan.number("delay", spec.replan_delay.value()));
    spec.replan_delay = Seconds(delay);
    if (p_node.text("verify") == "observed")
    {
        spec.verify = true;
        spec.observed = true;
    }
    else
    {
        ROBOTIK_TRY_ASSIGN(spec.verify, p_node.flag("verify", spec.verify));
    }
    ROBOTIK_TRY_ASSIGN(spec.parallel, p_node.flag("parallel", spec.parallel));
    spec.search = p_node.text("search", spec.search);
    double timeout = 0.0;
    ROBOTIK_TRY_ASSIGN(timeout, p_node.number("timeout", spec.timeout.value()));
    spec.timeout = Seconds(timeout);
    return spec;
}

Result<ScenarioSource> loadScenarioSource(std::filesystem::path const& p_scenario)
{
    ScenarioSource source;
    source.scenario_file =
        std::filesystem::absolute(p_scenario).lexically_normal();
    ROBOTIK_TRY_ASSIGN(source.package, packageRoot(source.scenario_file));
    ROBOTIK_TRY_ASSIGN(source.scenario, ConfigNode::load(source.scenario_file));
    ROBOTIK_TRY(source.scenario.checkKeys({ "scenario" }, p_scenario.string()));
    ConfigNode const& scenario = source.scenario.at("scenario");
    if (!scenario.isMap())
    {
        return failure(p_scenario.string() + ": no 'scenario:' section");
    }
    std::string const world = scenario.text("world");
    if (world.empty())
    {
        return failure(p_scenario.string() + ": scenario.world is required");
    }
    if (auto file = packageFile(source.package, world); file)
    {
        source.world_file = file.take();
    }
    else
    {
        return failure(p_scenario.string() + ": scenario.world " +
                       file.error());
    }
    ROBOTIK_TRY_ASSIGN(source.world, ConfigNode::load(source.world_file));
    ROBOTIK_TRY(
        source.world.checkKeys({ "world" }, source.world_file.string()));
    return source;
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
static Result<Fidelity> readFidelity(std::string const& p_text,
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
        return failure(p_where +
                       ": contact is not available yet (abstract or rigid)");
    }
    return failure(p_where + " must be abstract, rigid or contact");
}

/// Reads the fidelity and the per-entity overrides under physics.
static Result<void> readPhysics(Scenario& p_scenario, ConfigNode const& p_node)
{
    if (!p_node.isMap())
    {
        return success();
    }
    ROBOTIK_TRY(p_node.checkKeys({ "fidelity", "overrides" }, "physics"));
    ROBOTIK_TRY_ASSIGN(
        p_scenario.physics.fidelity,
        readFidelity(p_node.text("fidelity", "abstract"), "physics.fidelity"));
    for (auto const& [id, level] : p_node.at("overrides").entries())
    {
        if (!level.isScalar())
        {
            return failure("physics.overrides." + id + " must be a level");
        }
        Fidelity fidelity = Fidelity::Abstract;
        ROBOTIK_TRY_ASSIGN(
            fidelity, readFidelity(level.scalar(), "physics.overrides." + id));
        p_scenario.physics.overrides.emplace(id, fidelity);
    }
    return success();
}

/// Every override names an entity or an actuator of the world.
static Result<void> checkPhysics(Scenario const& p_scenario)
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
            return failure("physics.overrides: '" + id +
                           "' is no entity nor actuator of the world");
        }
    }
    return success();
}

// ----------------------------------------------------------------------------
/// Reads a perception level name.
/// @param p_text "sensed", "noisy_truth" or "truth".
/// @param p_where Field, for the error message.
/// @return The level, or why the word is not one.
// ----------------------------------------------------------------------------
static Result<PerceptionFidelity>
readPerceptionLevel(std::string const& p_text, std::string const& p_where)
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
    return failure(p_where + " must be sensed, noisy_truth or truth, not '" +
                   p_text + "'");
}

// ----------------------------------------------------------------------------
/// Reads the @c perception: section. The older @c oracle: key still works:
/// @c always is @c fidelity: truth, @c when_blind is @c blind: truth.
/// @param p_scenario Scenario to fill.
/// @param p_node The @c perception: node (may be null).
// ----------------------------------------------------------------------------
static Result<void> readPerception(Scenario& p_scenario,
                                   ConfigNode const& p_node)
{
    ROBOTIK_TRY(p_node.checkKeys(
        { "oracle", "fidelity", "blind", "overrides", "noise", "max_age" },
        "perception"));
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
            return failure(
                "perception.oracle must be never, when_blind or always");
        }
    }
    if (p_node.has("fidelity"))
    {
        ROBOTIK_TRY_ASSIGN(perception.fidelity,
                           readPerceptionLevel(p_node.text("fidelity"),
                                               "perception.fidelity"));
    }
    if (p_node.has("blind"))
    {
        ROBOTIK_TRY_ASSIGN(
            perception.blind,
            readPerceptionLevel(p_node.text("blind"), "perception.blind"));
    }
    for (auto const& [id, level] : p_node.at("overrides").entries())
    {
        PerceptionFidelity fidelity = PerceptionFidelity::Sensed;
        ROBOTIK_TRY_ASSIGN(
            fidelity,
            readPerceptionLevel(level.scalar(), "perception.overrides." + id));
        perception.overrides.emplace(id, fidelity);
    }
    ConfigNode const& noise = p_node.at("noise");
    ROBOTIK_TRY(
        noise.checkKeys({ "sigma", "dropout", "range" }, "perception.noise"));
    ROBOTIK_TRY_ASSIGN(perception.noise.sigma,
                       noise.number("sigma", perception.noise.sigma));
    ROBOTIK_TRY_ASSIGN(perception.noise.dropout,
                       noise.number("dropout", perception.noise.dropout));
    ROBOTIK_TRY_ASSIGN(perception.noise.range,
                       noise.number("range", perception.noise.range));
    double max_age = 0.0;
    ROBOTIK_TRY_ASSIGN(max_age,
                       p_node.number("max_age", perception.max_age.value()));
    perception.max_age = Seconds(max_age);
    return success();
}

/// Reads the scenario section itself (not its world).
static Result<void> readScenarioSection(Scenario& p_scenario,
                                        ConfigNode const& p_node,
                                        std::filesystem::path const& p_root)
{
    Scenario& scenario = p_scenario;
    ConfigNode const& node = p_node;
    ROBOTIK_TRY(node.checkKeys({ "id",
                                 "description",
                                 "seed",
                                 "world",
                                 "controller",
                                 "trajectories",
                                 "faults",
                                 "perception",
                                 "physics",
                                 "timeout",
                                 "stop_when",
                                 "parameters",
                                 "assertions" },
                               "scenario"));
    scenario.id = node.text("id");
    if (scenario.id.empty())
    {
        return failure("scenario.id is required");
    }
    scenario.description = node.text("description");
    double seed = 0.0;
    ROBOTIK_TRY_ASSIGN(seed, node.number("seed", 0.0));
    scenario.seed = static_cast<std::uint64_t>(seed);
    ConfigNode const& controller = node.at("controller");
    ROBOTIK_TRY(
        controller.checkKeys({ "behavior_tree", "planner" }, "controller"));
    ROBOTIK_TRY_ASSIGN(scenario.controller.behavior_tree,
                       packageFile(p_root, controller.text("behavior_tree")));
    if (controller.has("planner"))
    {
        if (!scenario.controller.behavior_tree.empty())
        {
            return failure(
                "controller: give a behavior_tree or a planner, not both");
        }
        ROBOTIK_TRY_ASSIGN(scenario.controller.planner,
                           readPlanner(p_root, controller.at("planner")));
    }
    for (auto const& [name, file] : node.at("trajectories").entries())
    {
        if (!file.isScalar())
        {
            return failure("trajectories." + name + " must be a file name");
        }
        std::filesystem::path path;
        ROBOTIK_TRY_ASSIGN(path, packageFile(p_root, file.scalar()));
        Trajectory trajectory;
        ROBOTIK_TRY_ASSIGN(trajectory, loadTrajectory(path));
        scenario.trajectories.emplace(name, std::move(trajectory));
    }
    ROBOTIK_TRY(readFaults(scenario, node.at("faults")));
    ROBOTIK_TRY(readPerception(scenario, node.at("perception")));
    ROBOTIK_TRY(readPhysics(scenario, node.at("physics")));
    double timeout = 0.0;
    ROBOTIK_TRY_ASSIGN(timeout, node.number("timeout", 0.0));
    scenario.timeout = Seconds(timeout);
    scenario.stop_when = node.text("stop_when");
    scenario.parameters = node.at("parameters");
    for (ConfigNode const& item : node.at("assertions").items())
    {
        AssertionSpec assertion;
        ROBOTIK_TRY_ASSIGN(assertion, readAssertion(item));
        scenario.assertions.push_back(std::move(assertion));
    }
    return success();
}

Result<Scenario> readScenario(ScenarioSource const& p_source,
                              AssetResolver const& p_assets)
{
    std::filesystem::path const& root = p_source.package;
    std::string const where = p_source.scenario_file.string();

    Scenario scenario;
    scenario.assets = p_assets;
    scenario.file = p_source.scenario_file;
    ROBOTIK_TRY(prefixed(
        where,
        readScenarioSection(scenario, p_source.scenario.at("scenario"), root)));

    ConfigNode const& world = p_source.world.at("world");
    if (!world.isMap())
    {
        return failure(p_source.world_file.string() + ": no 'world:' section");
    }
    ROBOTIK_TRY_ASSIGN(scenario.world,
                       prefixed(p_source.world_file.string(),
                                readWorld(world, root, p_assets)));
    ROBOTIK_TRY(prefixed(where, checkPhysics(scenario)));
    return scenario;
}

} // namespace robotik
