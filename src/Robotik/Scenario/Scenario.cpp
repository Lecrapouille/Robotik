// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Scenario/Scenario.hpp"

#include <stdexcept>

namespace robotik
{

namespace
{

std::filesystem::path relative(std::filesystem::path const& p_directory,
                               std::string const& p_file)
{
    return p_file.empty() ? std::filesystem::path{}
                          : (p_directory / p_file).lexically_normal();
}

void readFaults(Scenario& p_scenario, ConfigNode const& p_node)
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

} // namespace

ScenarioSource ScenarioSource::load(std::filesystem::path const& p_scenario)
{
    ScenarioSource source;
    source.scenario_file = std::filesystem::absolute(p_scenario).lexically_normal();
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
    source.world_file = relative(source.scenario_file.parent_path(), world);
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

Scenario Scenario::read(ScenarioSource const& p_source, AssetResolver const& p_assets)
{
    ConfigNode const& node = p_source.scenario.at("scenario");
    std::filesystem::path const directory = p_source.scenario_file.parent_path();
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
        scenario.controller.behavior_tree =
            relative(directory, node.at("controller").text("behavior_tree"));
        for (auto const& [name, file] : node.at("trajectories").entries())
        {
            if (!file.isScalar())
            {
                throw std::runtime_error("trajectories." + name + " must be a file name");
            }
            scenario.trajectories.emplace(name, Trajectory::load(relative(directory, file.scalar())));
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
            WorldDescription::read(world, p_source.world_file.parent_path(), p_assets);
    }
    catch (std::runtime_error const& error)
    {
        throw std::runtime_error(p_source.world_file.string() + ": " + error.what());
    }
    return scenario;
}

} // namespace robotik
