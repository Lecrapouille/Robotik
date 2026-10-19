// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/Scenario/Experiment.hpp"

#include "Robotik/Package/Scenario/PackagePath.hpp"

#include <algorithm>
#include <stdexcept>

namespace robotik
{

/// Reads the inject_fault steps that prepare one experiment case.
static void readSetup(ExperimentCase& p_case, ConfigNode const& p_setup)
{
    for (ConfigNode const& step : p_setup.items())
    {
        ConfigNode const& fault = step.at("inject_fault");
        if (!fault.isMap())
        {
            throw std::runtime_error(
                "setup steps are inject_fault: { resource, at, action } "
                "or { resource, rate }");
        }
        std::string resource = fault.text("resource");
        if (resource.empty())
        {
            throw std::runtime_error("inject_fault needs a resource");
        }
        if (fault.has("rate"))
        {
            p_case.random_faults.push_back(
                { std::move(resource), fault.number("rate", 0.0) });
            continue;
        }
        std::string const action = fault.text("action", "disable");
        if (action != "disable" && action != "restore")
        {
            throw std::runtime_error(
                "inject_fault action must be disable or restore");
        }
        p_case.faults.push_back({ Seconds(fault.number("at", 0.0)),
                                  std::move(resource),
                                  action == "disable" });
    }
}

bool Experiment::describes(std::filesystem::path const& p_path)
{
    return ConfigNode::load(p_path).has("experiment");
}

Experiment Experiment::load(std::filesystem::path const& p_path)
{
    Experiment experiment;
    experiment.file = std::filesystem::absolute(p_path).lexically_normal();
    ConfigNode const document = ConfigNode::load(experiment.file);
    ConfigNode const& node = document.at("experiment");
    try
    {
        std::filesystem::path const root = packageRoot(experiment.file);
        if (!node.isMap())
        {
            throw std::runtime_error("no 'experiment:' section");
        }
        experiment.id = node.text("id");
        if (experiment.id.empty())
        {
            throw std::runtime_error("experiment.id is required");
        }
        experiment.description = node.text("description");
        experiment.world = packageFile(root, node.text("world"));
        for (std::string const& seed : node.strings("seeds"))
        {
            std::optional<double> const value = ConfigNode(seed).asNumber();
            if (!value || *value < 0.0)
            {
                throw std::runtime_error("seeds must be non-negative integers");
            }
            experiment.seeds.push_back(static_cast<std::uint64_t>(*value));
        }
        if (!node.at("cases").isSequence())
        {
            throw std::runtime_error("cases must be a list");
        }
        for (ConfigNode const& item : node.at("cases").items())
        {
            ExperimentCase run;
            run.id = item.text("id");
            run.scenario = packageFile(root, item.text("scenario"));
            if (run.id.empty() || run.scenario.empty())
            {
                throw std::runtime_error(
                    "every case needs an id and a scenario");
            }
            for (ExperimentCase const& known : experiment.cases)
            {
                if (known.id == run.id)
                {
                    throw std::runtime_error("case '" + run.id +
                                             "' is declared twice");
                }
            }
            run.behavior_tree =
                packageFile(root, item.at("controller").text("behavior_tree"));
            for (auto const& [path, value] : item.at("overrides").entries())
            {
                run.overrides.emplace_back(path, value);
            }
            try
            {
                readSetup(run, item.at("setup"));
            }
            catch (std::runtime_error const& error)
            {
                throw std::runtime_error("case '" + run.id +
                                         "': " + error.what());
            }
            experiment.cases.push_back(std::move(run));
        }
        if (experiment.cases.empty())
        {
            throw std::runtime_error("an experiment needs at least one case");
        }
        ConfigNode const& evaluation = node.at("evaluation");
        for (ConfigNode const& item : evaluation.at("assertions").items())
        {
            experiment.assertions.push_back(AssertionSpec::read(item));
        }
        experiment.metrics = evaluation.strings("metrics");
    }
    catch (std::runtime_error const& error)
    {
        throw std::runtime_error(experiment.file.string() + ": " +
                                 error.what());
    }
    return experiment;
}

Scenario Experiment::scenario(ExperimentCase const& p_case,
                              AssetResolver const& p_assets) const
{
    ScenarioSource source = ScenarioSource::load(p_case.scenario);
    if (!world.empty())
    {
        source.world = ConfigNode::load(world);
        source.world_file = world;
    }
    for (auto const& [path, value] : p_case.overrides)
    {
        try
        {
            source.override(path, value);
        }
        catch (std::runtime_error const& error)
        {
            throw std::runtime_error(file.string() + ", case '" + p_case.id +
                                     "': " + error.what());
        }
    }
    Scenario scenario = Scenario::read(source, p_assets);
    if (!p_case.behavior_tree.empty())
    {
        scenario.controller.behavior_tree = p_case.behavior_tree;
    }
    scenario.faults.insert(
        scenario.faults.end(), p_case.faults.begin(), p_case.faults.end());
    // The injector expects its plan in time order.
    std::stable_sort(scenario.faults.begin(),
                     scenario.faults.end(),
                     [](Fault const& p_a, Fault const& p_b)
                     { return p_a.at < p_b.at; });
    scenario.random_faults.insert(scenario.random_faults.end(),
                                  p_case.random_faults.begin(),
                                  p_case.random_faults.end());
    scenario.assertions.insert(
        scenario.assertions.end(), assertions.begin(), assertions.end());
    return scenario;
}

} // namespace robotik
