// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/Scenario/Experiment.hpp"
#include "Robotik/Common/Must.hpp"

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

bool isExperiment(std::filesystem::path const& p_path)
{
    Result<ConfigNode> const document = ConfigNode::load(p_path);
    return document && document.value().has("experiment");
}

Result<Experiment> loadExperiment(std::filesystem::path const& p_path)
{
    // The border of this reader: whatever stops it inside becomes a message.
    try
    {
        Experiment experiment;
        experiment.file = std::filesystem::absolute(p_path).lexically_normal();
        ConfigNode const document = must(ConfigNode::load(experiment.file));
        ConfigNode const& node = document.at("experiment");
        try
        {
            std::filesystem::path const root = must(packageRoot(experiment.file));
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
            experiment.world = must(packageFile(root, node.text("world")));
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
                run.scenario = must(packageFile(root, item.text("scenario")));
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
                    must(packageFile(root, item.at("controller").text("behavior_tree")));
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
                experiment.assertions.push_back(must(readAssertion(item)));
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
    catch (std::exception const& error)
    {
        return failure(error.what());
    }
}

Result<Scenario> scenarioOf(Experiment const& p_experiment,
                            ExperimentCase const& p_case,
                            AssetResolver const& p_assets)
{
    // The border of this reader: whatever stops it inside becomes a message.
    try
    {
        ScenarioSource source = must(loadScenarioSource(p_case.scenario));
        if (!p_experiment.world.empty())
        {
            source.world = must(ConfigNode::load(p_experiment.world));
            source.world_file = p_experiment.world;
        }
        for (auto const& [path, value] : p_case.overrides)
        {
            if (Result<void> const changed = overrideValue(source, path, value); !changed)
            {
                return failure(p_experiment.file.string() + ", case '" + p_case.id +
                               "': " + changed.error());
            }
        }
        Scenario scenario = must(readScenario(source, p_assets));
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
            scenario.assertions.end(), p_experiment.assertions.begin(), p_experiment.assertions.end());
        return scenario;
    }
    catch (std::exception const& error)
    {
        return failure(error.what());
    }
}

} // namespace robotik
