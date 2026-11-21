// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/Scenario/Experiment.hpp"

#include "Robotik/Package/Scenario/PackagePath.hpp"

#include <algorithm>

namespace robotik
{

/// Reads the inject_fault steps that prepare one experiment case.
static Result<void> readSetup(ExperimentCase& p_case, ConfigNode const& p_setup)
{
    for (ConfigNode const& step : p_setup.items())
    {
        ROBOTIK_TRY(step.checkKeys({ "inject_fault" }, "setup"));
        ConfigNode const& fault = step.at("inject_fault");
        if (!fault.isMap())
        {
            return failure("setup steps are inject_fault: { resource, at, "
                           "action } or { resource, rate }");
        }
        ROBOTIK_TRY(fault.checkKeys({ "resource", "rate", "action", "at" },
                                    "inject_fault"));
        std::string resource = fault.text("resource");
        if (resource.empty())
        {
            return failure("inject_fault needs a resource");
        }
        if (fault.has("rate"))
        {
            double rate = 0.0;
            ROBOTIK_TRY_ASSIGN(rate, fault.number("rate", 0.0));
            p_case.random_faults.push_back({ std::move(resource), rate });
            continue;
        }
        std::string const action = fault.text("action", "disable");
        if (action != "disable" && action != "restore")
        {
            return failure("inject_fault action must be disable or restore");
        }
        double at = 0.0;
        ROBOTIK_TRY_ASSIGN(at, fault.number("at", 0.0));
        p_case.faults.push_back(
            { Seconds(at), std::move(resource), action == "disable" });
    }
    return success();
}

/// Reads one case of the experiment.
static Result<ExperimentCase> readCase(Experiment const& p_experiment,
                                       std::filesystem::path const& p_root,
                                       ConfigNode const& p_item)
{
    ExperimentCase run;
    ROBOTIK_TRY(p_item.checkKeys(
        { "id", "scenario", "controller", "overrides", "setup" },
        "case '" + p_item.text("id") + "'"));
    ROBOTIK_TRY(
        p_item.at("controller").checkKeys({ "behavior_tree" }, "controller"));
    run.id = p_item.text("id");
    ROBOTIK_TRY_ASSIGN(run.scenario,
                       packageFile(p_root, p_item.text("scenario")));
    if (run.id.empty() || run.scenario.empty())
    {
        return failure("every case needs an id and a scenario");
    }
    for (ExperimentCase const& known : p_experiment.cases)
    {
        if (known.id == run.id)
        {
            return failure("case '" + run.id + "' is declared twice");
        }
    }
    ROBOTIK_TRY_ASSIGN(
        run.behavior_tree,
        packageFile(p_root, p_item.at("controller").text("behavior_tree")));
    for (auto const& [path, value] : p_item.at("overrides").entries())
    {
        run.overrides.emplace_back(path, value);
    }
    if (Result<void> const setup = readSetup(run, p_item.at("setup")); !setup)
    {
        return failure("case '" + run.id + "': " + setup.error());
    }
    return run;
}

/// Reads the experiment section; errors are prefixed by the caller.
static Result<void> readExperiment(Experiment& p_experiment,
                                   ConfigNode const& p_document)
{
    Experiment& experiment = p_experiment;
    ConfigNode const& node = p_document.at("experiment");
    std::filesystem::path root;
    ROBOTIK_TRY_ASSIGN(root, packageRoot(experiment.file));
    if (!node.isMap())
    {
        return failure("no 'experiment:' section");
    }
    ROBOTIK_TRY(p_document.checkKeys({ "experiment" }, "file"));
    ROBOTIK_TRY(node.checkKeys(
        { "id", "description", "world", "seeds", "cases", "evaluation" },
        "experiment"));
    experiment.id = node.text("id");
    if (experiment.id.empty())
    {
        return failure("experiment.id is required");
    }
    experiment.description = node.text("description");
    ROBOTIK_TRY_ASSIGN(experiment.world, packageFile(root, node.text("world")));
    for (std::string const& seed : node.strings("seeds"))
    {
        std::optional<double> const value = ConfigNode(seed).asNumber();
        if (!value || *value < 0.0)
        {
            return failure("seeds must be non-negative integers");
        }
        experiment.seeds.push_back(static_cast<std::uint64_t>(*value));
    }
    if (!node.at("cases").isSequence())
    {
        return failure("cases must be a list");
    }
    for (ConfigNode const& item : node.at("cases").items())
    {
        ExperimentCase run;
        ROBOTIK_TRY_ASSIGN(run, readCase(experiment, root, item));
        experiment.cases.push_back(std::move(run));
    }
    if (experiment.cases.empty())
    {
        return failure("an experiment needs at least one case");
    }
    ConfigNode const& evaluation = node.at("evaluation");
    ROBOTIK_TRY(
        evaluation.checkKeys({ "assertions", "metrics" }, "evaluation"));
    for (ConfigNode const& item : evaluation.at("assertions").items())
    {
        AssertionSpec assertion;
        ROBOTIK_TRY_ASSIGN(assertion, readAssertion(item));
        experiment.assertions.push_back(std::move(assertion));
    }
    experiment.metrics = evaluation.strings("metrics");
    return success();
}

bool isExperiment(std::filesystem::path const& p_path)
{
    Result<ConfigNode> const document = ConfigNode::load(p_path);
    return document && document.value().has("experiment");
}

Result<Experiment> loadExperiment(std::filesystem::path const& p_path)
{
    Experiment experiment;
    experiment.file = std::filesystem::absolute(p_path).lexically_normal();
    ConfigNode document;
    ROBOTIK_TRY_ASSIGN(document, ConfigNode::load(experiment.file));
    if (Result<void> const read = readExperiment(experiment, document); !read)
    {
        return failure(experiment.file.string() + ": " + read.error());
    }
    return experiment;
}

Result<Scenario> scenarioOf(Experiment const& p_experiment,
                            ExperimentCase const& p_case,
                            AssetResolver const& p_assets)
{
    ScenarioSource source;
    ROBOTIK_TRY_ASSIGN(source, loadScenarioSource(p_case.scenario));
    if (!p_experiment.world.empty())
    {
        ROBOTIK_TRY_ASSIGN(source.world, ConfigNode::load(p_experiment.world));
        source.world_file = p_experiment.world;
    }
    for (auto const& [path, value] : p_case.overrides)
    {
        if (Result<void> const changed = overrideValue(source, path, value);
            !changed)
        {
            return failure(p_experiment.file.string() + ", case '" + p_case.id +
                           "': " + changed.error());
        }
    }
    Scenario scenario;
    ROBOTIK_TRY_ASSIGN(scenario, readScenario(source, p_assets));
    if (!p_case.behavior_tree.empty())
    {
        scenario.controller.behavior_tree = p_case.behavior_tree;
    }
    scenario.faults.insert(
        scenario.faults.end(), p_case.faults.begin(), p_case.faults.end());
    // The injector expects its plan in time order.
    std::ranges::stable_sort(scenario.faults,
                             [](Fault const& p_a, Fault const& p_b)
                             { return p_a.at < p_b.at; });
    scenario.random_faults.insert(scenario.random_faults.end(),
                                  p_case.random_faults.begin(),
                                  p_case.random_faults.end());
    scenario.assertions.insert(scenario.assertions.end(),
                               p_experiment.assertions.begin(),
                               p_experiment.assertions.end());
    return scenario;
}

} // namespace robotik
