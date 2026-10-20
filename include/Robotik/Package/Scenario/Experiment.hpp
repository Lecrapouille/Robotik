// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Experiment.hpp
/// How scenarios are run and compared: cases, seeds and evaluation.
///
/// @code
/// experiment:
///   id: painter_experiment
///   seeds: [7, 8]                      # every case, every seed
///   cases:
///     - id: nominal
///       scenario: scenarios/painter.scenario.yaml
///     - id: slippery_easel
///       scenario: scenarios/painter.scenario.yaml
///       overrides:                     # applied before the files are read
///         world.entities.easel.friction: 0.05
///     - id: camera_failure
///       scenario: scenarios/pick_and_place.scenario.yaml
///       controller: { behavior_tree: scenarios/other.bt.yaml }
///       setup:
///         - inject_fault: { resource: wrist_camera, at: 2.0, action: disable }
///   evaluation:
///     assertions:                      # added to the scenario's own
///       - { condition: "pose_error(easel) < 0.002", phase: always }
///     metrics: [ time, pose_error(easel), collision_count(robot) ]
/// @endcode
///
/// A case is a variant of the experiment, not a mutation of the previous
/// one: each run rebuilds its world from the files, then resets it with its
/// seed. An optional @c world: replaces the world of every scenario.
#pragma once

#include "Robotik/Common/Result.hpp"
#include "Robotik/Robot/Assets/AssetResolver.hpp"
#include "Robotik/Package/Scenario/Scenario.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// One variant of an experiment.
///
/// @code
/// for (robotik::ExperimentCase const& c : experiment.cases)
///     std::cout << c.id << ": " << c.scenario << ", "
///               << c.overrides.size() << " overrides\n";
/// @endcode
// ****************************************************************************
struct ExperimentCase
{
    /// Case id, unique in the experiment.
    std::string id;
    /// Scenario file of the case.
    std::filesystem::path scenario;
    /// Replaces the scenario controller when set.
    std::filesystem::path behavior_tree;
    /// Dotted path (scenario.* or world.*) and the value to put there.
    std::vector<std::pair<std::string, ConfigNode>> overrides;
    /// Faults added to those of the scenario.
    std::vector<Fault> faults;
    /// Random faults added to those of the scenario.
    std::vector<RandomFault> random_faults;
};

// ****************************************************************************
/// Parsed experiment file.
///
/// Each case is run once per seed; @ref scenario builds what one run needs.
///
/// @code
/// auto const experiment = robotik::loadExperiment(file).take();
/// for (auto const& c : experiment.cases)
///     for (std::uint64_t seed : experiment.seeds)
///         run(robotik::scenarioOf(experiment, c, assets).take(), robotik::Seed(seed));
/// @endcode
// ****************************************************************************
struct Experiment
{
    /// Experiment id.
    std::string id;
    /// Free text for the user.
    std::string description;
    /// File the experiment was read from.
    std::filesystem::path file;
    /// Replaces the world of every scenario when set.
    std::filesystem::path world;
    /// Empty: each scenario runs with its own seed.
    std::vector<std::uint64_t> seeds;
    /// Variants to run.
    std::vector<ExperimentCase> cases;
    /// Assertions added to those of every scenario.
    std::vector<AssertionSpec> assertions;
    /// Values recorded at the end of every run (metric names or queries).
    std::vector<std::string> metrics;

};

// ----------------------------------------------------------------------------
/// Reads an experiment file.
/// @param p_path A @c x.experiment.yaml file.
/// @return The experiment; the error names the file and the field at fault.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<Experiment> loadExperiment(std::filesystem::path const& p_path);

// ----------------------------------------------------------------------------
/// Tells whether a YAML file is an experiment, whatever its name.
/// @param p_path YAML file to look at.
/// @return True when its root holds @c experiment:.
// ----------------------------------------------------------------------------
[[nodiscard]] bool isExperiment(std::filesystem::path const& p_path);

// ----------------------------------------------------------------------------
/// Builds the scenario of one case: world, overrides, controller, extra
/// faults and extra assertions are applied.
/// @param p_experiment Experiment the case belongs to.
/// @param p_case Case of this experiment.
/// @param p_assets Resolves the models named by the files.
/// @return The scenario ready to run, or why it cannot be built.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<Scenario> scenarioOf(Experiment const& p_experiment,
                                          ExperimentCase const& p_case,
                                          AssetResolver const& p_assets);

} // namespace robotik
