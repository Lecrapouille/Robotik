// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Experiment.hpp
//! @brief How scenarios are run and compared: cases, seeds and evaluation.
//!
//! @code
//! experiment:
//!   id: painting_benchmark
//!   seeds: [7, 8]                      # every case, every seed
//!   cases:
//!     - id: nominal
//!       scenario: scenarios/painter.scenario.yaml
//!     - id: slippery_easel
//!       scenario: scenarios/painter.scenario.yaml
//!       overrides:                     # applied before the files are read
//!         world.entities.easel.friction: 0.05
//!     - id: camera_failure
//!       scenario: scenarios/pick_and_place.scenario.yaml
//!       controller: { behavior_tree: scenarios/other.bt.yaml }
//!       setup:
//!         - inject_fault: { resource: wrist_camera, at: 2.0, action: disable }
//!   evaluation:
//!     assertions:                      # added to the scenario's own
//!       - { condition: "pose_error(easel) < 0.002", phase: always }
//!     metrics: [ time, pose_error(easel), collision_count(robot) ]
//! @endcode
//!
//! A case is a variant of the experiment, not a mutation of the previous
//! one: each run rebuilds its world from the files, then resets it with its
//! seed. An optional @c world: replaces the world of every scenario.
#pragma once

#include "Robotik/Assets/AssetResolver.hpp"
#include "Robotik/Scenario/Scenario.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief One variant of an experiment.
// ****************************************************************************
struct ExperimentCase
{
    std::string id;
    std::filesystem::path scenario;
    //!< Replaces the scenario controller when set.
    std::filesystem::path behavior_tree;
    //!< Dotted path (scenario.* or world.*) and the value to put there.
    std::vector<std::pair<std::string, ConfigNode>> overrides;
    //!< Faults added to those of the scenario.
    std::vector<Fault> faults;
    std::vector<RandomFault> random_faults;
};

// ****************************************************************************
//! @brief Parsed experiment file.
// ****************************************************************************
struct Experiment
{
    //! @throws std::runtime_error naming the file and the field at fault.
    [[nodiscard]] static Experiment load(std::filesystem::path const& p_path);

    //! @brief True when the root of @p_path holds @c experiment:.
    [[nodiscard]] static bool describes(std::filesystem::path const& p_path);

    std::string id;
    std::string description;
    std::filesystem::path file;
    //!< Replaces the world of every scenario when set.
    std::filesystem::path world;
    //!< Empty: each scenario runs with its own seed.
    std::vector<std::uint64_t> seeds;
    std::vector<ExperimentCase> cases;
    std::vector<AssertionSpec> assertions;
    //!< Values recorded at the end of every run (metric names or queries).
    std::vector<std::string> metrics;

    // -------------------------------------------------------------------------
    //! @brief The scenario of @p_case once the experiment is applied: world,
    //! overrides, controller, extra faults and extra assertions.
    // -------------------------------------------------------------------------
    [[nodiscard]] Scenario scenario(ExperimentCase const& p_case,
                                    AssetResolver const& p_assets) const;
};

} // namespace robotik
