// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file FileRole.hpp
//! @brief The role of a package file, read from its double extension.
//!
//! @code{.text}
//! worlds/arm.world.yaml                   world
//! scenarios/wave.scenario.yaml            scenario
//! scenarios/pick.rl.yaml                  scenario driven by a learned policy (id pick_rl)
//! scenarios/wave.bt.yaml                  behavior tree
//! scenarios/pick.domain.pddl              PDDL domain
//! scenarios/trajectories/red.trajectory.yaml
//! experiments/bench.experiment.yaml       experiment
//! @endcode
//!
//! It is a convention, not a rule: the loaders read the content and never
//! refuse a file for its name. Files without a role (an older
//! @c wave.yml) keep working.
#pragma once

#include <filesystem>
#include <string>

namespace robotik
{

enum class FileRole
{
    World,
    Scenario,
    Rl,
    Tree,
    Domain,
    Trajectory,
    Experiment,
    //!< No known double extension.
    Unknown,
};

//! @brief Role of @p_file from its name: @c x.world.yaml is a world.
[[nodiscard]] FileRole fileRole(std::filesystem::path const& p_file);

//! @brief Double extension of @p_role, e.g. ".world.yaml"; empty for Unknown.
[[nodiscard]] char const* roleExtension(FileRole p_role);

//! @brief Id given by a file name: the extension is dropped, the default
//! role @c .scenario too, and every other dot becomes @c _.
//! @c wave.scenario.yaml -> @c wave, @c pick.rl.yaml -> @c pick_rl,
//! @c hello.experiment.yaml -> @c hello_experiment, @c wave.yml -> @c wave.
[[nodiscard]] std::string roleStem(std::filesystem::path const& p_file);

//! @brief True for a file the scenario list may hold: a scenario, an RL
//! scenario, an experiment, or a YAML file without a role.
[[nodiscard]] bool runnableRole(std::filesystem::path const& p_file);

} // namespace robotik
