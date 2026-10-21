// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file FileRole.hpp
/// The role of a package file, read from its double extension.
///
/// @code{.text}
/// worlds/arm.world.yaml                   world
/// scenarios/wave.scenario.yaml            scenario
/// scenarios/pick.rl.yaml                  learned policy scenario (id pick_rl)
/// scenarios/wave.bt.yaml                  behavior tree
/// scenarios/pick.domain.pddl              PDDL domain
/// scenarios/trajectories/red.trajectory.yaml
/// experiments/bench.experiment.yaml       experiment
/// @endcode
///
/// It is a convention, not a rule: the loaders read the content and never
/// refuse a file for its name. Files without a role (an older
/// @c wave.yml) keep working.
///
/// @par Contract
/// - Promises: a name without a known role is still accepted.
#pragma once

#include <filesystem>
#include <string>

namespace robotik
{

// ****************************************************************************
/// What a package file is for, from its name.
///
/// @code
/// if (robotik::fileRole(path) == robotik::FileRole::World)
///     loadWorld(path);
/// @endcode
// ****************************************************************************
enum class FileRole
{
    World,      ///< @c x.world.yaml: robots, objects, sensors.
    Scenario,   ///< @c x.scenario.yaml: a world plus a mission.
    Rl,         ///< @c x.rl.yaml: a scenario driven by a learned policy.
    Tree,       ///< @c x.bt.yaml: a behavior tree.
    Domain,     ///< @c x.domain.pddl: a PDDL domain.
    Trajectory, ///< @c x.trajectory.yaml: a list of waypoints.
    Experiment, ///< @c x.experiment.yaml: several runs to compare.
    Grafcet,    ///< @c x.grafcet.json: a GRAFCET (TimedPetriNetEditor file).
    Workflow,   ///< @c x.workflow.yaml: machines, products and routes of a cell.
    /// No known double extension.
    Unknown,
};

// ----------------------------------------------------------------------------
/// Reads the role of a file from its name.
/// @param p_file File name or path; only the name is read.
/// @return The role, @c x.world.yaml is a world; @c Unknown otherwise.
// ----------------------------------------------------------------------------
[[nodiscard]] FileRole fileRole(std::filesystem::path const& p_file);

// ----------------------------------------------------------------------------
/// Gives the double extension of a role.
/// @param p_role Role to name.
/// @return For example ".world.yaml"; empty for Unknown.
// ----------------------------------------------------------------------------
[[nodiscard]] char const* roleExtension(FileRole p_role);

// ----------------------------------------------------------------------------
/// Id given by a file name: the extension is dropped, the default role
/// @c .scenario too, and every other dot becomes @c _.
///
/// @code{.text}
/// wave.scenario.yaml    -> wave
/// pick.rl.yaml          -> pick_rl
/// hello.experiment.yaml -> hello_experiment
/// wave.yml              -> wave
/// @endcode
///
/// @param p_file File name or path.
/// @return The id, as used in @c plugin.yaml.
// ----------------------------------------------------------------------------
[[nodiscard]] std::string roleStem(std::filesystem::path const& p_file);

// ----------------------------------------------------------------------------
/// Tells whether a file can be run from the scenario list.
/// @param p_file File name or path.
/// @return True for a scenario, an RL scenario, an experiment, or a YAML
///   file without a role.
// ----------------------------------------------------------------------------
[[nodiscard]] bool runnableRole(std::filesystem::path const& p_file);

} // namespace robotik
