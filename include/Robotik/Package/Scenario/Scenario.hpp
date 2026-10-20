// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Scenario.hpp
/// What happens in a world, and how to tell whether it went well.
///
/// The scenario names its world (@ref World.hpp), the events applied during
/// the run (faults), when the run stops and what is checked
/// (@ref Assertions.hpp). It does not say how the robot solves the task: the
/// @c controller section only gives the default strategy, which an
/// experiment may replace.
///
/// @code
/// scenario:
///   id: pick_and_place
///   description: Pick the red cube and put it in the box.
///   world: worlds/manipulation_cell.world.yaml
///   seed: 42
///   controller:
///     behavior_tree: scenarios/pick_and_place.bt.yaml
///     # or, instead of a tree, a goal for the PDDL planner (PlannerSpec):
///     # planner:
///     #   domain: scenarios/pick_and_place.domain.pddl
///     #   goal:
///     #     - in(red_cube, box)
///   trajectories:                          # see Trajectory.hpp
///     drop: scenarios/trajectories/drop.trajectory.yaml
///   faults:
///     - { at: 4.0, resource: wrist_camera, action: disable }
///     - { resource: gripper, rate: 0.01 }
///   perception: { oracle: when_blind }     # never (default) | when_blind |
///   always timeout: 30 stop_when: inside(red_cube, box)       # optional
///   parameters:                            # free, read by the mission
///     approach_height: 0.10
///   assertions:
///     - controller.success
///     - inside(red_cube, box)
/// @endcode
///
/// The oracle copies the true pose of the props into what the robot believes.
/// It is off unless the scenario asks for it: @c when_blind turns it on when
/// no camera has an image source (a headless run), @c always regardless.
/// Results obtained with it are labelled as such.
///
/// Files are relative to the package root and stay inside it
/// (@ref PackagePath.hpp).
#pragma once

#include "Robotik/Robot/Assets/AssetResolver.hpp"
#include "Robotik/Task/Planning/Pddl.hpp"
#include "Robotik/Task/Faults.hpp"
#include "Robotik/Package/Scenario/Assertions.hpp"
#include "Robotik/Common/Config.hpp"
#include "Robotik/Common/Result.hpp"
#include "Robotik/Package/Scenario/Trajectory.hpp"
#include "Robotik/Package/Scenario/World.hpp"
#include "Robotik/Task/CellDescription.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{
// ****************************************************************************
/// The two files of a scenario, still editable (see @ref ConfigNode).
///
/// Change values here, then interpret with @ref readScenario. This is how
/// an experiment applies its @c overrides.
///
/// @code
/// robotik::ScenarioSource source = robotik::loadScenarioSource(file).take();
/// (void)robotik::overrideValue(source, "world.entities.easel.friction",
/// robotik::ConfigNode("0.05")); (void)robotik::overrideValue(source, "scenario.timeout",
/// robotik::ConfigNode("60")); robotik::Scenario const scenario =
/// robotik::readScenario(source, assets).take();
/// @endcode
// ****************************************************************************
struct ScenarioSource
{
    /// Root of the scenario file (holds @c scenario:).
    ConfigNode scenario;
    /// Path of the scenario file.
    std::filesystem::path scenario_file;
    /// Package root: the folder of @c plugin.yaml. File names start there.
    std::filesystem::path package;
    /// Root of the world file (holds @c world:).
    ConfigNode world;
    /// Path of the world file.
    std::filesystem::path world_file;
};

// ****************************************************************************
/// Interpreted scenario: a @ref CellDescription (what to build) plus how the
/// run is judged (time limit, stop condition, assertions).
///
/// @code
/// auto const assets = robotik::AssetResolver::standard();
/// robotik::Scenario const scenario = robotik::loadScenario(file, assets).take();
/// std::cout << scenario.id << ": " << scenario.world.entities.size()
///           << " entities, timeout " << scenario.timeout.value() << " s\n";
/// double const h = scenario.parameters.number("approach_height", 0.1);
/// @endcode
// ****************************************************************************
struct Scenario : CellDescription
{
    /// Free text for the user.
    std::string description;
    /// File the scenario was read from.
    std::filesystem::path file;
    /// Simulated time limit; zero means none.
    Seconds timeout{};
    /// Condition ending the run as soon as it holds; empty for none.
    std::string stop_when;
    /// What is checked during and at the end of the run.
    std::vector<AssertionSpec> assertions;
};

// ----------------------------------------------------------------------------
/// Reads the scenario file and the world file it names.
/// @param p_scenario A @c x.scenario.yaml file.
/// @return Both trees, not interpreted yet; fails when a file is missing or
///   unreadable.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<ScenarioSource> loadScenarioSource(std::filesystem::path const& p_scenario);

// ----------------------------------------------------------------------------
/// Changes one value before interpretation.
/// @param p_source Trees to change.
/// @param p_path Starts with @c scenario. or @c world., e.g.
///   @c world.entities.easel.friction.
/// @param p_value New value.
/// @return Fails for another prefix or a broken path.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<void> overrideValue(ScenarioSource& p_source,
                                         std::string_view p_path,
                                         ConfigNode p_value);

// ----------------------------------------------------------------------------
/// Interprets the two files of a scenario.
/// @param p_source Scenario and world trees, maybe overridden.
/// @param p_assets Resolves the model names.
/// @return The scenario; the error names the file and the field at fault, or
///   a model missing from @p_assets.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<Scenario> readScenario(ScenarioSource const& p_source,
                                            AssetResolver const& p_assets);

// ----------------------------------------------------------------------------
/// Reads and interprets a scenario file and its world.
/// @param p_path A @c x.scenario.yaml file.
/// @param p_assets Resolves the model names.
/// @return The scenario, or the error of @ref loadScenarioSource or
///   @ref readScenario.
// ----------------------------------------------------------------------------
[[nodiscard]] inline Result<Scenario> loadScenario(std::filesystem::path const& p_path,
                                                   AssetResolver const& p_assets)
{
    Result<ScenarioSource> source = loadScenarioSource(p_path);
    if (!source)
    {
        return failure(source.error());
    }
    return readScenario(source.value(), p_assets);
}

} // namespace robotik
