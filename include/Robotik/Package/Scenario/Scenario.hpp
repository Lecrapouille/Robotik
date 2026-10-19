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

#include "Robotik/Package/Assets/AssetResolver.hpp"
#include "Robotik/Task/Planning/Pddl.hpp"
#include "Robotik/Task/Faults.hpp"
#include "Robotik/Package/Scenario/Assertions.hpp"
#include "Robotik/Package/Scenario/Config.hpp"
#include "Robotik/Package/Scenario/Trajectory.hpp"
#include "Robotik/Package/Scenario/World.hpp"

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
/// Change values here, then interpret with @ref Scenario::read. This is how
/// an experiment applies its @c overrides.
///
/// @code
/// robotik::ScenarioSource source = robotik::ScenarioSource::load(file);
/// source.override("world.entities.easel.friction",
/// robotik::ConfigNode("0.05")); source.override("scenario.timeout",
/// robotik::ConfigNode("60")); robotik::Scenario const scenario =
/// robotik::Scenario::read(source, assets);
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

    // -------------------------------------------------------------------------
    /// Reads the scenario file and the world file it names.
    /// @param p_scenario A @c x.scenario.yaml file.
    /// @return Both trees, not interpreted yet.
    /// @throws std::runtime_error when a file is missing or unreadable.
    // -------------------------------------------------------------------------
    [[nodiscard]] static ScenarioSource
    load(std::filesystem::path const& p_scenario);

    // -------------------------------------------------------------------------
    /// Changes one value before interpretation.
    /// @param p_path Starts with @c scenario. or @c world., e.g.
    ///   @c world.entities.easel.friction.
    /// @param p_value New value.
    /// @throws std::runtime_error for another prefix or a broken path.
    // -------------------------------------------------------------------------
    void override(std::string_view p_path, ConfigNode p_value);
};

// ****************************************************************************
/// Goal and settings of the PDDL planner (@c controller.planner:, see
/// @ref ControllerSpec for the YAML).
///
/// @code
/// if (scenario.controller.planner)
///     for (robotik::Literal const& g : scenario.controller.planner->goal)
///         std::cout << "goal: " << g.toString() << '\n';
/// @endcode
// ****************************************************************************
struct PlannerSpec
{
    /// PDDL domain of the plugin: types, predicates, actions.
    std::filesystem::path domain;
    /// What cannot be observed and does not change: compatible(vacuum, cube).
    std::vector<Literal> facts;
    /// Conjunction to reach.
    std::vector<Literal> goal;
    /// Goal in PDDL, for what a list of literals cannot say (forall...).
    /// Replaces @ref goal; not checked against the beliefs.
    std::string goal_pddl;
    /// New plans allowed after the first one (failure, effect not seen).
    std::uint32_t max_replans = 3u;
    /// Simulated time waited before a new plan.
    Seconds replan_delay{ 0.5 };
    /// Checks the effects of each action on the beliefs once it is done.
    bool verify = true;
    /// Runs together the steps that do not depend on each other (see
    /// PlanGraph.hpp); false runs the plan step by step.
    bool parallel = true;
    /// Search preset of the planner.
    std::string search = "lama";
    /// Wall-clock safety net of one search (s).
    Seconds timeout{ 10.0 };
};

// ****************************************************************************
/// The strategy driving the robot: a behavior tree, a planner goal, or
/// neither (the mission drives it).
///
/// @code
/// controller:
///   planner:
///     domain: scenarios/pick_and_place.domain.pddl
///     facts:                              # block lists: a flow list [a(x, y)]
///       - compatible(vacuum, cube)        # would split at the comma
///     goal:
///       - in(red_cube, box)               # or goal_pddl: "(forall ...)"
///     replan: { max: 3, delay: 0.5 }      # new plans, simulated seconds
///     before each verify: true                        # check the effects on
///     the beliefs parallel: true                      # independent steps run
///     together search: lama                        # planner search preset
///     timeout: 10                         # wall-clock safety net (s)
/// @endcode
///
/// The objects of the problem are the world entities with a @c kind; the
/// state comes from the observers of the plugin (@ref PlanningDomain).
// ****************************************************************************
struct ControllerSpec
{
    /// Behavior tree file, or empty.
    std::filesystem::path behavior_tree;
    /// Planner goal, or nothing.
    std::optional<PlannerSpec> planner;
};

// ****************************************************************************
/// When the ground truth may stand in for perception (@c perception.oracle:).
// ****************************************************************************
enum class OraclePolicy : std::uint8_t
{
    Never,     ///< @c never: beliefs come from perception only.
    WhenBlind, ///< @c when_blind: when no camera has an image source.
    Always,    ///< @c always: the true poses, whatever the cameras.
};

// ****************************************************************************
/// How finely the physics simulates an interaction (FIDELITY.md).
// ****************************************************************************
enum class Fidelity : std::uint8_t
{
    /// A held object follows the tool, a released one is put down where it
    /// belongs: no contact, no fall. Checks the logic and the plan.
    Abstract,
    /// Objects are rigid bodies of the physics engine: a grasp welds the
    /// object to the tool, a release lets it fall for real.
    Rigid,
    /// No weld: the tool holds by force (suction, friction). Not built yet.
    Contact,
};

// ----------------------------------------------------------------------------
/// @param p_fidelity Level to name.
/// @return "abstract", "rigid" or "contact".
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(Fidelity p_fidelity);

// ****************************************************************************
/// Section @c physics: of a scenario.
///
/// @code{.yaml}
/// physics:
///   fidelity: abstract        # abstract (default) | rigid | contact
///   overrides:                # per world entity or actuator
///     red_cube: rigid
/// @endcode
// ****************************************************************************
struct PhysicsSpec
{
    /// Level of everything not in @ref overrides.
    Fidelity fidelity = Fidelity::Abstract;
    /// Level per world entity or actuator id.
    std::map<std::string, Fidelity, std::less<>> overrides;

    // -------------------------------------------------------------------------
    /// @param p_id Entity or actuator id.
    /// @return Its level.
    // -------------------------------------------------------------------------
    [[nodiscard]] Fidelity of(std::string_view p_id) const
    {
        auto const found = overrides.find(p_id);
        return found == overrides.end() ? fidelity : found->second;
    }

    // -------------------------------------------------------------------------
    /// @param p_a Id of one side.
    /// @param p_b Id of the other side.
    /// @return Level of an interaction: the most detailed of its two sides.
    // -------------------------------------------------------------------------
    [[nodiscard]] Fidelity between(std::string_view p_a,
                                   std::string_view p_b) const
    {
        return std::max(of(p_a), of(p_b));
    }
};

// ****************************************************************************
/// Interpreted scenario: everything a @ref Simulation needs to run.
///
/// @code
/// auto const assets = robotik::AssetResolver::standard();
/// robotik::Scenario const scenario = robotik::Scenario::load(file, assets);
/// std::cout << scenario.id << ": " << scenario.world.entities.size()
///           << " entities, timeout " << scenario.timeout.value() << " s\n";
/// double const h = scenario.parameters.number("approach_height", 0.1);
/// @endcode
// ****************************************************************************
struct Scenario
{
    // -------------------------------------------------------------------------
    /// Interprets the two files of a scenario.
    /// @param p_source Scenario and world trees, maybe overridden.
    /// @param p_assets Resolves the model names.
    /// @return The scenario.
    /// @throws std::runtime_error naming the file and the field at fault, or
    ///   a model missing from @p_assets.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Scenario read(ScenarioSource const& p_source,
                                       AssetResolver const& p_assets);

    // -------------------------------------------------------------------------
    /// Reads and interprets a scenario file and its world.
    /// @param p_path A @c x.scenario.yaml file.
    /// @param p_assets Resolves the model names.
    /// @return The scenario.
    /// @throws std::runtime_error as @ref read and @ref ScenarioSource::load.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Scenario load(std::filesystem::path const& p_path,
                                       AssetResolver const& p_assets)
    {
        return read(ScenarioSource::load(p_path), p_assets);
    }

    /// Scenario id.
    std::string id;
    /// Free text for the user.
    std::string description;
    /// File the scenario was read from.
    std::filesystem::path file;
    /// Master seed of the run; every random stream derives from it.
    std::uint64_t seed = 0;
    /// The world the scenario runs in.
    WorldDescription world;
    /// Behavior tree, planner goal, or neither.
    ControllerSpec controller;
    /// Paths named by the scenario, read from their own files.
    std::map<std::string, Trajectory, std::less<>> trajectories;
    /// Faults at fixed times.
    std::vector<Fault> faults;
    /// Faults drawn at random.
    std::vector<RandomFault> random_faults;
    /// When the ground truth may stand in for perception.
    OraclePolicy oracle = OraclePolicy::Never;
    /// Fidelity of the physics, per entity.
    PhysicsSpec physics;
    /// Simulated time limit; zero means none.
    Seconds timeout{};
    /// Condition ending the run as soon as it holds; empty for none.
    std::string stop_when;
    /// What is checked during and at the end of the run.
    std::vector<AssertionSpec> assertions;
    /// Values the mission reads (gains, model choices...). An experiment
    /// overrides them as scenario.parameters.<name>.
    ConfigNode parameters;
    /// Packs the scenario was resolved with; missions resolve their own
    /// models with it.
    AssetResolver assets;
};

} // namespace robotik
