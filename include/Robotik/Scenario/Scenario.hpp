// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Scenario.hpp
//! @brief What happens in a world, and how to tell whether it went well.
//!
//! The scenario names its world (@ref World.hpp), the events applied during
//! the run (faults), when the run stops and what is checked
//! (@ref Assertions.hpp). It does not say how the robot solves the task: the
//! @c controller section only gives the default strategy, which an
//! experiment may replace.
//!
//! @code
//! scenario:
//!   id: pick_and_place
//!   description: Pick the red cube and put it in the box.
//!   world: worlds/manipulation_cell.world.yaml
//!   seed: 42
//!   controller:
//!     behavior_tree: scenarios/pick_and_place.bt.yaml
//!     # or, instead of a tree, a goal for the PDDL planner (PlannerSpec):
//!     # planner:
//!     #   domain: scenarios/pick_and_place.domain.pddl
//!     #   goal:
//!     #     - in(red_cube, box)
//!   trajectories:                          # see Trajectory.hpp
//!     drop: scenarios/trajectories/drop.trajectory.yaml
//!   faults:
//!     - { at: 4.0, resource: wrist_camera, action: disable }
//!     - { resource: gripper, rate: 0.01 }
//!   perception: { oracle: when_blind }     # never (default) | when_blind | always
//!   timeout: 30
//!   stop_when: inside(red_cube, box)       # optional
//!   parameters:                            # free, read by the mission
//!     approach_height: 0.10
//!   assertions:
//!     - controller.success
//!     - inside(red_cube, box)
//! @endcode
//!
//! The oracle copies the true pose of the props into what the robot believes.
//! It is off unless the scenario asks for it: @c when_blind turns it on when
//! no camera has an image source (a headless run), @c always regardless.
//! Results obtained with it are labelled as such.
//!
//! Files are relative to the package root and stay inside it
//! (@ref PackagePath.hpp).
#pragma once

#include "Robotik/Assets/AssetResolver.hpp"
#include "Robotik/Planning/Pddl.hpp"
#include "Robotik/Runtime/Faults.hpp"
#include "Robotik/Scenario/Assertions.hpp"
#include "Robotik/Scenario/Config.hpp"
#include "Robotik/Scenario/Trajectory.hpp"
#include "Robotik/Scenario/World.hpp"

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief The two files of a scenario, still editable (see @ref ConfigNode).
// ****************************************************************************
struct ScenarioSource
{
    //!< Root of the scenario file (holds @c scenario:).
    ConfigNode scenario;
    std::filesystem::path scenario_file;
    //!< Package root: the folder of @c plugin.yaml. File names start there.
    std::filesystem::path package;
    //!< Root of the world file (holds @c world:).
    ConfigNode world;
    std::filesystem::path world_file;

    //! @brief Reads the scenario file and the world file it names.
    //! @throws std::runtime_error when a file is missing or unreadable.
    [[nodiscard]] static ScenarioSource load(std::filesystem::path const& p_scenario);

    // -------------------------------------------------------------------------
    //! @brief Changes one value before interpretation. @p_path starts with
    //! @c scenario. or @c world., e.g. @c world.entities.easel.friction.
    //! @throws std::runtime_error for another prefix or a broken path.
    // -------------------------------------------------------------------------
    void override(std::string_view p_path, ConfigNode p_value);
};

// ****************************************************************************
//! @brief The strategy driving the robot. Empty: the mission drives it.
// ****************************************************************************
struct PlannerSpec
{
    //!< PDDL domain of the plugin: types, predicates, actions.
    std::filesystem::path domain;
    //!< What cannot be observed and does not change: compatible(vacuum, cube).
    std::vector<Literal> facts;
    //!< Conjunction to reach.
    std::vector<Literal> goal;
    //!< Goal in PDDL, for what a list of literals cannot say (forall...).
    //!< Replaces @ref goal; not checked against the beliefs.
    std::string goal_pddl;
    //!< New plans allowed after the first one (failure, effect not seen).
    std::uint32_t max_replans = 3u;
    //!< Simulated time waited before a new plan.
    Seconds replan_delay{ 0.5 };
    //!< Checks the effects of each action on the beliefs once it is done.
    bool verify = true;
    //!< Search preset of the planner.
    std::string search = "lama";
    //!< Wall-clock safety net of one search (s).
    Seconds timeout{ 10.0 };
};

// ****************************************************************************
//! @brief The strategy driving the robot: a behavior tree, a planner goal,
//! or neither (the mission drives it).
//!
//! @code
//! controller:
//!   planner:
//!     domain: scenarios/pick_and_place.domain.pddl
//!     facts:                              # block lists: a flow list [a(x, y)]
//!       - compatible(vacuum, cube)        # would split at the comma
//!     goal:
//!       - in(red_cube, box)               # or goal_pddl: "(forall ...)"
//!     replan: { max: 3, delay: 0.5 }      # new plans, simulated seconds before each
//!     verify: true                        # check the effects on the beliefs
//!     search: lama                        # planner search preset
//!     timeout: 10                         # wall-clock safety net (s)
//! @endcode
//!
//! The objects of the problem are the world entities with a @c kind; the
//! state comes from the observers of the plugin (@ref PlanningDomain).
// ****************************************************************************
struct ControllerSpec
{
    std::filesystem::path behavior_tree;
    std::optional<PlannerSpec> planner;
};

// ****************************************************************************
//! @brief When the ground truth may stand in for perception.
// ****************************************************************************
enum class OraclePolicy : std::uint8_t
{
    Never,
    WhenBlind,
    Always,
};

// ****************************************************************************
//! @brief Interpreted scenario.
// ****************************************************************************
struct Scenario
{
    //! @throws std::runtime_error naming the file and the field at fault, or
    //! a model missing from @p_assets.
    [[nodiscard]] static Scenario read(ScenarioSource const& p_source,
                                       AssetResolver const& p_assets);

    [[nodiscard]] static Scenario load(std::filesystem::path const& p_path,
                                       AssetResolver const& p_assets)
    {
        return read(ScenarioSource::load(p_path), p_assets);
    }

    std::string id;
    std::string description;
    std::filesystem::path file;
    //!< Master seed of the run; every random stream derives from it.
    std::uint64_t seed = 0;
    WorldDescription world;
    ControllerSpec controller;
    //!< Paths named by the scenario, read from their own files.
    std::map<std::string, Trajectory, std::less<>> trajectories;
    std::vector<Fault> faults;
    std::vector<RandomFault> random_faults;
    OraclePolicy oracle = OraclePolicy::Never;
    //!< Simulated time limit; zero means none.
    Seconds timeout{};
    //!< Condition ending the run as soon as it holds; empty for none.
    std::string stop_when;
    std::vector<AssertionSpec> assertions;
    //!< Values the mission reads (gains, model choices...). An experiment
    //!< overrides them as scenario.parameters.<name>.
    ConfigNode parameters;
    //!< Packs the scenario was resolved with; missions resolve their own
    //!< models with it.
    AssetResolver assets;
};

} // namespace robotik
