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
//!   world: ../worlds/manipulation_cell.yaml
//!   seed: 42
//!   controller:
//!     behavior_tree: pick_and_place.bt.yml
//!   trajectories:                          # see Trajectory.hpp
//!     drop: trajectories/drop.yaml
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
//! Files are relative to the file that names them.
#pragma once

#include "Robotik/Assets/AssetResolver.hpp"
#include "Robotik/Runtime/Faults.hpp"
#include "Robotik/Scenario/Assertions.hpp"
#include "Robotik/Scenario/Config.hpp"
#include "Robotik/Scenario/Trajectory.hpp"
#include "Robotik/Scenario/World.hpp"

#include <cstdint>
#include <filesystem>
#include <map>
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
struct ControllerSpec
{
    std::filesystem::path behavior_tree;
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
