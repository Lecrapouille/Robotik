// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Scenario.hpp
//! @brief Declarative mission file: robot, world layout, behavior tree, checks.
//!
//! A <em>scenario</em> is the single YAML entry point for a pick-and-place (or
//! similar) demo. It does not run physics itself: @ref Scenario::load parses
//! the file into this struct; @ref Simulation constructs @ref RobotRuntime,
//! spawns objects, registers skills, and loads the BlackThorn tree referenced
//! by @ref behavior_tree. Relative paths (@c robot.model, @c execute.behavior_tree)
//! are resolved against the scenario file directory.
//!
//! See @c doc2/Scenario-et-Simulation.md for the full schema and workflow.
#pragma once

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/ECS/PerceptionComponents.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief In-memory result of @ref load — everything @ref Simulation needs
//! before the first @ref RobotRuntime::step.
//!
//! Typical YAML layout:
//! @li @c scenario — short id (@ref name).
//! @li @c world.robot — URDF path, @ref home posture, optional @ref camera and
//!     @ref tool_length for @ref ecs::VacuumGripper.
//! @li @c world.objects — props to parent under the robot root (@ref objects).
//! @li @c execute — human @ref task string and path to the BT YAML (@ref
//!     behavior_tree).
//! @li @c assert — strings evaluated after the run by @ref Simulation::checks.
//!
//! @example
//! @code
//! robotik::Scenario s = robotik::Scenario::load(
//!     std::filesystem::path("data/scenarios/pick_and_place.yml"));
//! robotik::Simulation sim(world, &scene, std::move(s));
//! @endcode
// ****************************************************************************
struct Scenario
{
    // -------------------------------------------------------------------------
    //! @brief Parses a scenario file.
    //! @param p_path Path to scenario YAML.
    //! @return Filled @ref Scenario.
    //! @throws std::runtime_error if the file or required fields are invalid.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Scenario load(std::filesystem::path const& p_path);

    // ---------------------------------------------------------------------
    //! @brief One manipulable object to spawn under the robot root.
    // ---------------------------------------------------------------------
    struct Object
    {
        //!< Shape, color, and name for ECS and rendering.
        ecs::SceneObject shape;
        //!< Object center in robot base frame, meters (X, Y, Z).
        std::array<float, 3> position{ 0.0f, 0.0f, 0.0f };
    };

    // ---------------------------------------------------------------------
    //! @brief Optional wrist or fixed camera mount.
    // ---------------------------------------------------------------------
    struct Camera
    {
        //!< URDF link name to parent the camera entity.
        std::string link;
        //!< Camera origin in link frame, meters.
        std::array<float, 3> position{ 0.0f, 0.0f, 0.0f };
        //!< Resolution and FOV copied to @ref ecs::CameraSensor.
        ecs::CameraSensor sensor;
    };

    //!< Short scenario id from YAML @c scenario:.
    std::string name;
    //!< Natural-language task description for the UI.
    std::string task;
    //!< Absolute or resolved path to the robot URDF.
    std::filesystem::path robot_model;
    //!< Named joint positions for initial @ref RobotRuntime::hold (YAML: rad).
    JointPosture home;
    //!< Suction cup length along tool Z for @ref ecs::VacuumGripper (SI: m).
    Length tool_length{ 0.06 };
    //!< Present when @c world.robot.camera is set in YAML.
    std::optional<Camera> camera;
    //!< Objects from @c world.objects.
    std::vector<Object> objects;
    //!< Resolved path to the behavior tree YAML.
    std::filesystem::path behavior_tree;
    //!< Assertion expressions evaluated by @ref Simulation::checks.
    std::vector<std::string> asserts;
};

} // namespace robotik
