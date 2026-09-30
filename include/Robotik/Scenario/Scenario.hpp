// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file Scenario.hpp
 * @brief YAML simulation scenario: world, behavior tree path, and assertions.
 */

#pragma once

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/ECS/PerceptionComponents.hpp"

#include <array>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotik
{

/**
 * @brief Parsed content of a scenario YAML file.
 *
 * Relative paths (@c robot.model, @c execute.behavior_tree) are resolved
 * against the directory containing the scenario file when loaded via @ref load.
 *
 * @example
 * @code
 * robotik::Scenario s = robotik::Scenario::load("data/scenarios/pick_and_place.yml");
 * // s.robot_model, s.home, s.objects, s.behavior_tree, s.asserts
 * @endcode
 */
struct Scenario
{
    /** @brief One manipulable object to spawn under the robot root. */
    struct Object
    {
        /** @brief Shape, color, and name for ECS and rendering. */
        ecs::SceneObject shape;

        /** @brief Object center in robot base frame, meters (X, Y, Z). */
        std::array<float, 3> position{ 0.0f, 0.0f, 0.0f };
    };

    /** @brief Optional wrist or fixed camera mount. */
    struct Camera
    {
        /** @brief URDF link name to parent the camera entity. */
        std::string link;

        /** @brief Camera origin in link frame, meters. */
        std::array<float, 3> position{ 0.0f, 0.0f, 0.0f };

        /** @brief Resolution and FOV copied to @ref ecs::CameraSensor. */
        ecs::CameraSensor sensor;
    };

    /** @brief Short scenario id from YAML @c scenario:. */
    std::string name;

    /** @brief Natural-language task description for the UI. */
    std::string task;

    /** @brief Absolute or resolved path to the robot URDF. */
    std::string robot_model;

    /** @brief Named joint positions for initial @ref RobotRuntime::hold. */
    std::unordered_map<std::string, double> home;

    /** @brief Suction cup length along tool Z for @ref ecs::VacuumGripper. */
    double tool_length = 0.06;

    /** @brief Present when @c world.robot.camera is set in YAML. */
    std::optional<Camera> camera;

    /** @brief Objects from @c world.objects. */
    std::vector<Object> objects;

    /** @brief Resolved path to the behavior tree YAML. */
    std::string behavior_tree;

    /** @brief Assertion expressions evaluated by @ref Simulation::checks. */
    std::vector<std::string> asserts;

    /**
     * @brief Parses a scenario file.
     * @param p_path Path to scenario YAML.
     * @return Filled @ref Scenario.
     * @throws std::runtime_error if the file or required fields are invalid.
     */
    [[nodiscard]] static Scenario load(std::string const& p_path);
};

} // namespace robotik
