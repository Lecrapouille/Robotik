// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Scenario.hpp
//! @brief Declarative mission file: robot, sensors, actuators, world layout,
//! randomization, faults, behavior tree and checks.
//!
//! @ref Scenario::load parses the YAML into plain data; @ref Simulation builds
//! and runs it. Relative paths are resolved against the scenario directory.
//! See @c doc/Scenario-et-Simulation.md for the schema.
#pragma once

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/Faults.hpp"
#include "Robotik/Sensors/Camera.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief In-memory scenario.
//!
//! @code
//! scenario: pick_and_place
//! seed: 42
//! robot:
//!   model: ../robot_6axis.urdf
//!   home: { joint2: 0.3, joint3: 1.3, joint5: 1.54 }
//!   sensors:
//!     wrist_camera: { type: camera, parent: link6, fov: 70, noise: 0.02 }
//!   actuators:
//!     arm: { type: joint_group }
//!     gripper: { type: vacuum, length: 0.06 }
//! world:
//!   objects:
//!     red_cube:
//!       type: cube
//!       position: [0.40, 0.20, 0.02]
//!       randomize: { x: [-0.03, 0.03], y: [-0.03, 0.03] }
//! faults:
//!   - { at: 4.0, resource: wrist_camera, action: disable }
//! @endcode
// ****************************************************************************
struct Scenario
{
    //! @throws std::runtime_error if the file or required fields are invalid.
    [[nodiscard]] static Scenario load(std::filesystem::path const& p_path);

    struct Camera
    {
        std::string name;
        CameraConfig config;
    };

    struct Actuator
    {
        enum class Type : std::uint8_t
        {
            JointGroup,
            Motor,
            Vacuum,
        };

        std::string name;
        Type type = Type::JointGroup;
        //!< Joint group members (empty: all) or the motor joint.
        std::vector<std::string> joints;
        //!< Vacuum flange link (empty: robot tool frame).
        std::string link;
        Length length{ 0.06 };
    };

    struct Object
    {
        ecs::SceneObject shape;
        //!< Nominal center in the robot base frame (m).
        Vector3 position;
        //!< Uniform offset ranges [min, max] per axis, drawn at each reset.
        std::array<std::array<double, 2>, 3> randomize{};
    };

    std::string name;
    std::string task;
    //!< Master seed of the run; every random stream derives from it.
    std::uint64_t seed = 0;
    std::filesystem::path robot_model;
    JointPosture home;
    std::vector<Camera> cameras;
    //!< Empty: an @c arm group of all joints and a vacuum @c gripper.
    std::vector<Actuator> actuators;
    std::vector<Object> objects;
    std::vector<Fault> faults;
    std::vector<RandomFault> random_faults;
    std::filesystem::path behavior_tree;
    std::vector<std::string> asserts;
};

} // namespace robotik
