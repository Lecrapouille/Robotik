// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file World.hpp
//! @brief What physically exists at the start of an experiment.
//!
//! A world file lists entities. The URDF of a model describes its structure;
//! the world says which instance is present, where, and what it hangs from.
//!
//! @code
//! world:
//!   id: painting_lab
//!   floor: { friction: 0.8 }
//!   entities:
//!     - id: robot
//!       type: robot                       # the controlled robot (one)
//!       model: robotik.robots.robot_6axis
//!       home: { joint2: 0.3, joint3: 1.3, joint5: 1.54 }
//!       sensors:
//!         wrist_camera: { type: camera, parent: link6, fov: 70 }
//!       actuators:
//!         arm: { type: joint_group }
//!     - id: brush
//!       type: tool                        # keeps its identity once mounted
//!       model: robotik.tools.default
//!       attach: { parent: robot.flange, frame: tool_mount }
//!     - id: easel
//!       type: body                        # simulated rigid body
//!       model: robotik.painter.easel
//!       physics: free                     # free | fixed
//!       pose: { position: [0.65, 0.0, 0.0], rpy: [0.0, 0.0, 3.14159] }
//!     - id: canvas
//!       type: body
//!       shape: { type: box, size: [0.02, 0.40, 0.50] }
//!       mass: 0.5
//!       attach:
//!         parent: easel.canvas_mount
//!         transform: { position: [0.011, 0.0, 0.251] }
//!     - id: red_cube
//!       type: prop                        # moved by the grasp system only
//!       shape: { type: box, size: [0.04, 0.04, 0.04] }
//!       pose: { position: [0.40, 0.20, 0.02] }
//!       randomize: { x: [-0.02, 0.02], y: [-0.02, 0.02] }
//! @endcode
//!
//! Frames are Z-up, in metres and radians. A tool that is not attached is in
//! storage: it is not built, but a host may mount it instead of another.
#pragma once

#include "Robotik/Assets/AssetResolver.hpp"
#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Sensors/Camera.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace robotik
{

class ConfigNode;

// ****************************************************************************
//! @brief A sensor carried by a robot or a tool.
// ****************************************************************************
struct SensorSpec
{
    std::string name;
    CameraConfig camera;
};

// ****************************************************************************
//! @brief An actuator of a robot or a tool; also the name of its resource.
// ****************************************************************************
struct ActuatorSpec
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
    //!< Vacuum cup link (empty: the robot work frame).
    std::string link;
    Length length{ 0.0 };
};

// ****************************************************************************
//! @brief Primitive geometry of a body or a prop.
// ****************************************************************************
struct ShapeSpec
{
    enum class Type : std::uint8_t
    {
        Box,       //!< Solid cuboid.
        Container, //!< Open-top bin: floor slab and four walls.
    };

    Type type = Type::Box;
    //!< Full extents along X, Y, Z (m).
    Vector3 size = Vector3(0.04, 0.04, 0.04);
    std::array<float, 3> color{ 0.8f, 0.8f, 0.8f };
};

// ****************************************************************************
//! @brief Placement of an entity relative to a frame of another one.
// ****************************************************************************
struct Attachment
{
    //!< Entity id.
    std::string parent;
    //!< Link of the parent (empty: its origin).
    std::string parent_frame;
    //!< Own link that lands on the parent frame (empty: own origin).
    std::string frame;
    //!< Offset of @ref frame in the parent frame.
    Pose transform{ zero3(), Quaternion{} };
};

// ****************************************************************************
//! @brief One entity of the world.
// ****************************************************************************
struct EntitySpec
{
    enum class Type : std::uint8_t
    {
        Robot, //!< The controlled robot.
        Tool,  //!< Hangs from the robot flange by its tool_mount link.
        Body,  //!< Rigid body simulated by the physics backend.
        Prop,  //!< Kinematic object: still unless the grasp system moves it.
    };

    enum class Physics : std::uint8_t
    {
        None,  //!< Not in the physics backend.
        Fixed, //!< Collides, never moves.
        Free,  //!< Six degrees of freedom.
    };

    std::string id;
    Type type = Type::Prop;
    //!< Asset id or path, as written.
    std::string model_ref;
    //!< URDF file, empty for a primitive.
    std::filesystem::path model;
    std::optional<ShapeSpec> shape;
    //!< World pose; ignored when @ref attach is set.
    Pose pose{ zero3(), Quaternion{} };
    std::optional<Attachment> attach;
    Physics physics = Physics::None;
    //!< Mass of a primitive body (kg).
    double mass = 1.0;
    //!< Sliding friction; negative keeps the URDF value.
    double friction = -1.0;
    //!< Uniform offsets [min, max] per axis (m), drawn at each reset.
    std::array<std::array<double, 2>, 3> randomize{};

    // --- Robot and tool -------------------------------------------------------
    //!< Joint positions held at reset (robot).
    JointPosture home;
    std::vector<SensorSpec> sensors;
    std::vector<ActuatorSpec> actuators;
};

// ****************************************************************************
//! @brief Parsed world file.
// ****************************************************************************
struct WorldDescription
{
    // -------------------------------------------------------------------------
    //! @brief Reads the @c world: node of a file. Paths are relative to
    //! @p_directory, models are resolved through @p_assets.
    //! @throws std::runtime_error naming the entity and the field at fault.
    // -------------------------------------------------------------------------
    [[nodiscard]] static WorldDescription read(ConfigNode const& p_world,
                                               std::filesystem::path const& p_directory,
                                               AssetResolver const& p_assets);

    //! @brief Reads a world file (its root holds @c world:).
    [[nodiscard]] static WorldDescription load(std::filesystem::path const& p_path,
                                               AssetResolver const& p_assets);

    std::string id;
    bool floor = false;
    double floor_friction = 1.0;
    std::vector<EntitySpec> entities;

    [[nodiscard]] EntitySpec const* find(std::string_view p_id) const;

    //! @brief The robot entity (validated to exist).
    [[nodiscard]] EntitySpec const& robot() const;

    //! @brief Tool attached to the robot flange, or null.
    [[nodiscard]] EntitySpec const* mountedTool() const;

    // -------------------------------------------------------------------------
    //! @brief Puts tool @p_id on the robot flange (empty: no tool). The
    //! previous tool goes back to storage.
    //! @throws std::invalid_argument for an unknown tool.
    // -------------------------------------------------------------------------
    void mount(std::string_view p_id);
};

} // namespace robotik
