// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file World.hpp
/// What physically exists at the start of an experiment.
///
/// A world file lists entities. The URDF of a model describes its structure;
/// the world says which instance is present, where, and what it hangs from.
///
/// @code
/// world:
///   id: painting_lab
///   floor: { friction: 0.8 }
///   entities:
///     - id: robot
///       type: robot                       # the controlled robot (one)
///       model: robotik.robots.robot_6axis
///       home: { joint2: 0.3, joint3: 1.3, joint5: 1.54 }
///       sensors:
///         wrist_camera: { type: camera, parent: link6, fov: 70 }
///       actuators:
///         arm: { type: joint_group }
///     - id: brush
///       type: tool                        # keeps its identity once mounted
///       model: robotik.tools.default
///       attach: { parent: robot.flange, frame: tool_mount }
///     - id: easel
///       type: body                        # simulated rigid body
///       model: robotik.painter.easel
///       physics: free                     # free | fixed
///       pose: { position: [0.65, 0.0, 0.0], rpy: [0.0, 0.0, 3.14159] }
///     - id: canvas
///       type: body
///       shape: { type: box, size: [0.02, 0.40, 0.50] }
///       mass: 0.5
///       attach:
///         parent: easel.canvas_mount
///         transform: { position: [0.011, 0.0, 0.251] }
///     - id: red_cube
///       type: prop                        # moved by the grasp system only
///       shape: { type: box, size: [0.04, 0.04, 0.04] }
///       pose: { position: [0.40, 0.20, 0.02] }
///       randomize: { x: [-0.02, 0.02], y: [-0.02, 0.02] }
/// @endcode
///
/// Frames are Z-up, in metres and radians. A tool that is not attached is in
/// storage: it is not built, but a host may mount it instead of another.
#pragma once

#include "Robotik/Package/Assets/AssetResolver.hpp"
#include "Robotik/Common/Math.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Robot/Sensors/Camera.hpp"
#include "Robotik/Robot/Sensors/ForceTorqueSensor.hpp"
#include "Robotik/Robot/Sensors/Imu.hpp"
#include "Robotik/Robot/Sensors/RangeScanner.hpp"

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
/// A sensor carried by a robot or a tool.
///
/// @code
/// sensors:
///   wrist_camera: { type: camera, parent: link6, position: [0.05, 0, 0],
///                   fov: 70, resolution: [320, 240], frequency: 15, noise:
///                   0.01 }
///   imu: { type: imu, parent: base_link, frequency: 200, latency: 0.002,
///          accelerometer: { sigma: 0.02, bias: 0.05, saturation: [-160, 160]
///          }, gyroscope: 0.001 }
///   lidar: { type: range, parent: base_link, beams: 180, fov: 360,
///            max_range: 10, frequency: 10, noise: { sigma: 0.01, resolution:
///            0.001 } }
///   wrist_ft: { type: force_torque, link: link6, frequency: 500,
///               force: 0.5, torque: 0.02 }
/// @endcode
/// A noise channel is a standard deviation, or a @ref NoiseModel map.
/// Angles are in degrees (fov), times in seconds, lengths in metres.
// ****************************************************************************
struct SensorSpec
{
    // ------------------------------------------------------------------------
    /// Kind of sensor (@c type: in the YAML).
    // ------------------------------------------------------------------------
    enum class Type : std::uint8_t
    {
        Camera,      ///< @c camera: @ref Camera.
        Imu,         ///< @c imu: @ref Imu.
        Range,       ///< @c range: @ref RangeScanner.
        ForceTorque, ///< @c force_torque: @ref ForceTorqueSensor.
    };

    /// Sensor name, also its resource name.
    std::string name;
    /// Kind of sensor; only the matching config below is used.
    Type type = Type::Camera;
    /// Configuration of a camera.
    CameraConfig camera;
    /// Configuration of an IMU.
    ImuConfig imu;
    /// Configuration of a range scanner.
    RangeScannerConfig range;
    /// Configuration of a force/torque sensor.
    ForceTorqueConfig force_torque;
};

// ****************************************************************************
/// An actuator of a robot or a tool; also the name of its resource.
///
/// @code
/// actuators:
///   arm: { type: joint_group, joints: [joint1, joint2, joint3] }
///   left_motor: { type: motor, joint: left_wheel_joint }
///   vacuum: { type: vacuum, parent: tool_tip, length: 0.02 }
/// @endcode
// ****************************************************************************
struct ActuatorSpec
{
    // ------------------------------------------------------------------------
    /// Kind of actuator (@c type: in the YAML).
    // ------------------------------------------------------------------------
    enum class Type : std::uint8_t
    {
        JointGroup, ///< @ref JointGroup.
        Motor,      ///< @ref Motor.
        Vacuum,     ///< @ref VacuumGripper.
    };

    /// Actuator name, also its resource name.
    std::string name;
    /// Kind of actuator.
    Type type = Type::JointGroup;
    /// Joint group members (empty: all) or the motor joint.
    std::vector<std::string> joints;
    /// Vacuum cup link (empty: the robot work frame).
    std::string link;
    /// Flange to cup tip distance of a vacuum gripper.
    Length length{ 0.0 };
};

// ****************************************************************************
/// Primitive geometry of a body or a prop.
///
/// @code
/// shape: { type: container, size: [0.20, 0.15, 0.08] }
/// color: [0.2, 0.4, 0.9]          # on the entity, next to shape
/// @endcode
// ****************************************************************************
struct ShapeSpec
{
    enum class Type : std::uint8_t
    {
        Box,       ///< Solid cuboid.
        Container, ///< Open-top bin: floor slab and four walls.
    };

    /// Box or container.
    Type type = Type::Box;
    /// Full extents along X, Y, Z (m).
    Vector3 size = Vector3(0.04, 0.04, 0.04);
    /// RGB color in [0, 1].
    std::array<float, 3> color{ 0.8f, 0.8f, 0.8f };
};

// ****************************************************************************
/// Placement of an entity relative to a frame of another one.
///
/// @code
/// attach: { parent: robot.flange, frame: tool_mount }   # parent "robot",
/// parent_frame "flange" attach: { parent: easel.canvas_mount, transform: {
/// position: [0.011, 0, 0.251] } }
/// @endcode
// ****************************************************************************
struct Attachment
{
    /// Entity id.
    std::string parent;
    /// Link of the parent (empty: its origin).
    std::string parent_frame;
    /// Own link that lands on the parent frame (empty: own origin).
    std::string frame;
    /// Offset of @ref frame in the parent frame.
    Pose transform{ Vector3::ZERO, Quaternion{} };
};

// ****************************************************************************
/// One entity of the world (see the file comment for the YAML).
///
/// @code
/// for (robotik::EntitySpec const& e : world.entities)
///     if (e.type == robotik::EntitySpec::Type::Prop)
///         std::cout << e.id << " at " << e.pose.position.x << '\n';
/// @endcode
// ****************************************************************************
struct EntitySpec
{
    // ------------------------------------------------------------------------
    /// Role of the entity (@c type: in the YAML).
    // ------------------------------------------------------------------------
    enum class Type : std::uint8_t
    {
        Robot, ///< The controlled robot.
        Tool,  ///< Hangs from the robot flange by its tool_mount link.
        Body,  ///< Rigid body simulated by the physics backend.
        Prop,  ///< Kinematic object: still unless the grasp system moves it.
    };

    // ------------------------------------------------------------------------
    /// How the physics backend treats the entity (@c physics:).
    // ------------------------------------------------------------------------
    enum class Physics : std::uint8_t
    {
        None,  ///< Not in the physics backend.
        Fixed, ///< Collides, never moves.
        Free,  ///< Six degrees of freedom.
    };

    /// Entity id, unique in the world.
    std::string id;
    /// Role of the entity.
    Type type = Type::Prop;
    /// Type of the entity for the planner (a type of the PDDL domain), or
    /// empty: the planner does not see it.
    std::string kind;
    /// Asset id or path, as written.
    std::string model_ref;
    /// URDF file, empty for a primitive.
    std::filesystem::path model;
    /// Primitive geometry, when there is no model.
    std::optional<ShapeSpec> shape;
    /// World pose; ignored when @ref attach is set.
    Pose pose{ Vector3::ZERO, Quaternion{} };
    /// Where it hangs from, or nothing (placed by @ref pose).
    std::optional<Attachment> attach;
    /// How the physics backend treats it.
    Physics physics = Physics::None;
    /// Mass of a primitive body (kg).
    double mass = 1.0;
    /// Sliding friction; negative keeps the URDF value.
    double friction = -1.0;
    /// Uniform offsets [min, max] per axis (m), drawn at each reset.
    std::array<std::array<double, 2>, 3> randomize{};

    // --- Robot and tool
    // -------------------------------------------------------
    /// Joint positions held at reset (robot).
    JointPosture home;
    /// Sensors carried.
    std::vector<SensorSpec> sensors;
    /// Actuators carried.
    std::vector<ActuatorSpec> actuators;
    /// Entities a tool works on (a drill on its workpiece): its contacts
    /// with them are not collisions.
    std::vector<std::string> touches;
};

// ****************************************************************************
/// Parsed world file.
///
/// @code
/// auto world = robotik::WorldDescription::load("worlds/arm.world.yaml",
/// assets); robotik::EntitySpec const& robot = world.robot(); if
/// (robotik::EntitySpec const* box = world.find("box"))
///     std::cout << box->id << '\n';
/// world.mount("gripper");  // swap the tool of the first robot
/// @endcode
// ****************************************************************************
struct WorldDescription
{
    // -------------------------------------------------------------------------
    /// Reads the @c world: node of a file.
    /// @param p_world The @c world: node.
    /// @param p_directory Package root; file names are relative to it.
    /// @param p_assets Resolves the model names.
    /// @return The world.
    /// @throws std::runtime_error naming the entity and the field at fault.
    // -------------------------------------------------------------------------
    [[nodiscard]] static WorldDescription
    read(ConfigNode const& p_world,
         std::filesystem::path const& p_directory,
         AssetResolver const& p_assets);

    // -------------------------------------------------------------------------
    /// Reads a world file (its root holds @c world:).
    /// @param p_path A @c x.world.yaml file.
    /// @param p_assets Resolves the model names.
    /// @return The world.
    /// @throws std::runtime_error naming the file and the field at fault.
    // -------------------------------------------------------------------------
    [[nodiscard]] static WorldDescription
    load(std::filesystem::path const& p_path, AssetResolver const& p_assets);

    /// World id.
    std::string id;
    /// A floor plane is built.
    bool floor = false;
    /// Sliding friction of the floor.
    double floor_friction = 1.0;
    /// Every entity, in file order.
    std::vector<EntitySpec> entities;

    // -------------------------------------------------------------------------
    /// @param p_id Entity id.
    /// @return The entity, or null.
    // -------------------------------------------------------------------------
    [[nodiscard]] EntitySpec const* find(std::string_view p_id) const;

    /// @return The first robot entity (validated to exist).
    [[nodiscard]] EntitySpec const& robot() const;

    /// @return Every robot entity, in file order (one at least).
    [[nodiscard]] std::vector<EntitySpec const*> robots() const;

    /// @return Tool attached to the flange of the first robot, or null.
    [[nodiscard]] EntitySpec const* mountedTool() const;

    /// @return Tool attached to the flange of robot @p_robot, or null.
    [[nodiscard]] EntitySpec const* mountedTool(std::string_view p_robot) const;

    // -------------------------------------------------------------------------
    /// Changes the tool of a robot. The previous tool of that robot goes back
    /// to storage.
    /// @param p_id Tool id; empty removes the tool.
    /// @param p_robot Robot id; empty for the first robot.
    /// @throws std::invalid_argument for an unknown tool or robot.
    // -------------------------------------------------------------------------
    void mount(std::string_view p_id, std::string_view p_robot = {});
};

} // namespace robotik
