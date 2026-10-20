// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file CellDescription.hpp
/// Everything needed to build a robot cell, written in C++: the robots and
/// objects, the physics level, the controller, the faults.
///
/// A *cell* is the place where robots work (the word industry uses for a
/// robotised work station). This description is plain data: a program fills it
/// in a few lines, or the Package layer reads it from YAML files (a world file
/// and a scenario file). A @ref robotik::Cell is then built from it.
///
/// Frames are Z-up, in metres and radians. A tool that is not attached is in
/// storage: it is not built, but a host may mount it instead of another.
///
/// @par Contract
/// - Promises: plain data and free functions; nothing here reads a file.
/// - Expects: entity ids are unique in the world; a robot named by a device or
///   a skill exists.
/// - Errors: @c mountTool returns a @c Result; building a @ref robotik::Cell
///   from a wrong description throws (see @ref robotik::Cell).
#pragma once

#include "Robotik/Common/Config.hpp"
#include "Robotik/Common/Math.hpp"
#include "Robotik/Common/Result.hpp"
#include "Robotik/Robot/Assets/AssetResolver.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Robot/Sensors/Camera.hpp"
#include "Robotik/Robot/Sensors/ForceTorqueSensor.hpp"
#include "Robotik/Robot/Sensors/Imu.hpp"
#include "Robotik/Robot/Sensors/RangeScanner.hpp"
#include "Robotik/Task/Faults.hpp"
#include "Robotik/Task/Planning/Pddl.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

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
    std::array<float, 3> color{ 0.8F, 0.8F, 0.8F };
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
/// What physically exists at the start of a run: robots, tools, bodies and
/// props, where they are and what they hang from.
///
/// Read from a world file by @c loadWorld (Package layer), or built in C++.
///
/// @code
/// robotik::WorldDescription world;
/// world.entities.push_back({ .id = "arm", .type = robotik::EntitySpec::Type::Robot,
///                            .model = "arm.urdf" });
/// robotik::EntitySpec const& robot = robotik::firstRobot(world);
/// (void)robotik::mountTool(world, "gripper");  // swap the tool of the first robot
/// @endcode
// ****************************************************************************
struct WorldDescription
{
    /// World id.
    std::string id;
    /// A floor plane is built.
    bool floor = false;
    /// Sliding friction of the floor.
    double floor_friction = 1.0;
    /// Every entity, in file order.
    std::vector<EntitySpec> entities;
};

// ----------------------------------------------------------------------------
/// @param p_world World to search.
/// @param p_id Entity id.
/// @return The entity, or null.
// ----------------------------------------------------------------------------
[[nodiscard]] EntitySpec const* findEntity(WorldDescription const& p_world, std::string_view p_id);

// ----------------------------------------------------------------------------
/// @param p_world A world with one robot at least (the readers check it).
/// @return The first robot entity.
// ----------------------------------------------------------------------------
[[nodiscard]] EntitySpec const& firstRobot(WorldDescription const& p_world);

// ----------------------------------------------------------------------------
/// @param p_world World to search.
/// @return Every robot entity, in file order.
// ----------------------------------------------------------------------------
[[nodiscard]] std::vector<EntitySpec const*> robotsOf(WorldDescription const& p_world);

// ----------------------------------------------------------------------------
/// @param p_world World to search.
/// @param p_robot Robot id; empty for the first robot.
/// @return Tool attached to the flange of that robot, or null.
// ----------------------------------------------------------------------------
[[nodiscard]] EntitySpec const* mountedTool(WorldDescription const& p_world,
                                            std::string_view p_robot = {});

// ----------------------------------------------------------------------------
/// Changes the tool of a robot. The previous tool of that robot goes back to
/// storage.
/// @param p_world World to change.
/// @param p_id Tool id; empty removes the tool.
/// @param p_robot Robot id; empty for the first robot.
/// @return Fails for an unknown tool or robot.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<void> mountTool(WorldDescription& p_world,
                                     std::string_view p_id,
                                     std::string_view p_robot = {});

// ****************************************************************************
/// A path the robot follows: tool poses in a frame, or joint values.
///
/// A cartesian trajectory fills @ref frame and @ref poses; a joint
/// trajectory fills @ref joints and @ref configurations. The other fields
/// stay empty.
///
/// @code
/// auto const path = robotik::loadTrajectory(file); // Package layer
/// if (path.type == robotik::Trajectory::Type::Joint)
///     for (auto const& q : path.configurations)
///         follow(path.joints, q);
/// @endcode
// ****************************************************************************
struct Trajectory
{
    // ------------------------------------------------------------------------
    /// What a point of the trajectory describes.
    // ------------------------------------------------------------------------
    enum class Type : std::uint8_t
    {
        Cartesian, ///< Tool poses in a frame.
        Joint,     ///< Joint values.
    };


    /// File the trajectory was read from.
    std::filesystem::path file;
    /// Cartesian or joint trajectory.
    Type type = Type::Cartesian;
    /// Frame of the cartesian points (entity or entity.link); empty: world.
    std::string frame;
    /// Cartesian points, in @ref frame.
    std::vector<Pose> poses;
    /// Joint names of a joint trajectory.
    std::vector<std::string> joints;
    /// One configuration per point, aligned with @ref joints.
    std::vector<std::vector<double>> configurations;
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
    std::uint32_t max_replans = 3U;
    /// Simulated time waited before a new plan.
    Seconds replan_delay{ 0.5 };
    /// Checks the effects of each action on the beliefs once it is done.
    bool verify = true;
    /// With @ref verify — an effect counts only when the beliefs confirm it
    /// with an observation made after the step started (@c verify: observed).
    /// Otherwise an effect the beliefs cannot tell (unknown) is accepted.
    bool observed = false;
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
///     replan: { max: 3, delay: 0.5 }      # new plans; simulated s before each
///     verify: true                        # check the effects on the beliefs
///                                         # (observed: only fresh observations)
///     parallel: true                      # independent steps run together
///     search: lama                        # planner search preset
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
/// Where the robot's beliefs about an object come from (@c perception:).
///
/// Giving the robot the true pose of an object is not cheating when it is
/// chosen, said and recorded: like a simpler physics, it lets a test focus
/// on what it checks. A planning test can skip image processing; a car
/// simulator can hand over the lane marks and the pedestrians instead of
/// running vision on photo-realistic images.
///
/// @code{.yaml}
/// perception:
///   fidelity: sensed          # truth | noisy_truth | sensed (default)
///   overrides: { red_cube: truth }
///   blind: truth              # when no camera has an image (a headless run)
///   noise: { sigma: 0.005, dropout: 0.1, range: 2.0 }   # for noisy_truth
///   max_age: 2.0              # older beliefs are unknown to the planner (s)
/// @endcode
// ****************************************************************************
enum class PerceptionFidelity : std::uint8_t
{
    /// The perception pipeline (detectors on camera images) fills the
    /// beliefs.
    Sensed,
    /// The simulator writes the true position, degraded by
    /// @ref PerceptionSpec::noise (noise, missed observations, range).
    NoisyTruth,
    /// The simulator writes the true position (a mock of perception).
    Truth,
};

// ----------------------------------------------------------------------------
/// @param p_fidelity Level to name.
/// @return "sensed", "noisy_truth" or "truth".
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(PerceptionFidelity p_fidelity);

// ****************************************************************************
/// How the true position is degraded at the @c noisy_truth level.
// ****************************************************************************
struct TruthNoise
{
    /// Gaussian noise on each axis (m).
    double sigma = 0.0;
    /// Chance that an object is not seen at a step, in [0, 1].
    double dropout = 0.0;
    /// Objects farther than this from every robot base are not seen (m).
    double range = std::numeric_limits<double>::infinity();
};

// ****************************************************************************
/// Section @c perception: of a scenario (see @ref PerceptionFidelity).
// ****************************************************************************
struct PerceptionSpec
{
    /// Level of every object not in @ref overrides.
    PerceptionFidelity fidelity = PerceptionFidelity::Sensed;
    /// Level per world entity id.
    std::map<std::string, PerceptionFidelity, std::less<>> overrides;
    /// Level of the @c sensed objects when no camera has an image source (a
    /// headless run); @c Sensed keeps them unseen.
    PerceptionFidelity blind = PerceptionFidelity::Sensed;
    /// Degradation of the @c noisy_truth level.
    TruthNoise noise;
    /// A belief older than this is unknown to the planner; infinite by
    /// default.
    Seconds max_age{ std::numeric_limits<double>::infinity() };
};

// ----------------------------------------------------------------------------
/// @param p_perception Perception section.
/// @param p_id Entity id.
/// @param p_blind True when no camera has an image source.
/// @return The level of that entity.
// ----------------------------------------------------------------------------
[[nodiscard]] PerceptionFidelity fidelityOf(PerceptionSpec const& p_perception,
                                            std::string_view p_id,
                                            bool p_blind);

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
};

// -------------------------------------------------------------------------
/// @param p_id Entity or actuator id.
/// @return Its level.
// -------------------------------------------------------------------------
[[nodiscard]] inline Fidelity fidelityOf(PhysicsSpec const& p_physics, std::string_view p_id)
{
    auto const found = p_physics.overrides.find(p_id);
    return found == p_physics.overrides.end() ? p_physics.fidelity : found->second;
}

// -------------------------------------------------------------------------
/// @param p_a Id of one side.
/// @param p_b Id of the other side.
/// @return Level of an interaction: the most detailed of its two sides.
// -------------------------------------------------------------------------
[[nodiscard]] inline Fidelity fidelityBetween(PhysicsSpec const& p_physics, std::string_view p_a,
                                   std::string_view p_b)
{
    return std::max(fidelityOf(p_physics, p_a), fidelityOf(p_physics, p_b));
}

// ****************************************************************************
/// Everything a @ref Cell is built from.
///
/// @code
/// robotik::CellDescription cell;
/// cell.id = "demo";
/// cell.world.entities.push_back({ .id = "arm", .type = robotik::EntitySpec::Type::Robot,
///                                 .model = "arm.urdf" });
/// cell.controller.behavior_tree = "pick.bt.yaml";
/// cell.faults.push_back({ Seconds(3.0), "camera", true });
/// @endcode
// ****************************************************************************
struct CellDescription
{
    /// Name of the cell (a scenario id), used in reports and PDDL problems.
    std::string id;
    /// Master seed of the run; every random stream derives from it.
    std::uint64_t seed = 0;
    /// The robots and objects.
    WorldDescription world;
    /// Behavior tree, planner goal, or neither.
    ControllerSpec controller;
    /// Paths the mission can follow, by name.
    std::map<std::string, Trajectory, std::less<>> trajectories;
    /// Faults at fixed times.
    std::vector<Fault> faults;
    /// Faults drawn at random.
    std::vector<RandomFault> random_faults;
    /// Where the beliefs about each object come from.
    PerceptionSpec perception;
    /// Fidelity of the physics, per entity.
    PhysicsSpec physics;
    /// Values the mission reads (gains, model choices...).
    ConfigNode parameters;
    /// Packs the models were resolved with; missions resolve their own
    /// models with it.
    AssetResolver assets;
};

} // namespace robotik
