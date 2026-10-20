// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file MujocoBackend.hpp
/// The physics engine of RobotIK: MuJoCo computes gravity, contacts and the
/// motor forces of every robot and object of a world.
///
/// Each URDF loaded becomes its own chain of bodies in one shared MuJoCo model,
/// so robots and objects touch each other.
///
/// @par Contract
/// - Promises: one physics for every robot and body of a world; contacts,
///   welds and wrenches come from MuJoCo.
/// - Expects: chains are loaded before the first step; names are unique.
/// - Errors: a URDF that MuJoCo cannot read, or an unknown chain or link,
///   throws @c std::runtime_error when the cell is built.
#pragma once

#include "Robotik/Robot/Backends/RobotBackend.hpp"
#include "Robotik/Common/Math.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// How the robot is placed in the MuJoCo world.
///
/// @code
/// robotik::MujocoOptions options;
/// options.floating_base = true;            // a walking robot
/// options.floor = true;
/// options.friction["caster_link"] = 0.05;  // a ball that slides
/// options.armature["hip_joint"] = 0.01;    // rotor inertia of a servo
/// @endcode
// ****************************************************************************
struct MujocoOptions
{
    /// Free joint on the root link (mobile robots); false bolts the root
    /// link at the base pose of the robot.
    bool floating_base = false;
    /// Infinite ground plane at z = 0.
    bool floor = false;
    /// Sliding friction per link name (default 1): e.g. a caster ball.
    std::unordered_map<std::string, double> friction;
    /// Sliding friction of the ground plane when @ref floor is set.
    double floor_friction = 1.0;
    /// Physics step; the joint loops run at this rate.
    Seconds timestep{ 0.001 };
    /// Adds gravity and Coriolis (@c qfrc_bias) to the actuated joints, as
    /// an industrial drive does. A legged robot runs by a learned policy
    /// turns it off: the policy was trained without it.
    bool gravity_compensation = true;
    /// Reflected rotor inertia per joint name (kg.m²): rotor inertia times
    /// the gear ratio squared. URDF cannot carry it. Values below the default
    /// rotor floor are raised, unless @ref exact_inertia is set.
    std::unordered_map<std::string, double> armature;
    /// Viscous joint damping per joint name (N·m·s/rad). Used only when
    /// @ref exact_inertia is set.
    std::unordered_map<std::string, double> damping;
    /// The joints of @ref armature and @ref damping keep these values, even
    /// below the default floors (a small servo). The floors stay for every
    /// other joint: they stand in for a URDF that declares no rotor inertia.
    bool exact_inertia = false;
};

// ****************************************************************************
/// How one loaded URDF chain is placed.
///
/// @code
/// robotik::ChainOptions box;
/// box.floating = true;
/// box.pose.position = robotik::Vector3(0.5, 0.0, 0.1);
/// box.prefix = "box_";
/// std::string const chain = physics.load("box.urdf", box);
/// @endcode
// ****************************************************************************
struct ChainOptions
{
    /// Free joint on the chain root. The robot chain stays fixed unless
    /// @ref MujocoOptions::floating_base is set.
    bool floating = false;
    /// World pose of the root link (URDF Z-up): its start pose when
    /// @ref floating, where it is bolted otherwise. Ignored for the first
    /// chain, which follows the robot base.
    Pose pose{ Vector3::ZERO, Quaternion{} };
    /// Sliding friction of every geom of the chain. Negative keeps the URDF.
    double friction = -1.0;
    /// Put before every MuJoCo name of the chain (bodies, joints, geoms,
    /// meshes), so two copies of a model can share the world. The robot and
    /// its tool use the same prefix; Robotik names stay local (see
    /// @ref MujocoBackend::join). Ignored for the first chain.
    std::string prefix;
};

// ****************************************************************************
/// One free box, not part of a URDF. Its origin is the centre.
///
/// @code
/// robotik::FreeBox cube;
/// cube.name = "red_cube";
/// cube.size = robotik::Vector3(0.04, 0.04, 0.04);
/// cube.mass = 0.05;
/// cube.pose.position = robotik::Vector3(0.4, 0.2, 0.02);
/// physics.addBox(cube);
/// @endcode
// ****************************************************************************
struct FreeBox
{
    /// Body name in MuJoCo, also the world entity id.
    std::string name;
    /// Full extents (m) along the box axes.
    Vector3 size = Vector3::ZERO;
    /// Mass (kg).
    double mass = 1.0;
    /// Diagonal inertia (kg·m²) at the centre, in the box frame.
    Vector3 inertia = Vector3::ZERO;
    /// Start pose in the world frame.
    Pose pose{ Vector3::ZERO, Quaternion{} };
    /// Sliding friction.
    double friction = 1.0;
    /// True: a static body (a wall), false: a free body.
    bool fixed = false;
};

// ****************************************************************************
/// Time-stepping physics backend built on MuJoCo.
///
/// A <em>backend</em> is a thin adapter around an external library that owns
/// the simulated bodies and exposes a stable API for the rest of the stack
/// (see @ref RobotBackend). Backends are not ECS components: the
/// @ref RobotSession holds them, next to the @ref PinocchioBackend that every
/// robot owns for analytical FK/IK. Skills talk to the robot; the robot
/// forwards work to its backends.
///
/// MuJoCo (Multi-Joint dynamics with Contact) integrates rigid-body motion,
/// actuators and contacts. This wrapper hides @c mjModel (constants) and
/// @c mjData (state) and maps Robotik joint names to MuJoCo indices. URDF
/// files missing inertial data are copied to a private temporary file with
/// defaults, so MuJoCo can load them without changing what Pinocchio or
/// Compages see. @ref load keeps each file as its own chain. @ref attach
/// hangs one chain on a link of another; @ref detach puts it back in the
/// world. The assembled spec is edited (free joint, floor, friction) then
/// compiled. Instances are independent: one per parallel environment.
///
/// @par MuJoCo state and indexing
/// @li @b qpos — generalized positions @c mjData.qpos. One scalar per entry
///     in the position vector (revolute/prismatic: angle or displacement in
///     rad or m). The free joint of a floating base uses seven consecutive
///     entries: position (m) then orientation quaternion (w, x, y, z).
/// @li @b qvel — generalized velocities @c mjData.qvel, aligned with @b DOFs.
///     Revolute/prismatic joints use one velocity each; the free joint uses
///     six: linear velocity in the world frame, angular velocity in the body
///     frame.
/// @li @b DOF (degree of freedom) — one independent motion axis in velocity
///     space (@c model.nv entries). Applied torques act on DOFs through
///     @c qfrc_applied. For 1-D joints the qvel index and the DOF index
///     coincide; multi-DOF joints (free, ball) do not share the @b qpos layout.
/// @li @c qfrc_applied — external generalized forces added each step: the
///     joint loop efforts (@ref JointSet::control) plus the gravity
///     compensation of the actuated joints (@c qfrc_bias), unless
///     @ref MujocoOptions::gravity_compensation is off.
/// @li @c ctrl — actuator commands @c mjData.ctrl, used instead of
///     @c qfrc_applied for joints that the URDF wires to a MuJoCo actuator.
/// @li @c mj_forward — recomputes kinematics and forces from the current
///     @b qpos / @b qvel without advancing time; used on @ref reset.
/// @li @c cfrc_int — interaction wrench between a body and its parent, read
///     by @ref wrench after @c mj_rnePostConstraint.
///
/// @code
/// auto physics = std::make_unique<robotik::MujocoBackend>(
///     robotik::MujocoOptions{ .floating_base = true, .floor = true });
/// std::string const arm = physics->load("robot_6axis.urdf");
/// std::string const tool = physics->load("tool_gripper.urdf");
/// physics->attach(arm, "flange", tool, "tool_mount");
/// robot.connect(std::move(physics));
/// @endcode
// ****************************************************************************
class MujocoBackend final: public RobotBackend
{
public:

    /// @param p_options Placement, floor, friction, timestep, inertia.
    explicit MujocoBackend(MujocoOptions p_options = {});
    ~MujocoBackend() override;

    // -------------------------------------------------------------------------
    /// Loads one URDF as its own kinematic chain.
    /// @param p_urdf URDF file.
    /// @param p_options Placement, friction and name prefix of the chain.
    /// @return The chain name (the URDF @c robot name, made unique if needed).
    /// @throws std::runtime_error if MuJoCo cannot parse the file.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string load(std::filesystem::path const& p_urdf,
                                   ChainOptions const& p_options = {});

    // -------------------------------------------------------------------------
    /// Adds a free box compiled with the chains. The origin is the centre.
    /// Call it before the backend is attached to a robot.
    /// @param p_box Name, size, mass and pose of the box.
    // -------------------------------------------------------------------------
    void addBox(FreeBox const& p_box);

    // -------------------------------------------------------------------------
    /// Declares a weld between two bodies, off, compiled with the chains:
    /// @ref weld turns it on. The two bodies never collide with each other.
    /// Call it before the backend is attached to a robot.
    /// @param p_name Weld name, given to @ref weld.
    /// @param p_body1 Body that holds.
    /// @param p_body2 Body that is held.
    // -------------------------------------------------------------------------
    void addWeld(std::string p_name, std::string p_body1, std::string p_body2);

    // -------------------------------------------------------------------------
    /// Makes two bodies never collide with each other. Call it before the
    /// backend is attached to a robot.
    /// @param p_body1 First body.
    /// @param p_body2 Second body.
    // -------------------------------------------------------------------------
    void excludeContacts(std::string p_body1, std::string p_body2);

    /// Turns a weld of @ref addWeld on or off (see @ref RobotBackend::weld).
    bool weld(std::string const& p_name, bool p_on) override;

    /// Moves a free body at rest (see @ref RobotBackend::placeBody).
    bool placeBody(std::string const& p_body, Pose const& p_pose) override;

    // -------------------------------------------------------------------------
    /// @param p_body Body name.
    /// @return World pose of the body after the last step or reset, or
    ///   nothing when unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<Pose>
    bodyPose(std::string const& p_body) const override;

    /// @return @c mjData.contact with the names of the bodies of both geoms.
    [[nodiscard]] std::vector<Contact> contactList() const override;

    // -------------------------------------------------------------------------
    /// Hangs @p_robot2 on @p_robot1 (a tool on an arm).
    ///
    /// @p_joint1 and @p_joint2 name a link, or a joint (the link that joint
    /// moves). The named link of @p_robot2 and every body under it become
    /// children of the named link of @p_robot1. The chain can be separated
    /// again with @ref detach.
    /// @param p_robot1 Chain that carries.
    /// @param p_joint1 Link or joint of @p_robot1 (e.g. @c flange).
    /// @param p_robot2 Chain that is carried.
    /// @param p_joint2 Link or joint of @p_robot2 (e.g. @c tool_mount).
    /// @throws std::runtime_error if a chain or a link is unknown, or if
    ///   @p_robot2 is already attached.
    // -------------------------------------------------------------------------
    void attach(std::string const& p_robot1,
                std::string const& p_joint1,
                std::string const& p_robot2,
                std::string const& p_joint2);

    // -------------------------------------------------------------------------
    /// Undoes @ref attach. @p_robot2 is a chain of its own again, standing
    /// in the world. The arguments are those given to @ref attach.
    /// @param p_robot1 Chain that carried.
    /// @param p_joint1 Link or joint of @p_robot1.
    /// @param p_robot2 Chain that was carried.
    /// @param p_joint2 Link or joint of @p_robot2.
    // -------------------------------------------------------------------------
    void detach(std::string const& p_robot1,
                std::string const& p_joint1,
                std::string const& p_robot2,
                std::string const& p_joint2);

    // -------------------------------------------------------------------------
    /// Maps the joints of a robot to MuJoCo @b qpos, @b DOF and actuator
    /// indices: the robot of the first chain (@ref join with no prefix).
    /// @param p_robot Robot of the first chain.
    // -------------------------------------------------------------------------
    void attach(Robot& p_robot) override;

    // -------------------------------------------------------------------------
    /// Binds one more robot of the world. Its joint @c j is the MuJoCo
    /// joint @c p_prefix + j. Binding the same robot again replaces its
    /// binding.
    /// @param p_robot Robot to bind.
    /// @param p_prefix @ref ChainOptions::prefix its chain was loaded with
    ///   (empty: the first chain).
    /// @throws std::runtime_error if no chain was loaded with @p_prefix.
    // -------------------------------------------------------------------------
    void join(Robot& p_robot, std::string const& p_prefix);

    // -------------------------------------------------------------------------
    /// Makes a backend for one robot of a world of several robots. Connect
    /// one per robot (@ref RobotSession::connect); stepping the first robot
    /// steps them all.
    ///
    /// @code
    /// auto physics = std::make_shared<robotik::MujocoBackend>();
    /// (void)physics->load("left.urdf", { .prefix = "left_" });
    /// (void)physics->load("right.urdf", { .prefix = "right_" });
    /// left.connect(robotik::MujocoBackend::member(physics, "left_"));
    /// right.connect(robotik::MujocoBackend::member(physics, "right_"));
    /// @endcode
    ///
    /// @param p_physics Physics shared by every robot of the world.
    /// @param p_prefix Chain prefix of the robot.
    /// @return The backend to connect.
    // -------------------------------------------------------------------------
    [[nodiscard]] static std::unique_ptr<RobotBackend>
    member(std::shared_ptr<MujocoBackend> p_physics, std::string p_prefix);

    // -------------------------------------------------------------------------
    /// Resets @c mjData, writes the joint positions and the base pose of
    /// every bound robot, then runs @c mj_forward.
    /// @param p_robot Robot asking for the reset.
    // -------------------------------------------------------------------------
    void reset(Robot& p_robot) override;

    // -------------------------------------------------------------------------
    /// Integrates @p_dt in steps of @ref MujocoOptions::timestep. Each
    /// physics step runs the joint loops of every bound robot, applies their
    /// efforts with gravity compensation, then measures the joints and the
    /// bases. Only the first bound robot steps the world: the call is a no-op
    /// for the others (their measures are already written).
    /// @param p_robot Robot asking for the step.
    /// @param p_dt Duration to integrate.
    // -------------------------------------------------------------------------
    void step(Robot& p_robot, Seconds p_dt) override;

    /// @return Active contacts after the last step (@c mjData.ncon), robot
    ///   against the floor excluded.
    [[nodiscard]] int contacts() const override;

    // -------------------------------------------------------------------------
    /// Casts a ray with @c mj_ray against every geom except the robot ones
    /// (see @ref RobotBackend::raycast).
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<double> raycast(Vector3 const& p_origin,
                                                Vector3 const& p_direction,
                                                double p_max) const override;

    /// Same as @ref raycast, ignoring the robot of chain prefix @p_prefix
    /// (the robot carrying the sensor) instead of the first one.
    [[nodiscard]] std::optional<double>
    raycast(Vector3 const& p_origin,
            Vector3 const& p_direction,
            double p_max,
            std::string const& p_prefix) const;

    // -------------------------------------------------------------------------
    /// Reads @c cfrc_int of the body of a link, moved to the link origin
    /// and expressed in the link frame.
    /// @param p_link Link name.
    /// @return The wrench, or nothing for an unknown link.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<Wrench>
    wrench(std::string const& p_link) const override;

private:

    /// MuJoCo spec, model, data and name tables (PIMPL).
    struct Impl;
    /// Heap-allocated implementation.
    std::unique_ptr<Impl> m_impl;

    /// One robot bound to the physics, with its MuJoCo indices.
    struct Member;

    /// Drops the compiled model: the spec changed.
    void invalidate();
    /// Compiles the spec into a model and its data.
    void compile();
    /// Maps the joints of a member to MuJoCo indices.
    void bind(Member& p_member);

    /// Options given at construction.
    MujocoOptions m_options;
};

} // namespace robotik
