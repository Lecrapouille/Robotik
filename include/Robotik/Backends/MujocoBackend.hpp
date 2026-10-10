// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file MujocoBackend.hpp
//! @brief MuJoCo dynamics backend. Each loaded URDF is its own kinematic chain.
#pragma once

#include "Robotik/Backends/RobotBackend.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief How the robot is placed in the MuJoCo world.
// ****************************************************************************
struct MujocoOptions
{
    //!< Free joint on the root link (mobile robots); false bolts the root
    //!< link at the base pose of the robot.
    bool floating_base = false;
    //!< Infinite ground plane at z = 0.
    bool floor = false;
    //!< Sliding friction per link name (default 1): e.g. a caster ball.
    std::unordered_map<std::string, double> friction;
    //!< Physics step; the joint loops run at this rate.
    Seconds timestep{ 0.001 };
};

// ****************************************************************************
//! @brief Time-stepping physics backend built on MuJoCo.
//!
//! A <em>backend</em> is a thin adapter around an external library that owns
//! the simulated bodies and exposes a stable API for the rest of the stack
//! (see @ref RobotBackend). Backends are not ECS components: the
//! @ref RobotSession holds them, next to the @ref PinocchioBackend that every
//! robot owns for analytical FK/IK. Skills talk to the robot; the robot
//! forwards work to its backends.
//!
//! MuJoCo (Multi-Joint dynamics with Contact) integrates rigid-body motion,
//! actuators and contacts. This wrapper hides @c mjModel (constants) and
//! @c mjData (state) and maps Robotik joint names to MuJoCo indices. URDF
//! files missing inertial data are copied to a private temporary file with
//! defaults, so MuJoCo can load them without changing what Pinocchio or
//! Compages see. @ref load keeps each file as its own chain. @ref attach
//! hangs one chain on a link of another; @ref detach puts it back in the
//! world. The assembled spec is edited (free joint, floor, friction) then
//! compiled. Instances are independent: one per parallel environment.
//!
//! @par MuJoCo state and indexing
//! @li @b qpos — generalized positions @c mjData.qpos. One scalar per entry
//!     in the position vector (revolute/prismatic: angle or displacement in
//!     rad or m). The free joint of a floating base uses seven consecutive
//!     entries: position (m) then orientation quaternion (w, x, y, z).
//! @li @b qvel — generalized velocities @c mjData.qvel, aligned with @b DOFs.
//!     Revolute/prismatic joints use one velocity each; the free joint uses
//!     six: linear velocity in the world frame, angular velocity in the body
//!     frame.
//! @li @b DOF (degree of freedom) — one independent motion axis in velocity
//!     space (@c model.nv entries). Applied torques act on DOFs through
//!     @c qfrc_applied. For 1-D joints the qvel index and the DOF index
//!     coincide; multi-DOF joints (free, ball) do not share the @b qpos layout.
//! @li @c qfrc_applied — external generalized forces added each step: the
//!     joint loop efforts (@ref JointSet::control) plus the gravity
//!     compensation of the actuated joints (@c qfrc_bias).
//! @li @c ctrl — actuator commands @c mjData.ctrl, used instead of
//!     @c qfrc_applied for joints that the URDF wires to a MuJoCo actuator.
//! @li @c mj_forward — recomputes kinematics and forces from the current
//!     @b qpos / @b qvel without advancing time; used on @ref reset.
//! @li @c cfrc_int — interaction wrench between a body and its parent, read
//!     by @ref wrench after @c mj_rnePostConstraint.
//!
//! @code
//! auto physics = std::make_unique<robotik::MujocoBackend>(
//!     robotik::MujocoOptions{ .floating_base = true, .floor = true });
//! std::string const arm = physics->load("robot_6axis.urdf");
//! std::string const tool = physics->load("tool_gripper.urdf");
//! physics->attach(arm, "flange", tool, "tool_mount");
//! robot.connect(std::move(physics));
//! @endcode
// ****************************************************************************
class MujocoBackend final: public RobotBackend
{
public:

    explicit MujocoBackend(MujocoOptions p_options = {});
    ~MujocoBackend() override;

    // -------------------------------------------------------------------------
    //! @brief Loads one URDF as its own kinematic chain.
    //! @return The chain name (the URDF @c robot name, made unique if needed).
    //! @throws std::runtime_error if MuJoCo cannot parse the file.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string load(std::filesystem::path const& p_urdf);

    // -------------------------------------------------------------------------
    //! @brief Hangs @p_robot2 on @p_robot1.
    //!
    //! @p_joint1 and @p_joint2 name a link, or a joint (the link that joint
    //! moves). The named link of @p_robot2 and every body under it become
    //! children of the named link of @p_robot1. The chain can be separated
    //! again with @ref detach.
    //! @throws std::runtime_error if a chain or a link is unknown, or if
    //! @p_robot2 is already attached.
    // -------------------------------------------------------------------------
    void attach(std::string const& p_robot1,
                std::string const& p_joint1,
                std::string const& p_robot2,
                std::string const& p_joint2);

    // -------------------------------------------------------------------------
    //! @brief Undoes @ref attach. @p_robot2 is a chain of its own again,
    //! standing in the world.
    // -------------------------------------------------------------------------
    void detach(std::string const& p_robot1,
                std::string const& p_joint1,
                std::string const& p_robot2,
                std::string const& p_joint2);

    // -------------------------------------------------------------------------
    //! @brief Maps the joints of @p_robot to MuJoCo @b qpos, @b DOF and
    //! actuator indices.
    // -------------------------------------------------------------------------
    void attach(Robot& p_robot) override;

    // -------------------------------------------------------------------------
    //! @brief Resets @c mjData, writes the joint positions and the base pose,
    //! then runs @c mj_forward.
    // -------------------------------------------------------------------------
    void reset(Robot& p_robot) override;

    // -------------------------------------------------------------------------
    //! @brief Integrates @p_dt in steps of @ref MujocoOptions::timestep. Each
    //! physics step runs the joint loops, applies their efforts with gravity
    //! compensation, then measures the joints and the base.
    // -------------------------------------------------------------------------
    void step(Robot& p_robot, Seconds p_dt) override;

    //! @brief Active contacts after the last step (@c mjData.ncon), robot
    //! against the floor excluded.
    [[nodiscard]] int contacts() const override;

    // -------------------------------------------------------------------------
    //! @brief @c mj_ray against every geom except the robot ones.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<double> raycast(Vector3 const& p_origin,
                                                Vector3 const& p_direction,
                                                double p_max) const override;

    // -------------------------------------------------------------------------
    //! @brief @c cfrc_int of the body of @p_link, moved to the link origin and
    //! expressed in the link frame.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<Wrench>
    wrench(std::string const& p_link) const override;

private:

    struct Impl;
    std::unique_ptr<Impl> m_impl;

    void invalidate();
    void compile();
    void bind(Robot& p_robot);

    struct Binding
    {
        int qpos = -1;
        int dof = -1;
        int actuator = -1;
    };

    //!< Indexed by @ref JointId.
    std::vector<Binding> m_bindings;
    MujocoOptions m_options;
};

} // namespace robotik
