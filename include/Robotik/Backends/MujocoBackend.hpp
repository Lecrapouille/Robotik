// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file MujocoBackend.hpp
//! @brief MuJoCo dynamics backend for one robot loaded from URDF.
#pragma once

#include "Compages/Core/Units.hpp"

#include <filesystem>
#include <string>

namespace robotik
{

// ****************************************************************************
//! @brief Time-stepping physics backend built on MuJoCo.
//!
//! A <em>backend</em> is a thin adapter around an external library
//! that owns one URDF instance and exposes a stable API for the rest of the
//! stack. Backends are not ECS components: @ref RobotRuntime holds them
//! (@ref PinocchioBackend for analytical FK/IK, this class for contact
//! dynamics). Skills and systems talk to the runtime; the runtime forwards
//! work to the appropriate backend.
//!
//! MuJoCo (Multi-Joint dynamics with Contact) integrates rigid-body motion,
//! actuators, and contact. This wrapper hides @c mjModel (constants) and
//! @c mjData (state) and maps Robotik joint names to MuJoCo indices. URDF
//! files missing inertial data are copied to a temporary file with defaults so
//! MuJoCo can load them without changing what Pinocchio or Compages see.
//!
//! @par MuJoCo state and indexing (see also method names below)
//! @li @b qpos — generalized positions @c mjData.qpos. One scalar per entry
//!     in the position vector (revolute/prismatic: angle or displacement in rad
//!     or m; free joints and ball joints use several consecutive entries,
//!     e.g. quaternion + translation). Use @ref qposIndex to find the first
//!     index for a joint; @ref qpos reads one element.
//! @li @b qvel — generalized velocities @c mjData.qvel, aligned with @b DOFs
//!     (see below). Revolute/prismatic joints use one velocity each.
//! @li @b DOF (degree of freedom) — one independent motion axis in velocity
//!     space (@c model.nv entries). Applied wrenches and torques in MuJoCo
//!     act on DOFs via @c qfrc_applied; @ref dofIndex maps a joint to its
//!     first DOF. For simple 1-D joints, @ref qvelIndex and @ref dofIndex
//!     coincide; multi-DOF joints (free, ball) do not share the same layout
//!     as @b qpos.
//! @li @c qfrc_applied — external generalized forces/torques you add each step
//!     (cleared by @ref clearAppliedForces). @ref addQfrc accumulates on one
//!     DOF.
//! @li @c ctrl — actuator commands @c mjData.ctrl (@ref setCtrl); distinct
//!     from @c qfrc_applied unless the model wires actuators that way.
//! @li @c mj_forward — recomputes kinematics and forces from current @b qpos
//!     / @b qvel without advancing time; used after @ref setQpos and on
//!     @ref reset.
//!
//! Joint indices returned by @ref jointId are MuJoCo joint ids, not Pinocchio
//! indices—bind ECS links with @c ecs::MujocoJointBinding when both backends
//! are active.
//!
//! @example
//! @code
//! robotik::MujocoBackend sim("arm.urdf");
//! sim.reset();
//! sim.clearAppliedForces();
//! sim.compensateGravity();
//! sim.addQfrc(sim.dofIndex(sim.jointId("joint1")), torque);
//! sim.step(Seconds(0.001));
//! double q = sim.qpos(sim.qposIndex(sim.jointId("joint1")));
//! @endcode
// ****************************************************************************
class MujocoBackend
{
public:

    MujocoBackend(MujocoBackend const&) = delete;
    MujocoBackend& operator=(MujocoBackend const&) = delete;

    // -------------------------------------------------------------------------
    //! @brief Loads the model from URDF (with optional inertia patch).
    //! @param p_filename Path to URDF.
    //! @throws std::runtime_error if MuJoCo cannot parse the file.
    // -------------------------------------------------------------------------
    explicit MujocoBackend(std::filesystem::path const& p_filename);

    // -------------------------------------------------------------------------
    //! @brief Frees model, data, and any generated URDF copy.
    // -------------------------------------------------------------------------
    ~MujocoBackend();

    // -------------------------------------------------------------------------
    //! @brief Resets state and runs @c mj_forward.
    // -------------------------------------------------------------------------
    void reset();

    // -------------------------------------------------------------------------
    //! @brief Integrates dynamics for @p_dt seconds.
    //! @param p_dt Timestep; also written to @c model->opt.timestep when
    //! positive.
    // -------------------------------------------------------------------------
    void step(Seconds p_dt);

    // -------------------------------------------------------------------------
    //! @brief Simulation time after the last step.
    // -------------------------------------------------------------------------
    [[nodiscard]] Seconds time() const;

    // -------------------------------------------------------------------------
    //! @brief MuJoCo joint index (@c jnt_*), or -1 if unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] int jointId(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    //! @brief First index into @c mjData.qpos for @p_joint_id (MuJoCo @c
    //! jnt_qposadr).
    // -------------------------------------------------------------------------
    [[nodiscard]] int qposIndex(int p_joint_id) const;

    // -------------------------------------------------------------------------
    //! @brief First index into @c mjData.qvel for @p_joint_id (MuJoCo @c
    //! jnt_dofadr).
    // -------------------------------------------------------------------------
    [[nodiscard]] int qvelIndex(int p_joint_id) const;

    // -------------------------------------------------------------------------
    //! @brief First DOF index for @p_joint_id; use with @ref addQfrc on @c
    //! qfrc_applied.
    // -------------------------------------------------------------------------
    [[nodiscard]] int dofIndex(int p_joint_id) const;

    // -------------------------------------------------------------------------
    //! @brief Actuator index by name, or -1.
    // -------------------------------------------------------------------------
    [[nodiscard]] int actuatorId(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    //! @brief One @b qpos component (rad, m, or unitless for quaternion parts).
    //! @param p_index Entry in @c mjData.qpos, usually from @ref qposIndex.
    // -------------------------------------------------------------------------
    [[nodiscard]] double qpos(int p_index) const;

    // -------------------------------------------------------------------------
    //! @brief One @b qvel component (rad/s or m/s).
    //! @param p_index Entry in @c mjData.qvel, usually from @ref qvelIndex.
    // -------------------------------------------------------------------------
    [[nodiscard]] double qvel(int p_index) const;

    // -------------------------------------------------------------------------
    //! @brief Number of active contacts after the last step.
    // -------------------------------------------------------------------------
    [[nodiscard]] int contacts() const;

    // -------------------------------------------------------------------------
    //! @brief Sets one @c qpos entry and calls @c mj_forward.
    //! @param p_index Generalized position index.
    //! @param p_value New value.
    // -------------------------------------------------------------------------
    void setQpos(int p_index, double p_value);

    // -------------------------------------------------------------------------
    //! @brief Zeros @c ctrl and @c qfrc_applied.
    // -------------------------------------------------------------------------
    void clearAppliedForces();

    // -------------------------------------------------------------------------
    //! @brief Adds gravity/Coriolis bias torques to @c qfrc_applied.
    //! Call after @ref clearAppliedForces.
    // -------------------------------------------------------------------------
    void compensateGravity();

    // -------------------------------------------------------------------------
    //! @brief Sets one actuator control if the index is valid.
    //! @param p_actuator_id Actuator index.
    //! @param p_effort Control value.
    // -------------------------------------------------------------------------
    void setCtrl(int p_actuator_id, double p_effort);

    // -------------------------------------------------------------------------
    //! @brief Adds to @c qfrc_applied at a DOF.
    //! @param p_dof_index DOF index.
    //! @param p_effort Generalized force increment.
    // -------------------------------------------------------------------------
    void addQfrc(int p_dof_index, double p_effort);

private:

    //!< Opaque MuJoCo handles and temp URDF path.
    struct Impl;
    //!< Implementation pointer (PIMPL).
    Impl* m_impl = nullptr;
    //!< Cached simulation time.
    double m_time = 0.0;
};

} // namespace robotik
