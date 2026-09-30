// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file MujocoBackend.hpp
 * @brief MuJoCo dynamics for one robot loaded from URDF.
 */

#pragma once

#include <string>

namespace robotik
{

/**
 * @brief Wraps @c mjModel / @c mjData for forces, stepping, and joint indexing.
 *
 * URDF files missing inertial data are copied to a temporary file with defaults
 * so MuJoCo can load them. One backend per @ref RobotRuntime.
 *
 * @example
 * @code
 * robotik::MujocoBackend sim("arm.urdf");
 * sim.reset();
 * sim.clearAppliedForces();
 * sim.compensateGravity();
 * sim.addQfrc(sim.dofIndex(sim.jointId("joint1")), torque);
 * sim.step(0.001);
 * double q = sim.qpos(sim.qposIndex(sim.jointId("joint1")));
 * @endcode
 */
class MujocoBackend
{
public:

    /**
     * @brief Loads the model from URDF (with optional inertia patch).
     * @param p_filename Path to URDF.
     * @throws std::runtime_error if MuJoCo cannot parse the file.
     */
    explicit MujocoBackend(std::string const& p_filename);

    /** @brief Frees model, data, and any generated URDF copy. */
    ~MujocoBackend();

    MujocoBackend(MujocoBackend const&) = delete;
    MujocoBackend& operator=(MujocoBackend const&) = delete;

    /** @brief Resets state and runs @c mj_forward. */
    void reset();

    /**
     * @brief Integrates dynamics for @p_dt seconds.
     * @param p_dt Timestep; also written to @c model->opt.timestep when positive.
     */
    void step(double p_dt);

    /** @brief Simulation time after the last step. */
    [[nodiscard]] double time() const;

    /** @brief MuJoCo joint index, or -1 if unknown. */
    [[nodiscard]] int jointId(std::string const& p_name) const;

    /** @brief Index into @c qpos for @p_joint_id. */
    [[nodiscard]] int qposIndex(int p_joint_id) const;

    /** @brief Index into @c qvel for @p_joint_id. */
    [[nodiscard]] int qvelIndex(int p_joint_id) const;

    /** @brief DOF index for applied generalized forces. */
    [[nodiscard]] int dofIndex(int p_joint_id) const;

    /** @brief Actuator index by name, or -1. */
    [[nodiscard]] int actuatorId(std::string const& p_name) const;

    /** @brief Generalized position at @p_index. */
    [[nodiscard]] double qpos(int p_index) const;

    /** @brief Generalized velocity at @p_index. */
    [[nodiscard]] double qvel(int p_index) const;

    /** @brief Number of active contacts after the last step. */
    [[nodiscard]] int contacts() const;

    /**
     * @brief Sets one @c qpos entry and calls @c mj_forward.
     * @param p_index Generalized position index.
     * @param p_value New value.
     */
    void setQpos(int p_index, double p_value);

    /** @brief Zeros @c ctrl and @c qfrc_applied. */
    void clearAppliedForces();

    /**
     * @brief Adds gravity/Coriolis bias torques to @c qfrc_applied.
     * Call after @ref clearAppliedForces.
     */
    void compensateGravity();

    /**
     * @brief Sets one actuator control if the index is valid.
     * @param p_actuator_id Actuator index.
     * @param p_effort Control value.
     */
    void setCtrl(int p_actuator_id, double p_effort);

    /**
     * @brief Adds to @c qfrc_applied at a DOF.
     * @param p_dof_index DOF index.
     * @param p_effort Generalized force increment.
     */
    void addQfrc(int p_dof_index, double p_effort);

private:

    /** @brief Opaque MuJoCo handles and temp URDF path. */
    struct Impl;

    /** @brief Implementation pointer (PIMPL). */
    Impl* m_impl = nullptr;

    /** @brief Cached simulation time. */
    double m_time = 0.0;
};

} // namespace robotik
