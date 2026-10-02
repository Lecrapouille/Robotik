// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PinocchioBackend.hpp
//! @brief Pinocchio kinematics and damped least-squares IK for one URDF.
#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief Position and orientation of a frame in the robot base.
//!
//! Translation in meters; quaternion @c (qw, qx, qy, qz) in scalar-first order.
// ****************************************************************************
struct Pose
{
    double px = 0.0; //!< X position in meters.
    double py = 0.0; //!< Y position in meters.
    double pz = 0.0; //!< Z position in meters.
    double qw = 1.0; //!< Quaternion scalar part.
    double qx = 0.0; //!< Quaternion X.
    double qy = 0.0; //!< Quaternion Y.
    double qz = 0.0; //!< Quaternion Z.
};

// ****************************************************************************
//! @brief Analytical kinematics backend built on Pinocchio.
//!
//! A <em>backend</em> is a thin adapter around an external library that owns
//! one URDF instance and exposes a stable API for the rest of the stack.
//! Backends are not ECS components: @ref RobotRuntime holds this class for FK
//! and IK, and @ref MujocoBackend for time-stepping contact dynamics. Skills
//! and systems talk to the runtime; the runtime forwards work to the
//! appropriate backend.
//!
//! Pinocchio builds a rigid-body model from URDF and runs fast analytical
//! kinematics (no contact simulation). This wrapper hides @c pinocchio::Model
//! and @c pinocchio::Data and maps Robotik joint and link names to Pinocchio
//! indices.
//!
//! @par Pinocchio configuration (see also method names below)
//! @li @b q — generalized coordinates (@c model.nq scalars). Revolute and
//!     prismatic joints use one entry each (rad or m); floating bases and
//!     spherical joints use several consecutive entries. Stored internally and
//!     exposed via @ref setConfiguration / @ref configuration.
//! @li @b v — generalized velocities (@c model.nv scalars), aligned with
//!     velocity degrees of freedom. Updated with @ref setVelocity /
//!     @ref velocity; used when differential kinematics or IK Jacobians are
//!     needed.
//! @li @b nq / @b nv — sizes of @b q and @b v (@ref nq, @ref nv). They differ
//!     when the model uses quaternions or other redundant position parameters.
//! @li @b FK — forward kinematics: @ref updateKinematics propagates @b q to
//!     every frame placement; @ref framePose reads one link pose in the base
//!     frame.
//! @li @b IK — inverse kinematics: @ref solveIK iterates damped least squares
//!     on the frame Jacobian toward a @ref Pose target.
//!
//! Joint and frame indices here may differ from MuJoCo—bind ECS links with
//! @c ecs::PinocchioJointBinding when both backends are active.
//!
//! @example
//! @code
//! robotik::PinocchioBackend kin("arm.urdf");
//! kin.setConfiguration(seed_q);
//! kin.updateKinematics();
//! robotik::Pose tcp = kin.framePose("link6");
//! if (auto q = kin.solveIK("link6", target_pose, seed_q))
//!     applyJointTargets(*q);
//! @endcode
// ****************************************************************************
class PinocchioBackend
{
public:

    PinocchioBackend(PinocchioBackend const&) = delete;
    PinocchioBackend& operator=(PinocchioBackend const&) = delete;

    // -------------------------------------------------------------------------
    //! @brief Builds the Pinocchio model from URDF.
    //! @param p_urdf Path to URDF file.
    //! @throws std::runtime_error if Pinocchio cannot parse the file.
    // -------------------------------------------------------------------------
    explicit PinocchioBackend(std::filesystem::path const& p_urdf);

    // -------------------------------------------------------------------------
    //! @brief Releases model and data.
    // -------------------------------------------------------------------------
    ~PinocchioBackend();

    // -------------------------------------------------------------------------
    //! @brief Number of generalized coordinates (@c model.nq).
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t nq() const;

    // -------------------------------------------------------------------------
    //! @brief Number of velocity DoFs (@c model.nv).
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t nv() const;

    // -------------------------------------------------------------------------
    //! @brief Copies @p_q into internal @b q (must match @ref nq).
    //! @param p_q Joint configuration vector.
    // -------------------------------------------------------------------------
    void setConfiguration(std::vector<double> const& p_q);

    // -------------------------------------------------------------------------
    //! @brief Copies @p_v into internal @b v (must match @ref nv).
    //! @param p_v Joint velocity vector.
    // -------------------------------------------------------------------------
    void setVelocity(std::vector<double> const& p_v);

    // -------------------------------------------------------------------------
    //! @brief Current @b q vector.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<double> configuration() const;

    // -------------------------------------------------------------------------
    //! @brief Current @b v vector.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<double> velocity() const;

    // -------------------------------------------------------------------------
    //! @brief Runs forward kinematics and updates frame placements.
    // -------------------------------------------------------------------------
    void updateKinematics();

    // -------------------------------------------------------------------------
    //! @brief True if @p_name is a Pinocchio joint in the model.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool hasJoint(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    //! @brief First index in @b q for joint @p_name, or -1.
    // -------------------------------------------------------------------------
    [[nodiscard]] int qIndex(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    //! @brief First index in @b v for joint @p_name, or -1.
    // -------------------------------------------------------------------------
    [[nodiscard]] int vIndex(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    //! @brief Pinocchio joint id for @p_name.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t jointId(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    //! @brief True if @p_name is a frame (link) in the model.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool hasFrame(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    //! @brief Pinocchio frame index for @p_name.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t frameId(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    //! @brief World pose of frame @p_frame after @ref updateKinematics.
    //! @param p_frame Link or operational frame name.
    //! @throws std::invalid_argument if the frame is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose framePose(std::string const& p_frame) const;

    // -------------------------------------------------------------------------
    //! @brief Damped least-squares IK toward @p_target for frame @p_frame.
    //! @param p_frame Link or frame name (e.g. tool link).
    //! @param p_target Desired pose in the robot base frame.
    //! @param p_seed Initial @b q; if size mismatches @ref nq, internal @b q is
    //! used.
    //! @return Joint vector on success, or empty if iteration did not converge.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<std::vector<double>>
    solveIK(std::string const& p_frame,
            Pose const& p_target,
            std::vector<double> const& p_seed) const;

private:

    //! <Pinocchio model, data, and configuration buffers.
    struct Impl;
    //!< Heap-allocated implementation (PIMPL).
    std::unique_ptr<Impl> m_impl;
};

} // namespace robotik
