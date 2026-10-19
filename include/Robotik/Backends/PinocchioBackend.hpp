// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PinocchioBackend.hpp
/// Pinocchio kinematics and damped least-squares IK for one URDF.
#pragma once

#include "Robotik/Math/Geometry.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// Analytical kinematics backend built on Pinocchio.
///
/// A <em>backend</em> is a thin adapter around an external library that owns
/// one URDF instance and exposes a stable API for the rest of the stack.
/// @ref Robot owns this class for FK and IK; time stepping is done by a
/// @ref RobotBackend such as @ref MujocoBackend.
///
/// Pinocchio builds a rigid-body model from URDF and runs fast analytical
/// kinematics (no contact simulation). This wrapper hides @c pinocchio::Model
/// and @c pinocchio::Data and maps Robotik joint and link names to Pinocchio
/// indices.
///
/// @par Pinocchio configuration (see also method names below)
/// @li @b q — generalized coordinates (@c model.nq scalars). Revolute and
///     prismatic joints use one entry each (rad or m); floating bases and
///     spherical joints use several consecutive entries. Stored internally and
///     exposed via @ref configuration.
/// @li @b v — generalized velocities (@c model.nv scalars), aligned with
///     velocity degrees of freedom. Updated through @ref velocity; used when
///     differential kinematics or IK Jacobians are needed.
/// @li @b nq / @b nv — sizes of @b q and @b v (@ref nq, @ref nv). They differ
///     when the model uses quaternions or other redundant position parameters.
/// @li @b FK — forward kinematics: @ref updateKinematics propagates @b q to
///     every frame placement; @ref framePose reads one link pose in the base
///     frame.
/// @li @b IK — inverse kinematics: @ref solveIK iterates damped least squares
///     on the frame Jacobian toward a @ref Pose target.
///
/// @par Example
/// @code
/// robotik::PinocchioBackend kin("arm.urdf");
/// std::ranges::copy(seed_q, kin.configuration().begin());
/// kin.updateKinematics();
/// robotik::Pose tcp = kin.framePose("link6");
/// if (auto q = kin.solveIK("link6", target_pose, kin.configuration()))
///     applyJointTargets(*q);
/// @endcode
// ****************************************************************************
class PinocchioBackend
{
public:

    PinocchioBackend(PinocchioBackend const&) = delete;
    PinocchioBackend& operator=(PinocchioBackend const&) = delete;

    // -------------------------------------------------------------------------
    /// Builds the Pinocchio model from URDF.
    /// @param p_urdf Path to URDF file.
    /// @param p_tool Tool URDF attached at the @c flange frame, or empty.
    /// @throws std::runtime_error if Pinocchio cannot parse the file.
    // -------------------------------------------------------------------------
    explicit PinocchioBackend(std::filesystem::path const& p_urdf,
                              std::filesystem::path const& p_tool = {});

    // -------------------------------------------------------------------------
    /// Releases model and data.
    // -------------------------------------------------------------------------
    ~PinocchioBackend();

    // -------------------------------------------------------------------------
    /// @return Number of generalized coordinates (@c model.nq).
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t nq() const;

    // -------------------------------------------------------------------------
    /// @return Number of velocity DoFs (@c model.nv).
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t nv() const;

    // -------------------------------------------------------------------------
    /// Internal @b q, written in place before @ref updateKinematics.
    /// @return The @ref nq entries.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::span<double> configuration();

    /// @copydoc configuration()
    [[nodiscard]] std::span<double const> configuration() const;

    // -------------------------------------------------------------------------
    /// Internal @b v.
    /// @return The @ref nv entries.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::span<double> velocity();

    /// @copydoc velocity()
    [[nodiscard]] std::span<double const> velocity() const;

    // -------------------------------------------------------------------------
    /// Runs forward kinematics and updates frame placements. Call it after
    /// changing @ref configuration and before @ref framePose.
    // -------------------------------------------------------------------------
    void updateKinematics();

    // -------------------------------------------------------------------------
    /// @param p_name Joint name from the URDF.
    /// @return True if @p_name is a Pinocchio joint in the model.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool hasJoint(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    /// @param p_name Joint name.
    /// @return First index in @b q for the joint, or -1.
    // -------------------------------------------------------------------------
    [[nodiscard]] int qIndex(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    /// @param p_name Joint name.
    /// @return First index in @b v for the joint, or -1.
    // -------------------------------------------------------------------------
    [[nodiscard]] int vIndex(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    /// @param p_name Joint name.
    /// @return Pinocchio joint id.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t jointId(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    /// @param p_name Frame name.
    /// @return True if @p_name is a frame (link) in the model.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool hasFrame(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    /// @return Names of the link frames, in model order (the root excluded).
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<std::string> linkFrames() const;

    // -------------------------------------------------------------------------
    /// @param p_name Frame name.
    /// @return Pinocchio frame index.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t frameId(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    /// Reads the pose of a frame after @ref updateKinematics.
    /// @param p_frame Link or operational frame name.
    /// @return The pose in the robot base frame.
    /// @throws std::invalid_argument if the frame is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose framePose(std::string const& p_frame) const;

    // -------------------------------------------------------------------------
    /// Damped least-squares IK: moves @b q step by step until the frame
    /// reaches the target.
    /// @param p_frame Link or frame name (e.g. tool link).
    /// @param p_target Desired pose in the robot base frame.
    /// @param p_seed Initial @b q; if size mismatches @ref nq, internal @b q is
    ///   used.
    /// @return Joint vector on success, or empty if iteration did not converge.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<std::vector<double>>
    solveIK(std::string const& p_frame,
            Pose const& p_target,
            std::span<double const> p_seed) const;

private:

    /// Pinocchio model, data, and configuration buffers.
    struct Impl;
    /// Heap-allocated implementation (PIMPL).
    std::unique_ptr<Impl> m_impl;
};

} // namespace robotik
