// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file BackendComponents.hpp
//! @brief ECS indices linking entities to MuJoCo and Pinocchio (no ownership).
//! Joint *names* match URDF on both backends, but numeric indices differ: store
//! @ref MujocoJointBinding and @ref PinocchioJointBinding separately on each
//! link.
#pragma once

#include <cstddef>

namespace robotik::ecs
{

// ****************************************************************************
//! @brief MuJoCo joint indices for one actuated link entity.
// ****************************************************************************
struct MujocoJointBinding
{
    //!< @c mj_name2id(..., mjOBJ_JOINT, ...).
    int joint_id = -1;
    //!< First index into @c mjData::qpos (generalized positions: rad, m, or
    //!< quaternion parts depending on joint type).
    int qpos_index = -1;
    //!< First index into @c mjData::qvel (generalized velocities: rad/s or m/s
    //!< per DOF; layout may differ from @c qpos for multi-DOF joints).
    int qvel_index = -1;
    //!< DOF index for @c qfrc_applied.
    int dof_index = -1;
};

// ****************************************************************************
//! @brief Optional named actuator for a joint (falls back to @c qfrc_applied).
// ****************************************************************************
struct MujocoActuatorBinding
{
    //!< @c mj_name2id(..., mjOBJ_ACTUATOR, ...).
    int actuator_id = -1;
};

// ****************************************************************************
//! @brief Pinocchio joint indices for configuration sync.
// ****************************************************************************
struct PinocchioJointBinding
{
    //!< Pinocchio joint index.
    std::size_t joint_id = 0;
    //!< First index in Pinocchio @c q (generalized coordinates: rad or m per
    //!< joint; multi-DOF joints use several consecutive entries).
    int q_index = -1;
    //!< First index in Pinocchio @c v (generalized velocities aligned with
    //!< velocity DOFs; @c model.nv may differ from @c model.nq).
    int v_index = -1;
};

// ****************************************************************************
//! @brief Pinocchio frame id for FK / IK on a link (e.g. tool flange).
// ****************************************************************************
struct PinocchioFrameBinding
{
    //!< Pinocchio frame index.
    std::size_t frame_id = 0;
};

} // namespace robotik::ecs
