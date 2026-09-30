// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file BackendComponents.hpp
 * @brief ECS indices linking entities to MuJoCo and Pinocchio (no ownership).
 */

#pragma once

#include <cstddef>

namespace robotik::ecs
{

/**
 * @brief MuJoCo joint indices for one actuated link entity.
 */
struct MujocoJointBinding
{
    /** @brief @c mj_name2id(..., mjOBJ_JOINT, ...). */
    int joint_id = -1;

    /** @brief Offset into @c mjData::qpos. */
    int qpos_index = -1;

    /** @brief Offset into @c mjData::qvel. */
    int qvel_index = -1;

    /** @brief DOF index for @c qfrc_applied. */
    int dof_index = -1;
};

/**
 * @brief Optional named actuator for a joint (falls back to @c qfrc_applied).
 */
struct MujocoActuatorBinding
{
    /** @brief @c mj_name2id(..., mjOBJ_ACTUATOR, ...). */
    int actuator_id = -1;
};

/**
 * @brief Pinocchio joint indices for configuration sync.
 */
struct PinocchioJointBinding
{
    /** @brief Pinocchio joint index. */
    std::size_t joint_id = 0;

    /** @brief Index in Pinocchio @c q. */
    int q_index = -1;

    /** @brief Index in Pinocchio @c v. */
    int v_index = -1;
};

/**
 * @brief Pinocchio frame id for FK / IK on a link (e.g. tool flange).
 */
struct PinocchioFrameBinding
{
    /** @brief Pinocchio frame index. */
    std::size_t frame_id = 0;
};

} // namespace robotik::ecs
