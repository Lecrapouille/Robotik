/**
 * @file State.hpp
 * @brief Robot state - Contains all dynamic data for a robot (positions,
 * velocities, etc.)
 *
 * This file defines the State class which holds all time-varying data for a
 * robot. Inspired by Pinocchio's Data class, this separation allows for
 * efficient computation and clear separation between model (Blueprint) and
 * state (State).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Common/Types.hpp"

#include <vector>

namespace robotik
{

// ****************************************************************************
//! \brief Container for all dynamic state data of a robot.
//!
//! This class stores all time-varying data for a robot, including:
//! - Joint positions, velocities, and accelerations
//! - Link and joint transformations (forward kinematics results)
//! - Jacobian matrix (cached for efficiency)
//!
//! This design separates the static model (Blueprint) from dynamic state
//! (State), which provides several benefits:
//! - Multiple state instances can exist for the same robot model
//! - Parallel computations on different states
//! - Clean separation of concerns
//! - Memory efficiency when working with multiple robot configurations
//!
//! This architecture is inspired by Pinocchio's Model/Data separation.
// ****************************************************************************
struct State
{
    // ------------------------------------------------------------------------
    //! \brief Constructor with sizes for preallocation.
    //! \param p_num_joints Number of actuable joints in the robot.
    //! \param p_num_links Number of links in the robot.
    // ------------------------------------------------------------------------
    State(size_t p_num_joints, size_t p_num_links);

    //! \brief Joint positions (generalized coordinates q)
    std::vector<double> joint_positions;

    //! \brief Joint velocities (generalized velocities q_dot)
    std::vector<double> joint_velocities;

    //! \brief Joint accelerations (generalized accelerations q_ddot)
    std::vector<double> joint_accelerations;

    //! \brief Joint local transformations (computed from positions)
    //! These are the transformations applied by each joint based on its
    //! current position (rotation or translation). Combined with static
    //! placements to get world transforms.
    std::vector<Transform> joint_local_transforms;

    //! \brief Joint world transformations (result of forward kinematics)
    std::vector<Transform> joint_transforms;

    //! \brief Link world transformations (result of forward kinematics)
    std::vector<Transform> link_transforms;

    //! \brief Cached Jacobian matrix
    Jacobian jacobian;
};

} // namespace robotik
