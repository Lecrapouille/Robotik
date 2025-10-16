/**
 * @file State.hpp
 * @brief Robot state - Contains all dynamic data for a robot (positions,
 * velocities, etc.)
 *
 * This file defines the State class which holds all time-varying data for a
 * robot. Inspired by Pinocchio's Data class, this separation allows for
 * efficient computation and clear separation between model (Hierarchy) and
 * state (State).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Types.hpp"

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
//! This design separates the static model (Hierarchy) from dynamic state
//! (State), which provides several benefits:
//! - Multiple state instances can exist for the same robot model
//! - Parallel computations on different states
//! - Clean separation of concerns
//! - Memory efficiency when working with multiple robot configurations
//!
//! This architecture is inspired by Pinocchio's Model/Data separation.
// ****************************************************************************
class State
{
public:

    // ------------------------------------------------------------------------
    //! \brief Default constructor creates an empty state.
    // ------------------------------------------------------------------------
    State() = default;

    // ------------------------------------------------------------------------
    //! \brief Constructor with sizes for preallocation.
    //! \param p_num_joints Number of actuable joints in the robot.
    //! \param p_num_links Number of links in the robot.
    // ------------------------------------------------------------------------
    State(size_t p_num_joints, size_t p_num_links);

    // ------------------------------------------------------------------------
    //! \brief Resize the state to match a robot's dimensions.
    //! \param p_num_joints Number of actuable joints.
    //! \param p_num_links Number of links.
    // ------------------------------------------------------------------------
    void resize(size_t p_num_joints, size_t p_num_links);

    // ------------------------------------------------------------------------
    //! \brief Get the number of joints in this state.
    //! \return Number of actuable joints.
    // ------------------------------------------------------------------------
    inline size_t numJoints() const
    {
        return m_joint_positions.size();
    }

    // ------------------------------------------------------------------------
    //! \brief Get the number of links in this state.
    //! \return Number of links.
    // ------------------------------------------------------------------------
    inline size_t numLinks() const
    {
        return m_link_transforms.size();
    }

    // ------------------------------------------------------------------------
    //! \brief Set all joint positions at once.
    //! \param p_positions Vector of joint positions (size must match
    //! numJoints).
    // ------------------------------------------------------------------------
    void setJointPositions(std::vector<double> const& p_positions);

    // ------------------------------------------------------------------------
    //! \brief Get joint positions vector.
    //! \return Const reference to joint positions.
    // ------------------------------------------------------------------------
    inline std::vector<double> const& jointPositions() const
    {
        return m_joint_positions;
    }

    // ------------------------------------------------------------------------
    //! \brief Get mutable joint positions vector.
    //! \return Reference to joint positions.
    // ------------------------------------------------------------------------
    inline std::vector<double>& jointPositions()
    {
        return m_joint_positions;
    }

    // ------------------------------------------------------------------------
    //! \brief Set all joint velocities at once.
    //! \param p_velocities Vector of joint velocities (size must match
    //! numJoints).
    // ------------------------------------------------------------------------
    void setJointVelocities(std::vector<double> const& p_velocities);

    // ------------------------------------------------------------------------
    //! \brief Get joint velocities vector.
    //! \return Const reference to joint velocities.
    // ------------------------------------------------------------------------
    inline std::vector<double> const& jointVelocities() const
    {
        return m_joint_velocities;
    }

    // ------------------------------------------------------------------------
    //! \brief Get mutable joint velocities vector.
    //! \return Reference to joint velocities.
    // ------------------------------------------------------------------------
    inline std::vector<double>& jointVelocities()
    {
        return m_joint_velocities;
    }

    // ------------------------------------------------------------------------
    //! \brief Set all joint accelerations at once.
    //! \param p_accelerations Vector of joint accelerations (size must match
    //! numJoints).
    // ------------------------------------------------------------------------
    void setJointAccelerations(std::vector<double> const& p_accelerations);

    // ------------------------------------------------------------------------
    //! \brief Get joint accelerations vector.
    //! \return Const reference to joint accelerations.
    // ------------------------------------------------------------------------
    inline std::vector<double> const& jointAccelerations() const
    {
        return m_joint_accelerations;
    }

    // ------------------------------------------------------------------------
    //! \brief Get mutable joint accelerations vector.
    //! \return Reference to joint accelerations.
    // ------------------------------------------------------------------------
    inline std::vector<double>& jointAccelerations()
    {
        return m_joint_accelerations;
    }

    // ------------------------------------------------------------------------
    //! \brief Get link transformations (world frames).
    //! \return Const reference to link transforms.
    // ------------------------------------------------------------------------
    inline std::vector<Transform> const& linkTransforms() const
    {
        return m_link_transforms;
    }

    // ------------------------------------------------------------------------
    //! \brief Get mutable link transformations.
    //! \return Reference to link transforms.
    // ------------------------------------------------------------------------
    inline std::vector<Transform>& linkTransforms()
    {
        return m_link_transforms;
    }

    // ------------------------------------------------------------------------
    //! \brief Get joint transformations.
    //! \return Const reference to joint transforms.
    // ------------------------------------------------------------------------
    inline std::vector<Transform> const& jointTransforms() const
    {
        return m_joint_transforms;
    }

    // ------------------------------------------------------------------------
    //! \brief Get mutable joint transformations.
    //! \return Reference to joint transforms.
    // ------------------------------------------------------------------------
    inline std::vector<Transform>& jointTransforms()
    {
        return m_joint_transforms;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the cached Jacobian matrix.
    //! \return Const reference to Jacobian.
    // ------------------------------------------------------------------------
    inline Jacobian const& jacobian() const
    {
        return m_jacobian;
    }

    // ------------------------------------------------------------------------
    //! \brief Get mutable Jacobian matrix.
    //! \return Reference to Jacobian.
    // ------------------------------------------------------------------------
    inline Jacobian& jacobian()
    {
        return m_jacobian;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the Jacobian needs recomputation.
    //! \return True if Jacobian is dirty (needs update).
    // ------------------------------------------------------------------------
    inline bool isJacobianDirty() const
    {
        return m_jacobian_dirty;
    }

    // ------------------------------------------------------------------------
    //! \brief Mark the Jacobian as dirty (needs recomputation).
    // ------------------------------------------------------------------------
    inline void markJacobianDirty()
    {
        m_jacobian_dirty = true;
    }

    // ------------------------------------------------------------------------
    //! \brief Mark the Jacobian as clean (up to date).
    // ------------------------------------------------------------------------
    inline void markJacobianClean()
    {
        m_jacobian_dirty = false;
    }

private:

    //! \brief Joint positions (generalized coordinates q)
    std::vector<double> m_joint_positions;

    //! \brief Joint velocities (generalized velocities q_dot)
    std::vector<double> m_joint_velocities;

    //! \brief Joint accelerations (generalized accelerations q_ddot)
    std::vector<double> m_joint_accelerations;

    //! \brief Link world transformations (result of forward kinematics)
    std::vector<Transform> m_link_transforms;

    //! \brief Joint world transformations
    std::vector<Transform> m_joint_transforms;

    //! \brief Cached Jacobian matrix
    Jacobian m_jacobian;

    //! \brief Flag indicating if Jacobian needs recomputation
    bool m_jacobian_dirty = true;
};

} // namespace robotik
