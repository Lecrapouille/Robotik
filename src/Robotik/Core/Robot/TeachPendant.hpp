/**
 * @file TeachPendant.hpp
 * @brief Teach pendant pour le contrôle interactif du robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Robot.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"

namespace robotik
{

// ****************************************************************************
//! \brief Virtual teach pendant for interactive control of the robot.
//!
//! This class implements a virtual teach pendant for interactive control of
//! the robot. It is stateless and takes Robot as a parameter to methods.
//! - Control the robot in joint space mode.
//! - Control the robot in Cartesian mode with IK.
// ****************************************************************************
class TeachPendant
{
public:

    // ------------------------------------------------------------------------
    //! \brief Default constructor.
    // ------------------------------------------------------------------------
    TeachPendant() = default;

    // ------------------------------------------------------------------------
    //! \brief Configure the IK solver.
    //! \param p_solver Pointer to the IK solver (not owned).
    // ------------------------------------------------------------------------
    void setIKSolver(IKSolver* p_solver);

    // ------------------------------------------------------------------------
    //! \brief Move a specific joint.
    //! \param p_robot Robot to control.
    //! \param p_joint_idx Index of the joint.
    //! \param p_delta Position variation (rad or m).
    //! \param p_speed Speed factor.
    //! \return true if the movement has been applied.
    // ------------------------------------------------------------------------
    bool moveJoint(Robot& p_robot,
                   size_t p_joint_idx,
                   double p_delta,
                   double p_speed = 1.0);

    // ------------------------------------------------------------------------
    //! \brief Move multiple joints simultaneously.
    //! \param p_robot Robot to control.
    //! \param p_deltas Position variations for each joint.
    //! \param p_speed Speed factor.
    //! \return true if the movement has been applied.
    // ------------------------------------------------------------------------
    bool moveJoints(Robot& p_robot,
                    const std::vector<double>& p_deltas,
                    double p_speed = 1.0);

    // ------------------------------------------------------------------------
    //! \brief Move the end effector in Cartesian translation.
    //! \param p_robot Robot to control.
    //! \param p_end_effector End effector to control.
    //! \param p_translation Translation vector (m).
    //! \param p_speed Speed factor.
    //! \param p_frame Reference frame (nullptr = world frame).
    //! \return true if the movement has been applied.
    // ------------------------------------------------------------------------
    bool moveCartesian(Robot& p_robot,
                       Node const* p_end_effector,
                       const Eigen::Vector3d& p_translation,
                       double p_speed = 1.0,
                       Node const* p_frame = nullptr);

    // ------------------------------------------------------------------------
    //! \brief Apply a rotation to the end effector.
    //! \param p_robot Robot to control.
    //! \param p_end_effector End effector to control.
    //! \param p_rotation_axis Rotation axis (normalized).
    //! \param p_angle Rotation angle (rad).
    //! \param p_speed Speed factor.
    //! \param p_frame Reference frame (nullptr = world frame).
    //! \return true if the movement has been applied.
    // ------------------------------------------------------------------------
    bool rotateCartesian(Robot& p_robot,
                         Node const* p_end_effector,
                         const Eigen::Vector3d& p_rotation_axis,
                         double p_angle,
                         double p_speed = 1.0,
                         Node const* p_frame = nullptr);

    // ------------------------------------------------------------------------
    //! \brief Get the error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    std::string const& error() const
    {
        return m_error;
    }

private:

    //! \brief Pointer to the IK solver (shared, not owned).
    IKSolver* m_ik_solver = nullptr;
    //! \brief Error message.
    std::string m_error;
};

} // namespace robotik
