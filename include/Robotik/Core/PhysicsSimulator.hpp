/**
 * @file PhysicsSimulator.hpp
 * @brief PhysicsSimulator class - Apply physics to the robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Types.hpp"

namespace robotik
{

// *********************************************************************************
//! \brief Physics simulator for robotic systems.
//!
//! This class implements forward dynamics simulation using the Newton-Euler
//! recursive algorithm. It computes joint accelerations from applied
//! torques/forces and integrates them to update joint velocities and positions.
//!
//! Features:
//! - Gravity effects on links
//! - Joint friction and damping
//! - Joint effort limits
//! - Semi-implicit Euler integration
// *********************************************************************************
class PhysicsSimulator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_dt Time step for integration (seconds).
    //! \param p_gravity Gravity vector (default: -9.81 m/s² in Z direction).
    // ------------------------------------------------------------------------
    explicit PhysicsSimulator(
        double p_dt,
        Eigen::Vector3d const& p_gravity = Eigen::Vector3d(0.0, 0.0, -9.81));

    // ------------------------------------------------------------------------
    //! \brief Set the gravity vector.
    //! \param p_gravity Gravity vector (m/s²).
    // ------------------------------------------------------------------------
    inline void setGravity(const Eigen::Vector3d& p_gravity)
    {
        m_gravity = p_gravity;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the gravity vector.
    //! \return The gravity vector (m/s²).
    // ------------------------------------------------------------------------
    inline Eigen::Vector3d const& gravity() const
    {
        return m_gravity;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the time step.
    //! \return The time step (seconds).
    // ------------------------------------------------------------------------
    inline double dt() const
    {
        return m_dt;
    }

    // ------------------------------------------------------------------------
    //! \brief Simulate one time step for the robot.
    //!
    //! This method computes the forward dynamics (joint accelerations from
    //! applied efforts) and integrates the equations of motion to update
    //! joint velocities and positions.
    //!
    //! The algorithm:
    //! 1. Traverse the kinematic tree from root to leaves
    //! 2. For each joint, compute acceleration from gravity, friction, damping
    //! 3. Integrate accelerations to update velocities
    //! 4. Integrate velocities to update positions
    //!
    //! \param p_robot The robot to simulate.
    // ------------------------------------------------------------------------
    void step(Robot& p_robot);

private:

    // ------------------------------------------------------------------------
    //! \brief Compute dynamics for a joint and its child link recursively.
    //! \param p_joint The joint to compute dynamics for.
    //! \param p_parent_vel The velocity of the parent node.
    //! \param p_parent_acc The acceleration of the parent node.
    // ------------------------------------------------------------------------
    void computeDynamics(Joint& p_joint,
                         const Eigen::Vector3d& p_parent_vel,
                         const Eigen::Vector3d& p_parent_acc);

    // ------------------------------------------------------------------------
    //! \brief Compute the gravitational force for a link.
    //! \param p_link The link to compute the force for.
    //! \return The gravitational force applied to the link (N).
    // ------------------------------------------------------------------------
    Eigen::Vector3d computeLinkForce(Link const& p_link) const;

    // ------------------------------------------------------------------------
    //! \brief Time integration using semi-implicit Euler method.
    //! \param p_joint The joint to integrate.
    // ------------------------------------------------------------------------
    void integrate(Joint& p_joint);

private:

    double m_dt;               //!< Time step (seconds)
    Eigen::Vector3d m_gravity; //!< Gravity vector (m/s²)
};

} // namespace robotik