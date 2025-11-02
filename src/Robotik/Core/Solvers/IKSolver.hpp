/**
 * @file IKSolver.hpp
 * @brief Inverse Kinematics solvers for robotic manipulators.
 *
 * This module provides various IK solving algorithms that are separate from
 * the Robot class, allowing for flexible solver selection and configuration.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Common/Types.hpp"

#include <memory>
#include <vector>

namespace robotik
{

// Forward declarations
struct State;
class Node;
class Robot;

// ****************************************************************************
//! \brief Base interface for Inverse Kinematics solvers.
//!
//! This abstract base class defines the interface for all IK solving
//! algorithms. Different implementations can provide various solving
//! strategies (Jacobian-based, analytical, optimization-based, etc.).
// ****************************************************************************
class IKSolver
{
public:

    virtual ~IKSolver() = default;

    // ------------------------------------------------------------------------
    //! \brief Solve inverse kinematics for a target pose.
    //!
    //! \param p_robot The robot to solve IK for.
    //! \param p_target_pose Desired end-effector 6D pose [x,y,z,rx,ry,rz].
    //! \param p_end_effector Reference to the end effector node.
    //! \return True if solution found, false otherwise.
    // ------------------------------------------------------------------------
    virtual bool solve(Robot& p_robot,
                       Node const& p_end_effector,
                       Pose const& p_target_pose) = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the pose error magnitude from the solving process.
    //! \return Error magnitude (distance to target).
    // ------------------------------------------------------------------------
    virtual double poseError() const = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the number of iterations used in the solving process.
    //! \return Number of iterations.
    // ------------------------------------------------------------------------
    virtual size_t numIterations() const = 0;

    // ------------------------------------------------------------------------
    //! \brief Check if the solving process converged successfully.
    //! \return True if converged, false otherwise.
    // ------------------------------------------------------------------------
    virtual bool converged() const = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the error message.
    // ------------------------------------------------------------------------
    virtual std::string const& errorMessage() const = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the computed IK solution.
    //! \return Joint values representing the IK solution.
    // ------------------------------------------------------------------------
    virtual JointValues const& solution() const = 0;
};

// ****************************************************************************
//! \brief Jacobian-based IK solver using damped least squares.
//!
//! This solver uses the Jacobian matrix and an iterative Newton-Raphson
//! approach with damping (Levenberg-Marquardt style) to find joint
//! configurations that achieve the desired end-effector pose.
//!
//! ALGORITHM:
//! 1. Compute current end-effector pose
//! 2. Calculate pose error (target - current)
//! 3. Compute Jacobian matrix
//! 4. Solve: Δq = J^T(JJ^T + λI)^(-1) * error (damped least squares)
//! 5. Update: q_new = q_old + Δq
//! 6. Repeat until convergence or max iterations
//!
//! ADVANTAGES:
//! - General purpose, works for most robots
//! - Handles singularities via damping
//! - Configurable convergence criteria
//!
//! LIMITATIONS:
//! - May get stuck in local minima
//! - Requires good initial guess
//! - Slower than analytical solutions when available
// ****************************************************************************
class JacobianIKSolver final: public IKSolver
{
public:

    // ------------------------------------------------------------------------
    //! \brief Configuration parameters for the Jacobian IK solver.
    // ------------------------------------------------------------------------
    struct Config
    {
        //! Maximum number of iterations
        size_t max_iterations = 500;
        //! Convergence tolerance (error threshold)
        double tolerance = 1e-4;
        //! Damping factor for singularity avoidance
        double damping = 0.01;
        //! Step size scaling factor (for stability)
        double step_size = 1.0;
        //! Verbose output for debugging
        bool verbose = false;
    };

    // ------------------------------------------------------------------------
    //! \brief Constructor with default configuration.
    // ------------------------------------------------------------------------
    JacobianIKSolver() = default;

    // ------------------------------------------------------------------------
    //! \brief Constructor with custom configuration.
    //! \param p_config Solver configuration parameters.
    // ------------------------------------------------------------------------
    explicit JacobianIKSolver(Config const& p_config) : m_config(p_config) {}

    Config& config()
    {
        return m_config;
    }

    Config const& config() const
    {
        return m_config;
    }

    bool solve(Robot& p_robot,
               Node const& p_end_effector,
               Pose const& p_target_pose) override;

    double poseError() const override
    {
        return m_pose_error;
    }

    size_t numIterations() const override
    {
        return m_num_iterations;
    }

    bool converged() const override
    {
        return m_converged;
    }

    std::string const& errorMessage() const override
    {
        return m_error_message;
    }

    JointValues const& solution() const override
    {
        return m_solution;
    }

private:

    Config m_config;
    double m_pose_error = 0.0;
    size_t m_num_iterations = 0;
    bool m_converged = false;
    std::string m_error_message;
    JointValues m_solution;
};

// ****************************************************************************
//! \brief Factory function to create default IK solver.
//!
//! \return A unique pointer to the default IK solver (JacobianIKSolver).
// ****************************************************************************
inline std::unique_ptr<IKSolver> createDefaultIKSolver()
{
    return std::make_unique<JacobianIKSolver>();
}

// ****************************************************************************
//! \brief Factory function to create configured Jacobian IK solver.
//!
//! \param p_config Solver configuration.
//! \return A unique pointer to a configured JacobianIKSolver.
// ****************************************************************************
inline std::unique_ptr<IKSolver>
createJacobianIKSolver(JacobianIKSolver::Config const& p_config)
{
    return std::make_unique<JacobianIKSolver>(p_config);
}

} // namespace robotik
