/**
 * @file IKSolver.cpp
 * @brief Implementation of Inverse Kinematics solvers.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "Robotik/Core/IKSolver.hpp"
#include "Robotik/Core/Conversions.hpp"
#include "Robotik/Core/Robot.hpp"

#include <iostream>

namespace robotik
{

// ============================================================================
std::vector<double> JacobianIKSolver::solve(Robot& p_robot,
                                            scene::Node const& p_end_effector,
                                            Pose const& p_target_pose)
{
    m_converged = false;
    m_last_iterations = 0;
    m_last_error = std::numeric_limits<double>::max();

    // Get initial joint values
    std::vector<double> q = p_robot.jointPositions();

    if (q.empty())
    {
        if (m_config.verbose)
            std::cerr << "IKSolver: Robot has no joints\n";
        return {};
    }

    // Iterative solving
    for (size_t iter = 0; iter < m_config.max_iterations; ++iter)
    {
        m_last_iterations = iter + 1;

        // Compute current end-effector pose
        Transform current_transform = p_end_effector.worldTransform();
        Pose current_pose = utils::transformToPose(current_transform);

        // Calculate error
        Pose error = calculatePoseError(p_target_pose, current_pose);
        m_last_error = error.norm();

        // Check convergence
        if (m_last_error < m_config.tolerance)
        {
            m_converged = true;
            if (m_config.verbose)
            {
                std::cout << "IKSolver: Converged in " << iter
                          << " iterations with error " << m_last_error << "\n";
            }
            return q;
        }

        // Compute Jacobian
        Jacobian J = p_robot.jacobian(p_end_effector);

        // Damped least squares: Δq = J^T(JJ^T + λI)^(-1) * error
        Eigen::MatrixXd JJt = J * J.transpose();
        Eigen::MatrixXd damped =
            JJt + m_config.damping *
                      Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols());

        // Solve for joint velocity
        Eigen::VectorXd dq = J.transpose() * damped.inverse() * error;

        // Scale by step size for stability
        dq *= m_config.step_size;

        // Update joint values
        for (size_t i = 0; i < q.size() && i < static_cast<size_t>(dq.size());
             ++i)
        {
            q[i] += dq[i];
        }

        // Apply updated joint values to robot
        if (!p_robot.setJointValues(q))
        {
            if (m_config.verbose)
                std::cerr << "IKSolver: Failed to set joint values\n";
            return {};
        }

        if (m_config.verbose && iter % 50 == 0)
        {
            std::cout << "IKSolver: Iteration " << iter
                      << ", error = " << m_last_error << "\n";
        }
    }

    // Max iterations reached without convergence
    if (m_config.verbose)
    {
        std::cerr << "IKSolver: Failed to converge after "
                  << m_config.max_iterations
                  << " iterations. Final error: " << m_last_error << "\n";
    }

    return {}; // Return empty vector to indicate failure
}

// ============================================================================
Pose JacobianIKSolver::calculatePoseError(Pose const& p_target,
                                          Pose const& p_current) const
{
    Pose error;

    // Position error (simple difference)
    error.head<3>() = p_target.head<3>() - p_current.head<3>();

    // Orientation error (convert to rotation matrices and compute proper error)
    Eigen::Matrix3d R_target =
        utils::eulerToRotation(p_target(3), p_target(4), p_target(5));
    Eigen::Matrix3d R_current =
        utils::eulerToRotation(p_current(3), p_current(4), p_current(5));

    // Rotation error: R_error = R_target * R_current^T
    Eigen::Matrix3d R_error = R_target * R_current.transpose();

    // Convert rotation error to axis-angle representation
    Eigen::AngleAxisd angle_axis(R_error);
    Eigen::Vector3d axis = angle_axis.axis();
    double angle = angle_axis.angle();

    // Orientation error vector
    error.tail<3>() = angle * axis;

    return error;
}

} // namespace robotik
