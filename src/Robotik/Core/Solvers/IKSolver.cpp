/**
 * @file IKSolver.cpp
 * @brief Implementation of Inverse Kinematics solvers.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "Robotik/Core/Solvers/IKSolver.hpp"
#include "Robotik/Core/Common/Conversions.hpp"
#include "Robotik/Core/Robot/Robot.hpp"

namespace robotik
{

// ============================================================================
bool JacobianIKSolver::solve(Robot& p_robot,
                             Node const& p_end_effector,
                             Pose const& p_target_pose)
{
    m_error_message.clear();
    m_converged = false;
    m_num_iterations = 0;
    m_pose_error = std::numeric_limits<double>::max();

    State& state = p_robot.state();

    // Get initial joint values
    auto& q = state.joint_positions;
    if (q.empty())
    {
        m_error_message = "IKSolver: Robot has no joints";
        return false;
    }

    // Compute or update the Jacobian
    Jacobian const& J = p_robot.computeJacobian(state, p_end_effector);

    // Iterative solving
    for (size_t iter = 0; iter < m_config.max_iterations; ++iter)
    {
        m_num_iterations = iter + 1;

        // Compute current end-effector pose
        Transform current_transform = p_end_effector.worldTransform();
        Pose current_pose = transformToPose(current_transform);

        // Calculate error
        Pose error = calculatePoseError(p_target_pose, current_pose);
        m_pose_error = error.norm();

        // Check convergence
        if (m_pose_error < m_config.tolerance)
        {
            m_converged = true;
            if (m_config.verbose)
            {
                m_error_message =
                    "IKSolver: Converged in " + std::to_string(iter) +
                    " iterations with error " + std::to_string(m_pose_error);
            }
            return true;
        }

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

        // Apply to robot
        p_robot.blueprint().forEachJoint([&q](Joint& joint, size_t index)
                                         { joint.position(q[index]); });

        // Log verbose information
        if (m_config.verbose && iter % 50 == 0)
        {
            m_error_message += "IKSolver: Iteration " + std::to_string(iter) +
                               ", error = " + std::to_string(m_pose_error);
        }
    }

    // Max iterations reached without convergence
    if (m_config.verbose)
    {
        m_error_message +=
            "IKSolver: Failed to converge after " +
            std::to_string(m_config.max_iterations) +
            " iterations. Final error: " + std::to_string(m_pose_error);
    }

    return false;
}

} // namespace robotik
