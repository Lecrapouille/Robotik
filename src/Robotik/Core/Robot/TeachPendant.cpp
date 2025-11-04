/**
 * @file TeachPendant.cpp
 * @brief Implementation of the teach pendant for interactive control of the
 * robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/TeachPendant.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/ControlledRobot.hpp"
#include "Robotik/Core/Solvers/Trajectory.hpp"

#include <iostream>

namespace robotik
{

// ----------------------------------------------------------------------------
TeachPendant::TeachPendant()
    : m_robot(nullptr),
      m_controlled_robot(nullptr),
      m_ik_solver(nullptr),
      m_end_effector(nullptr)
{
}

// ----------------------------------------------------------------------------
void TeachPendant::setRobot(application::ControlledRobot& p_robot)
{
    m_robot = &p_robot;
    m_controlled_robot = &p_robot;
    m_end_effector = p_robot.end_effector;
}

// ----------------------------------------------------------------------------
void TeachPendant::setIKSolver(IKSolver* p_solver)
{
    m_ik_solver = p_solver;
}

// ----------------------------------------------------------------------------
void TeachPendant::setEndEffector(Node const& p_end_effector)
{
    m_end_effector = &p_end_effector;
}

// ----------------------------------------------------------------------------
bool TeachPendant::moveJoint(size_t p_joint_idx, double p_delta, double p_speed)
{
    if (!m_robot || !m_controlled_robot)
    {
        m_error = "Robot or controlled robot not set";
        return false;
    }

    // Impossible to move while playing a trajectory
    if (m_controlled_robot->state ==
        application::ControlledRobot::State::PLAYING)
    {
        m_error = "Impossible to move while playing a trajectory";
        return false;
    }

    // Check that the index is valid
    if (p_joint_idx >= m_robot->blueprint().numJoints())
    {
        m_error = "Invalid joint index";
        return false;
    }

    // Calculate the new target position
    auto target = m_robot->states().joint_positions;
    target[p_joint_idx] += p_delta * m_controlled_robot->speed_factor * p_speed;

    // Check the limits via forEachJoint
    bool within_limits = true;
    m_robot->blueprint().forEachJoint(
        [&](Joint const& joint, size_t index)
        {
            if (index == p_joint_idx)
            {
                auto [min, max] = joint.limits();
                if (target[index] < min || target[index] > max)
                {
                    within_limits = false;
                }
            }
        });

    if (!within_limits)
    {
        m_error = "Target position is out of limits";
        return false;
    }

    // Apply the movement
    m_controlled_robot->state =
        application::ControlledRobot::State::MANUAL_CONTROL;
    m_robot->setJointPositions(target);

    return true;
}

// ----------------------------------------------------------------------------
bool TeachPendant::moveJoints(const std::vector<double>& p_deltas,
                              double p_speed)
{
    if (!m_robot || !m_controlled_robot)
    {
        m_error = "Robot or controlled robot not set";
        return false;
    }

    // Impossible to move while playing a trajectory
    if (m_controlled_robot->state ==
        application::ControlledRobot::State::PLAYING)
    {
        m_error = "Impossible to move while playing a trajectory";
        return false;
    }

    // Check that the number of deltas corresponds to the number of joints
    if (p_deltas.size() != m_robot->blueprint().numJoints())
    {
        m_error =
            "Number of deltas does not correspond to the number of joints";
        return false;
    }

    // Calculate the new target positions
    auto target = m_robot->states().joint_positions;
    for (size_t i = 0; i < p_deltas.size(); ++i)
    {
        target[i] += p_deltas[i] * m_controlled_robot->speed_factor * p_speed;
    }

    // Check all limits
    bool within_limits = true;
    m_robot->blueprint().forEachJoint(
        [&](Joint const& joint, size_t index)
        {
            auto [min, max] = joint.limits();
            if (target[index] < min || target[index] > max)
            {
                within_limits = false;
            }
        });

    if (!within_limits)
    {
        m_error = "Target position is out of limits";
        return false;
    }

    // Apply the movement
    m_controlled_robot->state =
        application::ControlledRobot::State::MANUAL_CONTROL;
    m_robot->setJointPositions(target);

    return true;
}

// ----------------------------------------------------------------------------
bool TeachPendant::moveCartesian(const Eigen::Vector3d& p_translation,
                                 double p_speed)
{
    if (!m_robot || !m_controlled_robot || !m_end_effector || !m_ik_solver)
    {
        m_error = "Robot, controlled robot, end effector or IK solver not set";
        return false;
    }

    // Check that the control mode is Cartesian
    if (m_controlled_robot->control_mode !=
        application::ControlledRobot::ControlMode::CARTESIAN)
    {
        m_error = "Control mode is not Cartesian";
        return false;
    }

    // Impossible to move while playing a trajectory
    if (m_controlled_robot->state ==
        application::ControlledRobot::State::PLAYING)
    {
        m_error = "Impossible to move while playing a trajectory";
        return false;
    }

    // Get the current transform of the end effector
    Transform current_transform = m_end_effector->worldTransform();

    // Apply the translation
    current_transform.block<3, 1>(0, 3) +=
        p_translation * m_controlled_robot->speed_factor * p_speed;

    // Convert to 6D pose for the IK solver
    Pose target_pose;
    target_pose.head<3>() = current_transform.block<3, 1>(0, 3);

    // Extract the Euler angles from the rotation
    Eigen::Matrix3d R = current_transform.block<3, 3>(0, 0);
    target_pose.tail<3>() = R.eulerAngles(0, 1, 2);

    // Solve the inverse kinematics
    if (!m_ik_solver->solve(*m_robot, *m_end_effector, target_pose))
    {
        m_error = "Failed to solve the inverse kinematics";
        return false;
    }

    // Apply the IK solution
    m_robot->setJointPositions(m_ik_solver->solution());
    m_controlled_robot->state =
        application::ControlledRobot::State::MANUAL_CONTROL;

    return true;
}

// ----------------------------------------------------------------------------
bool TeachPendant::rotateCartesian(const Eigen::Vector3d& p_rotation_axis,
                                   double p_angle,
                                   double p_speed)
{
    if (!m_robot || !m_controlled_robot || !m_end_effector || !m_ik_solver)
    {
        m_error = "Robot, controlled robot, end effector or IK solver not set";
        return false;
    }

    // Check that the control mode is Cartesian
    if (m_controlled_robot->control_mode !=
        application::ControlledRobot::ControlMode::CARTESIAN)
    {
        m_error = "Control mode is not Cartesian";
        return false;
    }

    // Impossible to move while playing a trajectory
    if (m_controlled_robot->state ==
        application::ControlledRobot::State::PLAYING)
    {
        m_error = "Impossible to move while playing a trajectory";
        return false;
    }

    // Get the current transform of the end effector
    Transform current_transform = m_end_effector->worldTransform();

    // Create the rotation via Rodrigues (AngleAxis)
    Eigen::AngleAxisd rotation(p_angle * m_controlled_robot->speed_factor *
                                   p_speed,
                               p_rotation_axis.normalized());

    // Apply the rotation to the rotational part of the transformation
    Eigen::Matrix3d R = current_transform.block<3, 3>(0, 0);
    R = rotation.toRotationMatrix() * R;
    current_transform.block<3, 3>(0, 0) = R;

    // Convert to 6D pose for the IK solver
    Pose target_pose;
    target_pose.head<3>() = current_transform.block<3, 1>(0, 3);
    target_pose.tail<3>() = R.eulerAngles(0, 1, 2);

    // Solve the inverse kinematics
    if (!m_ik_solver->solve(*m_robot, *m_end_effector, target_pose))
    {
        m_error = "Failed to solve the inverse kinematics";
        return false;
    }

    // Apply the IK solution
    m_robot->setJointPositions(m_ik_solver->solution());
    m_controlled_robot->state =
        application::ControlledRobot::State::MANUAL_CONTROL;

    return true;
}

// ----------------------------------------------------------------------------
size_t TeachPendant::recordWaypoint(const std::string& p_label)
{
    if (!m_robot || !m_controlled_robot)
    {
        m_error = "Robot or controlled robot not set";
        return 0;
    }

    // Create a waypoint with the current position
    Trajectory::States waypoint;
    waypoint.position = m_robot->states().joint_positions;
    waypoint.velocity.resize(waypoint.position.size(), 0.0);
    waypoint.acceleration.resize(waypoint.position.size(), 0.0);
    waypoint.time = 0.0; // Will be recalculated during playback

    // Add to the robot's waypoints
    m_controlled_robot->waypoints.push_back(waypoint);

    std::cout << "📍 Recorded waypoint "
              << m_controlled_robot->waypoints.size() - 1 << ": "
              << (p_label.empty() ? "unnamed" : p_label) << std::endl;

    return m_controlled_robot->waypoints.size() - 1;
}

// ----------------------------------------------------------------------------
void TeachPendant::deleteWaypoint(size_t p_idx)
{
    if (!m_controlled_robot)
        return;

    if (p_idx < m_controlled_robot->waypoints.size())
    {
        m_controlled_robot->waypoints.erase(
            m_controlled_robot->waypoints.begin() + p_idx);
    }
}

// ----------------------------------------------------------------------------
void TeachPendant::clearWaypoints()
{
    if (!m_controlled_robot)
        return;

    m_controlled_robot->waypoints.clear();
}

// ----------------------------------------------------------------------------
bool TeachPendant::goToWaypoint(size_t p_idx, double p_duration)
{
    if (!m_robot || !m_controlled_robot)
        return false;

    // Check that the index is valid
    if (p_idx >= m_controlled_robot->waypoints.size())
        return false;

    // Create a trajectory between the current position and the waypoint
    auto const& target_waypoint = m_controlled_robot->waypoints[p_idx];
    auto current_pos = m_robot->states().joint_positions;

    m_controlled_robot->playing_trajectory =
        std::make_unique<JointSpaceTrajectory>(
            current_pos, target_waypoint.position, p_duration);

    m_controlled_robot->trajectory_time = 0.0;
    m_controlled_robot->state = application::ControlledRobot::State::PLAYING;

    std::cout << "🎯 Going to waypoint " << p_idx << std::endl;

    return true;
}

// ----------------------------------------------------------------------------
bool TeachPendant::playRecordedTrajectory(bool p_loop)
{
    if (!m_robot || !m_controlled_robot)
    {
        m_error = "Robot or controlled robot not set";
        return false;
    }

    // Check that there are at least 2 waypoints
    if (m_controlled_robot->waypoints.size() < 2)
    {
        std::cout << "⚠️ Need at least 2 waypoints to play trajectory"
                  << std::endl;
        return false;
    }

    // For now: simple trajectory between first and last waypoint
    // TODO: Create a multi-segment trajectory passing through all waypoints
    auto const& wps = m_controlled_robot->waypoints;
    double duration_per_segment = 3.0; // 3 secondes entre waypoints

    m_controlled_robot->playing_trajectory =
        std::make_unique<JointSpaceTrajectory>(wps.front().position,
                                               wps.back().position,
                                               duration_per_segment *
                                                   (wps.size() - 1));

    m_controlled_robot->trajectory_time = 0.0;
    m_controlled_robot->state = application::ControlledRobot::State::PLAYING;

    std::cout << "▶️ Playing trajectory with " << wps.size() << " waypoints"
              << std::endl;

    return true;
}

// ----------------------------------------------------------------------------
void TeachPendant::stopTrajectory()
{
    if (!m_controlled_robot)
        return;

    m_controlled_robot->playing_trajectory.reset();
    m_controlled_robot->state = application::ControlledRobot::State::IDLE;

    std::cout << "⏹️ Trajectory stopped" << std::endl;
}

// ----------------------------------------------------------------------------
void TeachPendant::update(double p_dt)
{
    if (!m_robot || !m_controlled_robot)
        return;

    // Check that we are in PLAYING mode with a trajectory
    if (m_controlled_robot->state !=
        application::ControlledRobot::State::PLAYING)
    {
        m_error = "Not in PLAYING mode with a trajectory";
        return;
    }

    if (!m_controlled_robot->playing_trajectory)
    {
        m_error = "No trajectory to play";
        return;
    }

    // Advance in time
    m_controlled_robot->trajectory_time += p_dt;

    // Check if the trajectory is finished
    if (m_controlled_robot->trajectory_time >=
        m_controlled_robot->playing_trajectory->duration())
    {
        // The trajectory is finished
        std::cout << "✓ Trajectory completed" << std::endl;
        m_controlled_robot->playing_trajectory.reset();
        m_controlled_robot->state = application::ControlledRobot::State::IDLE;
        return;
    }

    // Evaluate the target position at this time
    auto target_positions = m_controlled_robot->playing_trajectory->getPosition(
        m_controlled_robot->trajectory_time);

    // Apply with speed limits
    m_robot->applyJointTargetsWithSpeedLimit(target_positions, p_dt);
}

} // namespace robotik
