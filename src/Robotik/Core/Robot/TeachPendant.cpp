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
#include "Robotik/Core/Common/Conversions.hpp"
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
        [&target, &within_limits, p_joint_idx](Joint const& joint, size_t index)
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
        [&target, &within_limits](Joint const& joint, size_t index)
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

    // Transform translation vector from frame to world coordinates
    Eigen::Vector3d translation_world = p_translation;
    if (m_controlled_robot->cartesian_frame != nullptr)
    {
        // Get rotation matrix from frame to world
        Eigen::Matrix3d R_frame =
            m_controlled_robot->cartesian_frame->worldTransform().block<3, 3>(
                0, 0);
        // Transform direction vector from frame to world
        translation_world = R_frame * p_translation;
    }

    // Apply the translation
    current_transform.block<3, 1>(0, 3) +=
        translation_world * m_controlled_robot->speed_factor * p_speed;

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

    // Transform rotation axis from frame to world coordinates
    Eigen::Vector3d rotation_axis_world = p_rotation_axis.normalized();
    if (m_controlled_robot->cartesian_frame != nullptr)
    {
        // Get rotation matrix from frame to world
        Eigen::Matrix3d R_frame =
            m_controlled_robot->cartesian_frame->worldTransform().block<3, 3>(
                0, 0);
        // Transform direction vector from frame to world
        rotation_axis_world = R_frame * p_rotation_axis;
        rotation_axis_world.normalize();
    }

    // Create the rotation via Rodrigues (AngleAxis)
    Eigen::AngleAxisd rotation(p_angle * m_controlled_robot->speed_factor *
                                   p_speed,
                               rotation_axis_world);

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
size_t TeachPendant::recordWaypoint(const std::string& p_label,
                                    double p_duration)
{
    if (!m_robot || !m_controlled_robot)
    {
        m_error = "Robot or controlled robot not set";
        return 0;
    }

    // Create a waypoint with the current position
    application::ControlledRobot::Waypoint waypoint;
    waypoint.states.position = m_robot->states().joint_positions;
    waypoint.states.velocity.resize(waypoint.states.position.size(), 0.0);
    waypoint.states.acceleration.resize(waypoint.states.position.size(), 0.0);
    waypoint.states.time = 0.0; // Will be recalculated during playback

    // Store duration
    waypoint.duration = p_duration;
    m_controlled_robot->waypoints.push_back(waypoint);

    // Store label (default to auto-generated name if empty)
    // Calculate index after push_back to get correct index
    size_t waypoint_index = m_controlled_robot->waypoints.size() - 1;
    m_controlled_robot->waypoints[waypoint_index].label =
        p_label.empty() ? ("waypoint_" + std::to_string(waypoint_index))
                        : p_label;

    std::cout << "📍 Recorded waypoint "
              << m_controlled_robot->waypoints.size() - 1 << ": " << p_label
              << " (duration: " << p_duration << "s)" << std::endl;

    return m_controlled_robot->waypoints.size() - 1;
}

// ----------------------------------------------------------------------------
void TeachPendant::deleteWaypoint(size_t p_idx)
{
    if (!m_controlled_robot || p_idx >= m_controlled_robot->waypoints.size())
        return;

    // Adjust current_waypoint_index if necessary
    if (m_controlled_robot->current_waypoint_index != -1)
    {
        int current_idx = m_controlled_robot->current_waypoint_index;
        if (static_cast<size_t>(current_idx) == p_idx)
        {
            // The current waypoint is being deleted, reset to -1
            m_controlled_robot->current_waypoint_index = -1;
        }
        else if (static_cast<size_t>(current_idx) > p_idx)
        {
            // A waypoint before current was deleted, decrement
            m_controlled_robot->current_waypoint_index = current_idx - 1;
        }
    }

    // Adjust target_waypoint_index if necessary
    if (m_controlled_robot->target_waypoint_index == p_idx)
    {
        // The target waypoint is being deleted, stop the trajectory
        if (m_controlled_robot->state ==
            application::ControlledRobot::State::PLAYING)
        {
            m_controlled_robot->trajectory.reset();
            m_controlled_robot->state =
                application::ControlledRobot::State::IDLE;
        }
    }
    else if (m_controlled_robot->target_waypoint_index > p_idx)
    {
        // A waypoint before target was deleted, decrement
        m_controlled_robot->target_waypoint_index--;
    }

    // Check if indices become invalid after deletion
    size_t new_size = m_controlled_robot->waypoints.size() - 1;
    if ((m_controlled_robot->target_waypoint_index >= new_size ||
         (m_controlled_robot->current_waypoint_index != -1 &&
          static_cast<size_t>(m_controlled_robot->current_waypoint_index) >=
              new_size)) &&
        m_controlled_robot->state ==
            application::ControlledRobot::State::PLAYING)
    {
        // Indices are invalid, stop the trajectory
        m_controlled_robot->trajectory.reset();
        m_controlled_robot->state = application::ControlledRobot::State::IDLE;
    }

    // Delete the waypoint
    m_controlled_robot->waypoints.erase(m_controlled_robot->waypoints.begin() +
                                        p_idx);
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
    auto const& current_pos = m_robot->states().joint_positions;

    m_controlled_robot->trajectory = std::make_unique<JointSpaceTrajectory>(
        current_pos, target_waypoint.states.position, p_duration);

    m_controlled_robot->trajectory_current_time = 0.0;
    m_controlled_robot->current_waypoint_index =
        -1; // Coming from current position
    m_controlled_robot->target_waypoint_index = p_idx;
    m_controlled_robot->is_playing_all_waypoints =
        false; // "go" mode: single waypoint
    m_controlled_robot->state = application::ControlledRobot::State::PLAYING;

    std::cout << "🎯 Going to waypoint " << p_idx << std::endl;

    return true;
}

// ----------------------------------------------------------------------------
bool TeachPendant::playRecordedTrajectory()
{
    if (!m_robot || !m_controlled_robot)
    {
        m_error = "Robot or controlled robot not set";
        return false;
    }

    // Check that there are at least 1 waypoint
    if (m_controlled_robot->waypoints.empty())
    {
        std::cout << "⚠️ Need at least 1 waypoint to play trajectory"
                  << std::endl;
        return false;
    }

    // The robot must first reach waypoint 0 (initial waypoint)
    auto const& wps = m_controlled_robot->waypoints;
    auto const& current_pos = m_robot->states().joint_positions;

    // Create trajectory from current position to waypoint 0
    m_controlled_robot->trajectory = std::make_unique<JointSpaceTrajectory>(
        current_pos, wps[0].states.position, wps[0].duration);

    m_controlled_robot->trajectory_current_time = 0.0;
    m_controlled_robot->current_waypoint_index =
        -1; // Coming from current position
    m_controlled_robot->target_waypoint_index = 0; // Destination: waypoint 0
    m_controlled_robot->is_playing_all_waypoints =
        true; // "play" mode: all waypoints
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

    m_controlled_robot->current_waypoint_index = -1; // Reset to initial state
    m_controlled_robot->is_playing_all_waypoints = false; // Reset flag
    m_controlled_robot->trajectory.reset();
    m_controlled_robot->state = application::ControlledRobot::State::IDLE;

    std::cout << "⏹️ Trajectory stopped" << std::endl;
}

// ----------------------------------------------------------------------------
bool TeachPendant::isWaypointReached(application::ControlledRobot* p_robot,
                                     size_t p_waypoint_index)
{
    if (!p_robot || p_waypoint_index >= p_robot->waypoints.size())
        return false;

    // Check that end-effector is configured
    if (!p_robot->end_effector)
        return false;

    // Get current end-effector pose
    Transform current_transform = p_robot->end_effector->worldTransform();
    Pose current_pose = transformToPose(current_transform);

    // Save current joint positions
    auto current_joint_positions = p_robot->states().joint_positions;

    // Apply waypoint joint positions temporarily
    auto const& waypoint = p_robot->waypoints[p_waypoint_index];
    if (waypoint.states.position.size() != current_joint_positions.size())
        return false;

    p_robot->setJointPositions(waypoint.states.position);

    // Calculate target pose
    Transform target_transform = p_robot->end_effector->worldTransform();
    Pose target_pose = transformToPose(target_transform);

    // Restore original joint positions
    p_robot->setJointPositions(current_joint_positions);

    // Calculate pose error
    Pose error = calculatePoseError(target_pose, current_pose);

    // Check if error is within tolerance
    // Position tolerance: 0.01 m
    // Orientation tolerance: 0.01 rad
    double position_error = error.head<3>().norm();
    double orientation_error = error.tail<3>().norm();

    return (position_error < 0.01 && orientation_error < 0.01);
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

    if (!m_controlled_robot->trajectory)
    {
        m_error = "No trajectory to play";
        return;
    }

    // Check if the target waypoint has been reached
    if (m_controlled_robot->target_waypoint_index <
            m_controlled_robot->waypoints.size() &&
        isWaypointReached(m_controlled_robot,
                          m_controlled_robot->target_waypoint_index))
    {
        // Waypoint reached
        auto const& wps = m_controlled_robot->waypoints;
        size_t target_idx = m_controlled_robot->target_waypoint_index;

        // Check if we reached the last waypoint
        if (target_idx == wps.size() - 1)
        {
            // Last waypoint reached
            if (m_controlled_robot->play_in_loop)
            {
                // Loop: go back to waypoint 0
                m_controlled_robot->current_waypoint_index =
                    static_cast<int>(wps.size() - 1);
                m_controlled_robot->target_waypoint_index = 0;

                // Create trajectory from last waypoint to first waypoint
                m_controlled_robot->trajectory =
                    std::make_unique<JointSpaceTrajectory>(
                        wps[wps.size() - 1].states.position,
                        wps[0].states.position,
                        wps[0].duration);

                m_controlled_robot->trajectory_current_time = 0.0;
                std::cout << "✓ Waypoint " << target_idx
                          << " reached, looping to waypoint 0" << std::endl;
            }
            else
            {
                // No loop: stop trajectory
                std::cout << "✓ Trajectory completed (all waypoints reached)"
                          << std::endl;
                m_controlled_robot->current_waypoint_index =
                    -1; // Reset to initial state
                m_controlled_robot->is_playing_all_waypoints =
                    false; // Reset flag
                m_controlled_robot->trajectory.reset();
                m_controlled_robot->state =
                    application::ControlledRobot::State::IDLE;
                return;
            }
        }
        else
        {
            // Check if we're playing all waypoints or just going to one
            if (!m_controlled_robot->is_playing_all_waypoints)
            {
                // We're doing "go" to a single waypoint, stop when reached
                m_controlled_robot->current_waypoint_index =
                    static_cast<int>(target_idx);
                m_controlled_robot->trajectory.reset();
                m_controlled_robot->state =
                    application::ControlledRobot::State::IDLE;
                std::cout << "✓ Waypoint " << target_idx
                          << " reached (go completed)" << std::endl;
                return;
            }

            // Move to next waypoint
            m_controlled_robot->current_waypoint_index =
                static_cast<int>(target_idx);
            m_controlled_robot->target_waypoint_index = target_idx + 1;

            // Create trajectory to next waypoint
            if (m_controlled_robot->current_waypoint_index == -1)
            {
                // Coming from current position
                auto const& current_pos = m_robot->states().joint_positions;
                m_controlled_robot
                    ->trajectory = std::make_unique<JointSpaceTrajectory>(
                    current_pos,
                    wps[m_controlled_robot->target_waypoint_index]
                        .states.position,
                    wps[m_controlled_robot->target_waypoint_index].duration);
            }
            else
            {
                // Coming from a waypoint
                m_controlled_robot
                    ->trajectory = std::make_unique<JointSpaceTrajectory>(
                    wps[static_cast<size_t>(
                            m_controlled_robot->current_waypoint_index)]
                        .states.position,
                    wps[m_controlled_robot->target_waypoint_index]
                        .states.position,
                    wps[m_controlled_robot->target_waypoint_index].duration);
            }

            m_controlled_robot->trajectory_current_time = 0.0;
            std::cout << "✓ Waypoint " << target_idx
                      << " reached, moving to waypoint "
                      << m_controlled_robot->target_waypoint_index << std::endl;
        }
    }

    // Advance in time
    m_controlled_robot->trajectory_current_time += p_dt;

    // Check if the trajectory is finished (time-based fallback)
    if (m_controlled_robot->trajectory &&
        m_controlled_robot->trajectory_current_time >=
            m_controlled_robot->trajectory->duration())
    {
        // Trajectory time exceeded, but waypoint might not be reached yet
        // Continue executing until waypoint is reached
        m_controlled_robot->trajectory_current_time =
            m_controlled_robot->trajectory->duration();
    }

    // Evaluate the target position at this time
    if (m_controlled_robot->trajectory)
    {
        auto target_positions = m_controlled_robot->trajectory->getPosition(
            m_controlled_robot->trajectory_current_time);

        // Apply with speed limits
        m_robot->applyJointTargetsWithSpeedLimit(target_positions, p_dt);
    }
}

} // namespace robotik
