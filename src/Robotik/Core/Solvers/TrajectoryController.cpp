/**
 * @file TrajectoryController.cpp
 * @brief Implementation of TrajectoryController.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Solvers/TrajectoryController.hpp"

#include <iostream>

namespace robotik
{

// ----------------------------------------------------------------------------
bool TrajectoryController::goToWaypoint(JointValues const& p_current_position,
                                        JointValues const& p_target_position,
                                        double p_duration)
{
    if (p_current_position.size() != p_target_position.size())
    {
        std::cerr << "⚠️ Position size mismatch" << std::endl;
        return false;
    }

    if (p_duration <= 0.0)
    {
        std::cerr << "⚠️ Invalid duration" << std::endl;
        return false;
    }

    // Create trajectory
    m_trajectory = std::make_unique<JointSpaceTrajectory>(
        p_current_position, p_target_position, p_duration);

    m_time = 0.0;
    m_mode = Mode::GO_TO_WAYPOINT;
    m_current_waypoint_index = -1;
    m_target_waypoint_index = 0;
    m_loop = false;
    m_waypoints.clear();

    std::cout << "🎯 Going to waypoint" << std::endl;

    return true;
}

// ----------------------------------------------------------------------------
bool TrajectoryController::playWaypoints(
    JointValues const& p_current_position,
    std::vector<WaypointManager::Waypoint> const& p_waypoints,
    bool p_loop)
{
    if (p_waypoints.empty())
    {
        std::cerr << "⚠️ No waypoints to play" << std::endl;
        return false;
    }

    // Store waypoints for sequence playback
    m_waypoints = p_waypoints;
    m_loop = p_loop;

    // Create trajectory from current position to first waypoint
    m_trajectory = std::make_unique<JointSpaceTrajectory>(
        p_current_position, m_waypoints[0].position, m_waypoints[0].duration);

    m_time = 0.0;
    m_mode = Mode::PLAY_SEQUENCE;
    m_current_waypoint_index = -1; // Coming from current position
    m_target_waypoint_index = 0;

    std::cout << "▶️ Playing trajectory with " << m_waypoints.size()
              << " waypoints" << std::endl;

    return true;
}

// ----------------------------------------------------------------------------
void TrajectoryController::stop()
{
    m_trajectory.reset();
    m_mode = Mode::IDLE;
    m_current_waypoint_index = -1;
    m_target_waypoint_index = -1;
    m_loop = false;
    m_waypoints.clear();
    m_time = 0.0;

    std::cout << "⏹️ Trajectory stopped" << std::endl;
}

// ----------------------------------------------------------------------------
void TrajectoryController::update(double p_dt)
{
    if (!isPlaying())
        return;

    // Advance in time
    m_time += p_dt;

    // Check if current trajectory segment is finished
    if (m_trajectory && m_time >= m_trajectory->duration())
    {
        if (m_mode == Mode::GO_TO_WAYPOINT)
        {
            // Single waypoint reached - stop
            std::cout << "✓ Waypoint reached" << std::endl;
            stop();
            return;
        }
        else if (m_mode == Mode::PLAY_SEQUENCE)
        {
            // Move to next waypoint in sequence
            m_current_waypoint_index = m_target_waypoint_index;

            // Check if we've reached the last waypoint
            if (static_cast<size_t>(m_target_waypoint_index) ==
                m_waypoints.size() - 1)
            {
                if (m_loop)
                {
                    // Loop back to first waypoint
                    m_target_waypoint_index = 0;
                    std::cout << "🔄 Looping back to waypoint 0" << std::endl;
                }
                else
                {
                    // Sequence finished
                    std::cout << "✓ Trajectory sequence completed" << std::endl;
                    stop();
                    return;
                }
            }
            else
            {
                // Move to next waypoint
                m_target_waypoint_index++;
            }

            // Create trajectory to next waypoint
            createNextTrajectory();
        }
    }
}

// ----------------------------------------------------------------------------
bool TrajectoryController::isPlaying() const
{
    return m_mode != Mode::IDLE && m_trajectory != nullptr;
}

// ----------------------------------------------------------------------------
bool TrajectoryController::isFinished() const
{
    if (!m_trajectory)
        return true;

    return m_time >= m_trajectory->duration();
}

// ----------------------------------------------------------------------------
JointValues TrajectoryController::getCurrentTarget() const
{
    if (!m_trajectory)
        return JointValues();

    // Clamp time to trajectory duration
    double t = std::min(m_time, m_trajectory->duration());
    return m_trajectory->getPosition(t);
}

// ----------------------------------------------------------------------------
void TrajectoryController::createNextTrajectory()
{
    if (m_waypoints.empty())
        return;

    if (m_current_waypoint_index == -1)
    {
        // Should not happen, but handle it
        std::cerr << "⚠️ Invalid waypoint index" << std::endl;
        stop();
        return;
    }

    // Create trajectory from current to target waypoint
    m_trajectory = std::make_unique<JointSpaceTrajectory>(
        m_waypoints[static_cast<size_t>(m_current_waypoint_index)].position,
        m_waypoints[static_cast<size_t>(m_target_waypoint_index)].position,
        m_waypoints[static_cast<size_t>(m_target_waypoint_index)].duration);

    m_time = 0.0;

    std::cout << "✓ Waypoint " << m_current_waypoint_index
              << " reached, moving to waypoint " << m_target_waypoint_index
              << std::endl;
}

} // namespace robotik
