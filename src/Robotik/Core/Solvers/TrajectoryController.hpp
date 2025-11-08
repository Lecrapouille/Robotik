/**
 * @file TrajectoryController.hpp
 * @brief Controller for executing robot trajectories.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/WaypointManager.hpp"
#include "Robotik/Core/Solvers/Trajectory.hpp"

#include <memory>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! \brief Controller for executing robot trajectories.
//!
//! This class manages trajectory execution, including single waypoint
//! movements and sequences of waypoints. It is completely independent
//! of Robot class and only returns target joint positions.
// ****************************************************************************
class TrajectoryController
{
public:

    // ************************************************************************
    //! \brief Execution mode.
    // ************************************************************************
    enum class Mode
    {
        IDLE,           //!< No trajectory playing
        GO_TO_WAYPOINT, //!< Moving to a single waypoint
        PLAY_SEQUENCE   //!< Playing a sequence of waypoints
    };

    // ------------------------------------------------------------------------
    //! \brief Default constructor.
    // ------------------------------------------------------------------------
    TrajectoryController() = default;

    // ------------------------------------------------------------------------
    //! \brief Start going to a single waypoint.
    //! \param p_current_position Current joint positions.
    //! \param p_target_position Target joint positions.
    //! \param p_duration Duration of the movement (s).
    //! \return true if trajectory was started successfully.
    // ------------------------------------------------------------------------
    bool goToWaypoint(JointValues const& p_current_position,
                      JointValues const& p_target_position,
                      double p_duration);

    // ------------------------------------------------------------------------
    //! \brief Start playing a sequence of waypoints.
    //! \param p_current_position Current joint positions.
    //! \param p_waypoints Sequence of waypoints to play.
    //! \param p_loop Whether to loop the sequence.
    //! \return true if playback was started successfully.
    // ------------------------------------------------------------------------
    bool
    playWaypoints(JointValues const& p_current_position,
                  std::vector<WaypointManager::Waypoint> const& p_waypoints,
                  bool p_loop = false);

    // ------------------------------------------------------------------------
    //! \brief Stop the current trajectory.
    // ------------------------------------------------------------------------
    void stop();

    // ------------------------------------------------------------------------
    //! \brief Update the controller (advance in time).
    //! \param p_dt Time step (s).
    // ------------------------------------------------------------------------
    void update(double p_dt);

    // ------------------------------------------------------------------------
    //! \brief Check if a trajectory is currently playing.
    //! \return true if playing.
    // ------------------------------------------------------------------------
    bool isPlaying() const;

    // ------------------------------------------------------------------------
    //! \brief Check if the trajectory is finished.
    //! \return true if finished.
    // ------------------------------------------------------------------------
    bool isFinished() const;

    // ------------------------------------------------------------------------
    //! \brief Get the current target joint positions.
    //! \return Target joint positions at current time.
    // ------------------------------------------------------------------------
    JointValues getCurrentTarget() const;

    // ------------------------------------------------------------------------
    //! \brief Get the current mode.
    //! \return Current execution mode.
    // ------------------------------------------------------------------------
    Mode mode() const
    {
        return m_mode;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current waypoint index.
    //! \return Current waypoint index (-1 if coming from current position).
    // ------------------------------------------------------------------------
    int currentWaypointIndex() const
    {
        return m_current_waypoint_index;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the target waypoint index.
    //! \return Target waypoint index.
    // ------------------------------------------------------------------------
    int targetWaypointIndex() const
    {
        return m_target_waypoint_index;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Create trajectory to next waypoint in sequence.
    // ------------------------------------------------------------------------
    void createNextTrajectory();

private:

    //! \brief Current trajectory being executed
    std::unique_ptr<Trajectory> m_trajectory;
    //! \brief Current time in the trajectory
    double m_time = 0.0;
    //! \brief Current execution mode
    Mode m_mode = Mode::IDLE;
    //! \brief Current waypoint index (-1 if from current position)
    int m_current_waypoint_index = -1;
    //! \brief Target waypoint index
    int m_target_waypoint_index = -1;
    //! \brief Whether to loop the sequence
    bool m_loop = false;
    //! \brief Waypoints for sequence playback
    std::vector<WaypointManager::Waypoint> m_waypoints;
};

} // namespace robotik
