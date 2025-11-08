/**
 * @file WaypointManager.hpp
 * @brief Manager for robot waypoints organized by end effector.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Common/Types.hpp"
#include "Robotik/Core/Robot/Robot.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! \brief Manager for robot waypoints.
//!
//! This class manages waypoints for a robot, organized by end effector.
//! Each waypoint stores joint positions and can be used for trajectory
//! generation.
// ****************************************************************************
class WaypointManager
{
public:

    // ************************************************************************
    //! \brief Waypoint structure containing joint states and metadata.
    // ************************************************************************
    struct Waypoint
    {
        JointValues position;     //!< Joint positions
        JointValues velocity;     //!< Joint velocities
        JointValues acceleration; //!< Joint accelerations
        std::string label;        //!< Waypoint label/name
        double duration;          //!< Duration to reach this waypoint (s)
    };

    // ------------------------------------------------------------------------
    //! \brief Default constructor.
    // ------------------------------------------------------------------------
    WaypointManager() = default;

    // ------------------------------------------------------------------------
    //! \brief Add a waypoint with the current robot state.
    //! \param p_robot Robot to record the waypoint from.
    //! \param p_end_effector End effector associated with this waypoint.
    //! \param p_label Label for the waypoint.
    //! \param p_duration Duration to reach this waypoint (s).
    //! \return Index of the added waypoint for this end effector.
    // ------------------------------------------------------------------------
    size_t addWaypoint(Robot const& p_robot,
                       Node const* p_end_effector,
                       std::string const& p_label,
                       double p_duration);

    // ------------------------------------------------------------------------
    //! \brief Delete a waypoint.
    //! \param p_end_effector End effector whose waypoint to delete.
    //! \param p_index Index of the waypoint to delete.
    // ------------------------------------------------------------------------
    void deleteWaypoint(Node const* p_end_effector, size_t p_index);

    // ------------------------------------------------------------------------
    //! \brief Clear all waypoints for a specific end effector.
    //! \param p_end_effector End effector whose waypoints to clear.
    // ------------------------------------------------------------------------
    void clearWaypoints(Node const* p_end_effector);

    // ------------------------------------------------------------------------
    //! \brief Clear all waypoints for all end effectors.
    // ------------------------------------------------------------------------
    void clearAllWaypoints();

    // ------------------------------------------------------------------------
    //! \brief Get waypoints for a specific end effector.
    //! \param p_end_effector End effector.
    //! \return Vector of waypoints (const reference).
    // ------------------------------------------------------------------------
    std::vector<Waypoint> const& getWaypoints(Node const* p_end_effector) const;

    // ------------------------------------------------------------------------
    //! \brief Get the number of waypoints for a specific end effector.
    //! \param p_end_effector End effector.
    //! \return Number of waypoints.
    // ------------------------------------------------------------------------
    size_t size(Node const* p_end_effector) const;

    // ------------------------------------------------------------------------
    //! \brief Check if waypoints exist for a specific end effector.
    //! \param p_end_effector End effector.
    //! \return true if waypoints exist.
    // ------------------------------------------------------------------------
    bool hasWaypoints(Node const* p_end_effector) const;

    // ------------------------------------------------------------------------
    //! \brief Check if a waypoint has been reached using pose error.
    //! \param p_robot The robot to check.
    //! \param p_end_effector The end effector to check.
    //! \param p_waypoint The waypoint to check against.
    //! \param p_position_tolerance Position tolerance (m).
    //! \param p_orientation_tolerance Orientation tolerance (rad).
    //! \return true if the waypoint is reached (within tolerance).
    // ------------------------------------------------------------------------
    static bool isWaypointReached(Robot const& p_robot,
                                  Node const* p_end_effector,
                                  Waypoint const& p_waypoint,
                                  double p_position_tolerance = 0.01,
                                  double p_orientation_tolerance = 0.01);

private:

    //! \brief Waypoints organized by end effector
    std::unordered_map<Node const*, std::vector<Waypoint>> m_waypoints;

    //! \brief Empty vector for when no waypoints exist
    static std::vector<Waypoint> const s_empty_waypoints;
};

} // namespace robotik
