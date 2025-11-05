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

// Forward declaration
namespace application
{
class ControlledRobot;
}

// ****************************************************************************
//! \brief Virtual teach pendant for interactive control of the robot.
//!
//! This class implements a virtual teach pendant for interactive control of the
//! robot.
//! - Control the robot in joint space mode.
//! - Control the robot in Cartesian mode with IK.
//! - Record waypoints (key positions).
//! - Playback trajectories via waypoints.
// ****************************************************************************
class TeachPendant
{
public:

    // ------------------------------------------------------------------------
    //! \brief Default constructor.
    // ------------------------------------------------------------------------
    TeachPendant();

    // ------------------------------------------------------------------------
    //! \brief Configure the robot to operate on.
    //! \param p_robot Controlled robot.
    // ------------------------------------------------------------------------
    void setRobot(application::ControlledRobot& p_robot);

    // ------------------------------------------------------------------------
    //! \brief Configure the IK solver.
    //! \param p_solver Pointer to the IK solver (not owned).
    // ------------------------------------------------------------------------
    void setIKSolver(IKSolver* p_solver);

    // ------------------------------------------------------------------------
    //! \brief Configure the end effector for Cartesian control.
    //! \param p_end_effector Reference to the end effector node.
    // ------------------------------------------------------------------------
    void setEndEffector(Node const& p_end_effector);

    // ------------------------------------------------------------------------
    //! \brief Move a specific joint.
    //! \param p_joint_idx Index of the joint.
    //! \param p_delta Position variation (rad or m).
    //! \param p_speed Speed factor.
    //! \return true if the movement has been applied.
    // ------------------------------------------------------------------------
    bool moveJoint(size_t p_joint_idx, double p_delta, double p_speed = 1.0);

    // ------------------------------------------------------------------------
    //! \brief Move multiple joints simultaneously.
    //! \param p_deltas Position variations for each joint.
    //! \param p_speed Speed factor.
    //! \return true if the movement has been applied.
    // ------------------------------------------------------------------------
    bool moveJoints(const std::vector<double>& p_deltas, double p_speed = 1.0);

    // ------------------------------------------------------------------------
    //! \brief Move the end effector in Cartesian translation.
    //! \param p_translation Translation vector (m).
    //! \param p_speed Speed factor.
    //! \return true if the movement has been applied.
    // ------------------------------------------------------------------------
    bool moveCartesian(const Eigen::Vector3d& p_translation,
                       double p_speed = 1.0);

    // ------------------------------------------------------------------------
    //! \brief Apply a rotation to the end effector.
    //! \param p_rotation_axis Rotation axis (normalized).
    //! \param p_angle Rotation angle (rad).
    //! \param p_speed Speed factor.
    //! \return true if the movement has been applied.
    // ------------------------------------------------------------------------
    bool rotateCartesian(const Eigen::Vector3d& p_rotation_axis,
                         double p_angle,
                         double p_speed = 1.0);

    // ------------------------------------------------------------------------
    //! \brief Record the current position of the robot as a waypoint.
    //! \param p_label Optional label for the waypoint.
    //! \param p_duration Duration for reaching this waypoint (seconds).
    //! \return Index of the recorded waypoint.
    // ------------------------------------------------------------------------
    size_t recordWaypoint(const std::string& p_label, double p_duration);

    // ------------------------------------------------------------------------
    //! \brief Delete a waypoint.
    //! \param p_idx Index of the waypoint to delete.
    // ------------------------------------------------------------------------
    void deleteWaypoint(size_t p_idx);

    // ------------------------------------------------------------------------
    //! \brief Clear all recorded waypoints.
    // ------------------------------------------------------------------------
    void clearWaypoints();

    // ------------------------------------------------------------------------
    //! \brief Move the robot to a recorded waypoint.
    //! \param p_idx Index of the target waypoint.
    //! \param p_duration Duration of the movement (s).
    //! \return true if the movement has started.
    // ------------------------------------------------------------------------
    bool goToWaypoint(size_t p_idx, double p_duration = 3.0);

    // ------------------------------------------------------------------------
    //! \brief Start the playback of the recorded waypoints.
    //! \return true if the playback has started.
    // ------------------------------------------------------------------------
    bool playRecordedTrajectory();

    // ------------------------------------------------------------------------
    //! \brief Stop the playback of the current trajectory.
    // ------------------------------------------------------------------------
    void stopTrajectory();

    // ------------------------------------------------------------------------
    //! \brief Update the state of the robot (advance in the trajectory).
    //! \param p_dt Time step (s).
    // ------------------------------------------------------------------------
    void update(double p_dt);

    // ------------------------------------------------------------------------
    //! \brief Get the error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    std::string const& error() const;

    // ------------------------------------------------------------------------
    //! \brief Check if a waypoint has been reached using pose error.
    //! \param p_robot The controlled robot.
    //! \param p_waypoint_index Index of the waypoint to check.
    //! \return true if the waypoint is reached (within tolerance).
    // ------------------------------------------------------------------------
    static bool isWaypointReached(application::ControlledRobot* p_robot,
                                  size_t p_waypoint_index);

private:

    // ------------------------------------------------------------------------
    //! \brief Check the preconditions for the operation.
    //! \return true if the preconditions are met.
    // ------------------------------------------------------------------------
    bool checkPreconditions(size_t p_joint_idx);

private:

    //! \brief Pointer to the controlled robot (can change via setRobot).
    Robot* m_robot = nullptr;
    //! \brief Pointer to the controlled robot with its state.
    application::ControlledRobot* m_controlled_robot = nullptr;
    //! \brief Pointer to the IK solver (shared, not owned).
    IKSolver* m_ik_solver = nullptr;
    //! \brief Pointer to the end effector.
    Node const* m_end_effector = nullptr;
    //! \brief Error message.
    std::string m_error;
};

} // namespace robotik
