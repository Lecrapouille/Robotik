/**
 * @file ControlledRobot.hpp
 * @brief Controlled robot with interactive control state.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Robot.hpp"
#include "Robotik/Core/Solvers/Trajectory.hpp"

#include <memory>

namespace robotik::application
{

// ****************************************************************************
//! \brief Controlled robot class that extends Robot with control capabilities.
//!
//! This class inherits from Robot and adds state for interactive control
//! (teach pendant), waypoints, and camera tracking support.
// ****************************************************************************
class ControlledRobot: public robotik::Robot
{
public:

    // ------------------------------------------------------------------------
    //! \brief Control mode for the robot.
    // ------------------------------------------------------------------------
    enum class ControlMode
    {
        JOINT,      //!< Joint control (joint space)
        CARTESIAN,  //!< Cartesian control (task space)
        TRAJECTORY, //!< Trajectory control
    };

    // ------------------------------------------------------------------------
    //! \brief State of the robot control.
    // ------------------------------------------------------------------------
    enum class State
    {
        IDLE,           //!< Idle
        MANUAL_CONTROL, //!< Manual control
        PLAYING         //!< Playing trajectory
    };

    // ------------------------------------------------------------------------
    //! \brief Constructor forwarding to Robot constructor.
    //! \param p_name The name of the robot.
    //! \param p_blueprint The robot's kinematic structure.
    // ------------------------------------------------------------------------
    ControlledRobot(std::string const& p_name, robotik::Blueprint&& p_blueprint)
        : Robot(p_name, std::move(p_blueprint))
    {
    }

public:

    //! \brief Waypoint recorded by the user (editable in the HMI)
    struct Waypoint
    {
        robotik::Trajectory::States states;
        std::string label;
        double duration;
    };

    //! \brief Current control mode
    ControlMode control_mode = ControlMode::JOINT;
    //! \brief Current state of the robot
    State state = State::IDLE;
    //! \brief Speed factor [0.0, 1.0]
    double speed_factor = 0.5;
    //! \brief Current recorded trajectory
    std::unique_ptr<robotik::Trajectory> trajectory;
    //! \brief Waypoints recorded by the user (HMI)
    std::vector<ControlledRobot::Waypoint> waypoints;
    //! \brief Current waypoint (-1 indicates coming from current robot
    //! position, not a recorded waypoint)
    int current_waypoint_index = -1;
    //! \brief Destination waypoint
    size_t target_waypoint_index = 0;
    //! \brief Play in loop the trajectory
    bool play_in_loop = false;
    //! \brief Whether we're playing all waypoints (play) or going to a single
    //! waypoint (go)
    bool is_playing_all_waypoints = false;
    //! \brief Time elapsed in the trajectory
    double trajectory_current_time = 0.0;
    //! \brief End effector for Cartesian control
    robotik::Node const* end_effector = nullptr;
    //! \brief Target node for the camera
    robotik::Node const* camera_target = nullptr;
    //! \brief Camera tracking enabled
    bool camera_tracking_enabled = true;
    //! \brief Frame for Cartesian control (nullptr = world frame)
    robotik::Node const* cartesian_frame = nullptr;
};

} // namespace robotik::application
