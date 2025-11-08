/**
 * @file ControlledRobot.hpp
 * @brief Controlled robot with UI state for the simulator application.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Robot.hpp"

namespace robotik::application
{

// ****************************************************************************
//! \brief Controlled robot class that extends Robot with UI state.
//!
//! This class inherits from Robot and adds minimal state for the UI layer.
//! All complex logic (waypoints, trajectory control) is handled by separate
//! managers in the application layer.
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

    //! \brief Current control mode
    ControlMode control_mode = ControlMode::JOINT;
    //! \brief Current state of the robot
    State state = State::IDLE;
    //! \brief Speed factor [0.0, 1.0]
    double speed_factor = 0.5;
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

