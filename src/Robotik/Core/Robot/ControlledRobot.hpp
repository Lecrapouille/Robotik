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
        JOINT,      //!< Contrôle articulaire (joint space)
        CARTESIAN,  //!< Contrôle cartésien (task space)
        TRAJECTORY  //!< Lecture de trajectoire enregistrée
    };

    // ------------------------------------------------------------------------
    //! \brief State of the robot control.
    // ------------------------------------------------------------------------
    enum class State
    {
        IDLE,           //!< Inactif
        MANUAL_CONTROL, //!< Contrôle manuel en cours
        PLAYING         //!< Lecture de trajectoire
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

    // === État du contrôle ===
    //! \brief Mode de contrôle actuel
    ControlMode control_mode = ControlMode::JOINT;
    //! \brief État actuel du robot
    State state = State::IDLE;
    //! \brief Facteur de vitesse [0.0, 1.0]
    double speed_factor = 0.5;

    // === Waypoints et trajectoires ===
    //! \brief Waypoints enregistrés (positions des joints)
    std::vector<robotik::Trajectory::States> waypoints;
    //! \brief Trajectoire en cours de lecture
    std::unique_ptr<robotik::Trajectory> playing_trajectory;
    //! \brief Temps écoulé dans la trajectoire
    double trajectory_time = 0.0;

    // === Configuration du contrôle ===
    //! \brief End effector pour contrôle cartésien
    robotik::Node const* end_effector = nullptr;

    // === Suivi caméra ===
    //! \brief Nœud cible pour la caméra
    robotik::Node const* camera_target = nullptr;
    //! \brief Suivi caméra activé
    bool camera_tracking_enabled = true;
};

} // namespace robotik::application

