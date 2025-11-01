/**
 * @file Component.hpp
 * @brief Base classes for all robot components (sensors, actuators, etc.).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Blueprint/Node.hpp"

namespace robotik
{

// ****************************************************************************
//! \brief Base class for all robot components.
// ****************************************************************************
class RobotComponent: public Node
{
public:

    // ------------------------------------------------------------------------
    //! \brief Update the component with the given time step.
    //! \param p_dt The time step.
    // ------------------------------------------------------------------------
    virtual void update(double p_dt) = 0;
};

// ****************************************************************************
//! \brief Base class for all sensors.
// ****************************************************************************
class Sensor: public RobotComponent
{
public:

    // ------------------------------------------------------------------------
    //! \brief Read the sensor data.
    //! \note the value shall be stored as variable member of the node.
    // ------------------------------------------------------------------------
    virtual void readSensor() = 0;

    // ------------------------------------------------------------------------
    //! \brief Check if the sensor data is ready.
    //! \return True if the sensor data is ready, false otherwise.
    // ------------------------------------------------------------------------
    virtual bool isDataReady() const = 0;

    // ------------------------------------------------------------------------
    //! \brief Accept a visitor (Visitor pattern override).
    //! \param visitor The visitor to accept.
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    //! \brief Accept a const visitor (Visitor pattern override).
    //! \param visitor The const visitor to accept.
    // ------------------------------------------------------------------------
};

// ****************************************************************************
//! \brief Base class for all actuators.
// ****************************************************************************
class Actuator: public RobotComponent
{
public:

    // ------------------------------------------------------------------------
    //! \brief Set the command for the actuator.
    //! \param p_command The command.
    // ------------------------------------------------------------------------
    virtual void setCommand(double p_command) = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the current value of the actuator.
    // ------------------------------------------------------------------------
    virtual double getCurrentValue() const = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the effort of the actuator.
    //! \return The effort.
    //! \note the effort is the torque/force applied by the actuator.
    // ------------------------------------------------------------------------
    virtual double getEffort() const = 0; // couple/force

    // ------------------------------------------------------------------------
    //! \brief Accept a visitor (Visitor pattern override).
    //! \param visitor The visitor to accept.
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    //! \brief Accept a const visitor (Visitor pattern override).
    //! \param visitor The const visitor to accept.
    // ------------------------------------------------------------------------
};

#if 0
namespace example
{

// ****************************************************************************
//! \brief Servo motor actuator.
// ****************************************************************************
class ServoMotor: public Actuator
{
public:

    void setCommand(double p_target) override
    {
        m_target_position = p_target;
    }

    void update(double p_dt) override
    {
        // Simulation du servo avec PID
        double error = m_target_position - m_current_position;
        double torque = m_pid.compute(error, p_dt);
        // Mise à jour position basée sur le couple
    }

private:

    double m_target_position;
    double m_current_position;
    double m_max_torque;
    PIDController m_pid;
};

class EncoderSensor: public Sensor
{
public:

    void readSensor() override {}

    double getPosition() const
    {
        return m_position;
    }

    double getVelocity() const
    {
        return m_velocity;
    }

private:

    double m_position;
    double m_velocity;
    double m_resolution;
};

class ForceSensor: public Sensor
{
    Eigen::Vector3d force;
    Eigen::Vector3d torque;

public:

    void readSensor() override
    {
        // Read
    }

    Eigen::Vector6d getWrench() const
    {
        Eigen::Vector6d wrench;
        wrench << force, torque;
        return wrench;
    }
};

} // namespace example
#endif

} // namespace robotik