/**
 * @file Component.hpp
 * @brief Base classes for all robot components (sensors, actuators, etc.).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/SceneNode.hpp"

namespace robotik
{

// ****************************************************************************
//! \brief Base class for all robot components.
// ****************************************************************************
class RobotComponent: public scene::Node
{
public:

    virtual void update(double p_dt) = 0;
};

// ****************************************************************************
//! \brief Base class for all sensors.
// ****************************************************************************
class Sensor: public RobotComponent
{
public:

    virtual void readSensor() = 0;
    virtual bool isDataReady() const = 0;
};

// ****************************************************************************
//! \brief Base class for all actuators.
// ****************************************************************************
class Actuator: public RobotComponent
{
public:

    virtual void setCommand(double p_command) = 0;
    virtual double getCurrentValue() const = 0;
    virtual double getEffort() const = 0; // couple/force
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