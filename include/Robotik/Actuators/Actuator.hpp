// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Actuator.hpp
//! @brief Things that act on the robot or the world: motors, joint groups,
//! grippers.
//!
//! A joint is kinematics (an axis and limits); an actuator is a device that
//! drives one or several joints, or the world (suction). Each actuator is a
//! resource of the same name, reserved by skills and disabled on failure.
#pragma once

#include "Robotik/Math/Pose.hpp"
#include "Robotik/Robot/Joints.hpp"

#include "Compages/World/EntityId.hpp"

#include <span>
#include <string>
#include <vector>

namespace robotik
{

class Robot;

// ****************************************************************************
//! @brief Base class of the actuators.
// ****************************************************************************
class Actuator
{
public:

    virtual ~Actuator() = default;

    [[nodiscard]] std::string const& name() const
    {
        return m_name;
    }

    // -------------------------------------------------------------------------
    //! @brief Resolves joints and frames once the robot is loaded.
    //! @throws std::invalid_argument on unknown names.
    // -------------------------------------------------------------------------
    virtual void bind(Robot& /*p_robot*/)
    {
        /* no-op */
    }

    // -------------------------------------------------------------------------
    //! @brief The resource failed: switch to a safe state.
    // -------------------------------------------------------------------------
    virtual void disable(Robot& /*p_robot*/)
    {
        /* no-op */
    }

protected:

    explicit Actuator(std::string p_name) : m_name(std::move(p_name)) {}

private:

    std::string m_name;
};

// ****************************************************************************
//! @brief One motor driving one joint (e.g. a wheel).
//!
//! @code
//! auto& left = robot.actuators().add<robotik::Motor>("left_motor",
//!                                                    "left_wheel_joint");
//! left.spin(4.0); // rad/s
//! @endcode
// ****************************************************************************
class Motor final: public Actuator
{
public:

    Motor(std::string p_name, std::string p_joint);

    void bind(Robot& p_robot) override;
    //! @brief A failed motor delivers no effort.
    void disable(Robot& p_robot) override;

    void moveTo(double p_position);
    void spin(double p_velocity);
    void push(double p_effort);
    void stop();

    [[nodiscard]] JointId joint() const
    {
        return m_joint;
    }

    [[nodiscard]] double position() const;
    [[nodiscard]] double velocity() const;

private:

    std::string m_joint_name;
    JointId m_joint = NO_JOINT;
    JointSet* m_joints = nullptr;
};

// ****************************************************************************
//! @brief A set of joints commanded together (an arm, a head...).
// ****************************************************************************
class JointGroup final: public Actuator
{
public:

    //! @param p_joints Joint names; empty takes every joint of the robot.
    JointGroup(std::string p_name, std::vector<std::string> p_joints = {});

    void bind(Robot& p_robot) override;
    //! @brief A failed group brakes where it is.
    void disable(Robot& p_robot) override;

    [[nodiscard]] std::span<JointId const> joints() const
    {
        return m_joints;
    }

    [[nodiscard]] bool contains(JointId p_joint) const;

    //! @brief One target per joint of the group, in @ref joints order.
    void moveTo(std::span<double const> p_positions);
    void hold();
    [[nodiscard]] bool reached(double p_tolerance) const;

private:

    std::vector<std::string> m_names;
    std::vector<JointId> m_joints;
    JointSet* m_set = nullptr;
};

// ****************************************************************************
//! @brief Suction cup mounted along +Z of a link.
//!
//! The command is the suction; what is held is measured (vacuum sensor on
//! hardware, contact model in simulation).
// ****************************************************************************
class VacuumGripper final: public Actuator
{
public:

    //! @param p_link Flange link; empty uses the robot tool frame.
    //! @param p_length Flange to cup tip distance along +Z (SI: m).
    VacuumGripper(std::string p_name,
                  std::string p_link = {},
                  Length p_length = Length(0.06));

    void bind(Robot& p_robot) override;
    //! @brief A failed pump drops what it holds.
    void disable(Robot& p_robot) override;

    void suction(bool p_on)
    {
        m_suction = p_on;
    }

    [[nodiscard]] bool suction() const
    {
        return m_suction;
    }

    [[nodiscard]] bool holding() const
    {
        return m_held.valid();
    }

    [[nodiscard]] compages::world::EntityId held() const
    {
        return m_held;
    }

    //! @brief Measured grasp (physics or vacuum sensor side).
    void held(compages::world::EntityId p_object)
    {
        m_held = p_object;
    }

    [[nodiscard]] std::string const& link() const
    {
        return m_link;
    }

    [[nodiscard]] Length length() const
    {
        return m_length;
    }

    //! @brief Flange pose in the robot base frame.
    [[nodiscard]] Pose flange(Robot const& p_robot) const;
    //! @brief Cup tip in the robot base frame, @p_extra beyond the cup.
    [[nodiscard]] Vector3 tip(Robot const& p_robot, double p_extra = 0.0) const;

private:

    std::string m_link;
    Length m_length;
    bool m_suction = false;
    compages::world::EntityId m_held{};
};

} // namespace robotik
