// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Actuators.hpp
/// Things that act on the robot or the world: motors, joint groups,
/// grippers.
///
/// A joint is kinematics (an axis and limits); an actuator is a device that
/// drives one or several joints, or the world (suction). Each actuator is a
/// resource of the same name, reserved by skills and disabled on failure.
#pragma once

#include "Robotik/Common/Math.hpp"
#include "Robotik/Robot/Joints.hpp"

#include "Compages/World/EntityId.hpp"

#include <span>
#include <string>
#include <vector>

namespace robotik
{

class Robot;

// ****************************************************************************
/// Base class of the actuators.
///
/// An actuator is added with @c robot.actuators().add<T>(name, ...); the set
/// calls @ref bind, and the robot calls @ref disable when its resource
/// fails.
///
/// @code
/// class Buzzer final : public robotik::Actuator
/// {
/// public:
///     explicit Buzzer(std::string p_name) : Actuator(std::move(p_name)) {}
///     void disable(robotik::Robot&) override { on = false; }
///     bool on = false;
/// };
/// @endcode
// ****************************************************************************
class Actuator
{
public:

    virtual ~Actuator() = default;

    /// @return Actuator name, also the name of its resource.
    [[nodiscard]] std::string const& name() const
    {
        return m_name;
    }

    // -------------------------------------------------------------------------
    /// Resolves joints and frames once the robot is loaded.
    /// @param p_robot Robot carrying the actuator.
    /// @throws std::invalid_argument on unknown names.
    // -------------------------------------------------------------------------
    virtual void bind(Robot& /*p_robot*/)
    {
        /* no-op */
    }

    // -------------------------------------------------------------------------
    /// The resource failed: switch to a safe state.
    /// @param p_robot Robot carrying the actuator.
    // -------------------------------------------------------------------------
    virtual void disable(Robot& /*p_robot*/)
    {
        /* no-op */
    }

protected:

    /// @param p_name Actuator name, unique in the world.
    explicit Actuator(std::string p_name) : m_name(std::move(p_name)) {}

private:

    /// Actuator and resource name.
    std::string m_name;
};

// ****************************************************************************
/// One rotary motor driving one revolute joint (a wheel, a pan axis).
///
/// @code
/// auto& left = robot.actuators().add<robotik::Motor>("left_motor",
///                                                    "left_wheel_joint");
/// left.spin(AngularVelocity(4.0));
/// @endcode
// ****************************************************************************
class Motor final: public Actuator
{
public:

    // -------------------------------------------------------------------------
    /// @param p_name Motor name, also its resource name.
    /// @param p_joint Name of the revolute joint it drives.
    // -------------------------------------------------------------------------
    Motor(std::string p_name, std::string p_joint);

    /// Finds the joint.
    void bind(Robot& p_robot) override;
    /// A failed motor delivers no torque.
    void disable(Robot& p_robot) override;

    /// Servos the joint to an angle.
    void moveTo(Radians p_position);
    /// Turns the joint at a speed.
    void spin(AngularVelocity p_velocity);
    /// Applies a raw torque.
    void push(Torque p_effort);
    /// Zero speed.
    void stop();

    /// @return The joint driven.
    [[nodiscard]] Revolute joint() const
    {
        return m_joint;
    }

    /// @return The measured angle.
    [[nodiscard]] Radians position() const;
    /// @return The measured speed.
    [[nodiscard]] AngularVelocity velocity() const;
    /// @return The torque of the last control step.
    [[nodiscard]] Torque effort() const;

private:

    /// Name of the joint driven.
    std::string m_joint_name;
    /// Joint resolved by @ref bind.
    Revolute m_joint;
    /// Joints of the robot, set by @ref bind.
    JointSet* m_joints = nullptr;
};

// ****************************************************************************
/// A set of joints commanded together (an arm, a head...).
///
/// Targets are given in the SI base unit of each joint (rad or m), in the
/// order of @ref joints, like an IK solution.
///
/// @code
/// auto& arm = robot.actuators().add<robotik::JointGroup>(
///     "arm", std::vector<std::string>{ "joint1", "joint2", "joint3" });
/// arm.moveTo(std::vector<double>{ 0.0, 0.4, -0.2 });
/// if (arm.reached(0.01)) { ... }
/// @endcode
// ****************************************************************************
class JointGroup final: public Actuator
{
public:

    // -------------------------------------------------------------------------
    /// @param p_name Group name, also its resource name.
    /// @param p_joints Joint names; empty takes every joint of the robot.
    // -------------------------------------------------------------------------
    JointGroup(std::string p_name, std::vector<std::string> p_joints = {});

    /// Finds the joints.
    void bind(Robot& p_robot) override;
    /// A failed group brakes where it is.
    void disable(Robot& p_robot) override;

    /// @return The joints of the group, in command order.
    [[nodiscard]] std::span<JointId const> joints() const
    {
        return m_joints;
    }

    /// @return True when @p_joint belongs to the group.
    [[nodiscard]] bool contains(JointId p_joint) const;

    // -------------------------------------------------------------------------
    /// Servos every joint of the group.
    /// @param p_positions One target per joint of the group, in @ref joints
    ///   order.
    // -------------------------------------------------------------------------
    void moveTo(std::span<double const> p_positions);
    /// Servos every joint of the group where it is.
    void hold();
    /// @return True when every joint is within @p_tolerance of its target.
    [[nodiscard]] bool reached(double p_tolerance) const;

private:

    /// Joint names given at construction.
    std::vector<std::string> m_names;
    /// Joints resolved by @ref bind.
    std::vector<JointId> m_joints;
    /// Joints of the robot, set by @ref bind.
    JointSet* m_set = nullptr;
};

// ****************************************************************************
/// Suction cup mounted along +Z of a link.
///
/// The command is the suction; what is held is measured (vacuum sensor on
/// hardware, contact model in simulation, see @ref GraspModel).
///
/// @code
/// auto* cup = robot.actuators().first<robotik::VacuumGripper>();
/// cup->suction(true);           // the grasp model attaches what touches it
/// if (cup->holding()) { ... }
/// cup->suction(false);          // release
/// @endcode
// ****************************************************************************
class VacuumGripper final: public Actuator
{
public:

    // -------------------------------------------------------------------------
    /// @param p_name Gripper name, also its resource name.
    /// @param p_link Flange link; empty uses the robot tool frame.
    /// @param p_length Flange to cup tip distance along +Z.
    // -------------------------------------------------------------------------
    VacuumGripper(std::string p_name,
                  std::string p_link = {},
                  Length p_length = Length(0.06));

    /// Checks the flange link.
    void bind(Robot& p_robot) override;
    /// A failed pump drops what it holds.
    void disable(Robot& p_robot) override;

    /// Turns the pump on (true) or off (false).
    void suction(bool p_on)
    {
        m_suction = p_on;
    }

    /// @return True when the pump is commanded on.
    [[nodiscard]] bool suction() const
    {
        return m_suction;
    }

    /// @return True when an object is held (measured).
    [[nodiscard]] bool holding() const
    {
        return m_held.valid();
    }

    /// @return The entity held, or an invalid id.
    [[nodiscard]] compages::world::EntityId held() const
    {
        return m_held;
    }

    // -------------------------------------------------------------------------
    /// Writes the measured grasp (physics or vacuum sensor side).
    /// @param p_object Entity held, or an invalid id for nothing.
    // -------------------------------------------------------------------------
    void held(compages::world::EntityId p_object)
    {
        m_held = p_object;
    }

    /// @return The flange link; empty for the robot tool frame.
    [[nodiscard]] std::string const& link() const
    {
        return m_link;
    }

    /// @return The flange to cup tip distance.
    [[nodiscard]] Length length() const
    {
        return m_length;
    }

    // -------------------------------------------------------------------------
    /// @param p_robot Robot carrying the gripper.
    /// @return Flange pose in the robot base frame.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose flange(Robot const& p_robot) const;

    // -------------------------------------------------------------------------
    /// @param p_robot Robot carrying the gripper.
    /// @param p_extra Distance beyond the cup, along +Z of the flange.
    /// @return Cup tip in the robot base frame, @p_extra beyond the cup.
    // -------------------------------------------------------------------------
    [[nodiscard]] Vector3 tip(Robot const& p_robot,
                              Length p_extra = Length(0.0)) const;

private:

    /// Flange link; empty for the tool frame.
    std::string m_link;
    /// Flange to cup tip distance.
    Length m_length;
    /// Pump commanded on.
    bool m_suction = false;
    /// Entity held (measured).
    compages::world::EntityId m_held{};
};

} // namespace robotik
