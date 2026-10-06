// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Robot/Actuators.hpp"

#include "Robotik/Robot/Robot.hpp"

#include <algorithm>
#include <stdexcept>

namespace robotik
{

Motor::Motor(std::string p_name, std::string p_joint)
    : Actuator(std::move(p_name)), m_joint_name(std::move(p_joint))
{
}

void Motor::bind(Robot& p_robot)
{
    m_joints = &p_robot.joints();
    m_joint = m_joints->revolute(m_joint_name);
}

void Motor::disable(Robot& /*p_robot*/)
{
    m_joints->release(m_joint.id);
}

void Motor::moveTo(Radians p_position)
{
    m_joints->moveTo(m_joint, p_position);
}

void Motor::spin(AngularVelocity p_velocity)
{
    m_joints->spin(m_joint, p_velocity);
}

void Motor::push(Torque p_effort)
{
    m_joints->push(m_joint, p_effort);
}

void Motor::stop()
{
    m_joints->spin(m_joint, AngularVelocity(0.0));
}

Radians Motor::position() const
{
    return m_joints->position(m_joint);
}

AngularVelocity Motor::velocity() const
{
    return m_joints->velocity(m_joint);
}

Torque Motor::effort() const
{
    return m_joints->effort(m_joint);
}

JointGroup::JointGroup(std::string p_name, std::vector<std::string> p_joints)
    : Actuator(std::move(p_name)), m_names(std::move(p_joints))
{
}

void JointGroup::bind(Robot& p_robot)
{
    m_set = &p_robot.joints();
    m_joints.clear();
    if (m_names.empty())
    {
        for (JointId id = 0; id < m_set->size(); ++id)
        {
            m_joints.push_back(id);
        }
        return;
    }
    for (std::string const& name : m_names)
    {
        m_joints.push_back(m_set->require(name));
    }
}

void JointGroup::disable(Robot& /*p_robot*/)
{
    for (JointId const id : m_joints)
    {
        if (m_set->mode(id) != JointMode::Position)
        {
            m_set->hold(id);
        }
    }
}

bool JointGroup::contains(JointId p_joint) const
{
    return std::find(m_joints.begin(), m_joints.end(), p_joint) !=
           m_joints.end();
}

void JointGroup::moveTo(std::span<double const> p_positions)
{
    if (p_positions.size() != m_joints.size())
    {
        throw std::invalid_argument("Joint group '" + name() +
                                    "': wrong number of targets");
    }
    for (std::size_t i = 0; i < m_joints.size(); ++i)
    {
        m_set->moveTo(m_joints[i], p_positions[i]);
    }
}

void JointGroup::hold()
{
    for (JointId const id : m_joints)
    {
        m_set->hold(id);
    }
}

bool JointGroup::reached(double p_tolerance) const
{
    return std::all_of(m_joints.begin(),
                       m_joints.end(),
                       [this, p_tolerance](JointId p_id)
                       { return m_set->reached(p_id, p_tolerance); });
}

VacuumGripper::VacuumGripper(std::string p_name,
                             std::string p_link,
                             Length p_length)
    : Actuator(std::move(p_name)), m_link(std::move(p_link)), m_length(p_length)
{
}

void VacuumGripper::bind(Robot& p_robot)
{
    if (m_link.empty())
    {
        m_link = p_robot.tool();
    }
    if (!p_robot.link(m_link))
    {
        throw std::invalid_argument("Vacuum gripper '" + name() +
                                    "': unknown link '" + m_link + "'");
    }
}

void VacuumGripper::disable(Robot& /*p_robot*/)
{
    m_suction = false;
}

Pose VacuumGripper::flange(Robot const& p_robot) const
{
    return p_robot.framePose(m_link);
}

Vector3 VacuumGripper::tip(Robot const& p_robot, Length p_extra) const
{
    return flange(p_robot) *
           Vector3(0.0, 0.0, (m_length + p_extra).value());
}

} // namespace robotik
