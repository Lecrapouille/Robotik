// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Robot/Robot.hpp"

#include <atomic>
#include <stdexcept>

namespace robotik
{

/// A serial number never given before in this process (0 is "no robot").
static std::uint64_t nextSerial()
{
    static std::atomic<std::uint64_t> s_last{ 0 };
    return ++s_last;
}

Robot::Robot(std::filesystem::path const& p_urdf,
             ToolMount const& p_tool,
             std::shared_ptr<ResourceManager> p_resources)
    : m_kinematics(p_urdf, p_tool),
      m_resources(p_resources ? std::move(p_resources)
                              : std::make_shared<ResourceManager>()),
      m_sensors(*this, *m_resources),
      m_actuators(*this, *m_resources),
      m_serial(nextSerial())
{
}

Robot::~Robot() = default;

Pose Robot::requireFramePose(std::string const& p_frame) const
{
    return m_kinematics.requireFramePose(p_frame);
}

Pose Robot::requireWorldPose(std::string const& p_frame) const
{
    return p_frame.empty() ? m_base.pose
                           : m_base.pose * requireFramePose(p_frame);
}

LinkRef Robot::requireLink(std::string const& p_name) const
{
    if (p_name.empty())
    {
        return LinkRef{ m_serial, LinkRef::BASE };
    }
    if (!m_kinematics.hasFrame(p_name))
    {
        throw std::invalid_argument("Unknown Pinocchio frame: " + p_name);
    }
    return LinkRef{ m_serial, m_kinematics.frameId(p_name) };
}

Pose Robot::getWorldPose(LinkRef const& p_link) const
{
    return poseFrom(m_base, p_link);
}

Pose Robot::poseFrom(BaseState const& p_base, LinkRef const& p_link) const
{
    if (p_link.m_robot != m_serial)
    {
        throw std::logic_error(
            "Robot '" + m_id +
            "': this LinkRef was not found on this robot (another robot, or "
            "never found): call requireLink on this robot");
    }
    return p_link.m_frame == LinkRef::BASE
               ? p_base.pose
               : p_base.pose * m_kinematics.framePose(p_link.m_frame);
}

std::optional<std::vector<double>>
Robot::inverseKinematics(std::string const& p_frame,
                         Pose const& p_target,
                         IkGoal p_goal) const
{
    auto solution = m_kinematics.solveIK(
        p_frame, p_target, m_kinematics.configuration(), p_goal);
    if (!solution)
    {
        return std::nullopt;
    }
    std::vector<double> targets(m_joints.size());
    for (JointId id = 0; id < m_joints.size(); ++id)
    {
        int const index = m_model.q_indices[id];
        targets[id] = index >= 0 ? (*solution)[static_cast<std::size_t>(index)]
                                 : m_joints.position(id);
    }
    return targets;
}

void Robot::updateKinematics()
{
    std::span<double> q = m_kinematics.configuration();
    std::span<double> v = m_kinematics.velocity();
    for (JointId id = 0; id < m_joints.size(); ++id)
    {
        if (int const index = m_model.q_indices[id]; index >= 0)
        {
            q[static_cast<std::size_t>(index)] = m_joints.position(id);
        }
        if (int const index = m_model.v_indices[id]; index >= 0)
        {
            v[static_cast<std::size_t>(index)] = m_joints.velocity(id);
        }
    }
    m_kinematics.updateKinematics();
}

} // namespace robotik
