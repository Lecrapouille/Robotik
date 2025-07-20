#include "Robotik/private/Joint.hpp"

#include <iostream>

namespace robotik
{

// ----------------------------------------------------------------------------
Joint::Joint(std::string_view const& p_name,
             Joint::Type p_type,
             const Eigen::Vector3d& p_axis)
    : Node(p_name), m_type(p_type), m_axis(p_axis.normalized())
{
    switch (p_type)
    {
        case Joint::Type::CONTINUOUS:
        case Joint::Type::REVOLUTE:
            m_min = -M_PI;
            m_max = M_PI;
            m_position = 0.0;
            break;
        case Joint::Type::PRISMATIC:
            m_min = -1.0;
            m_max = 1.0;
            m_position = 0.0;
            break;
        case Joint::Type::FIXED:
            m_min = 0.0;
            m_max = 0.0;
            m_position = 0.0;
            break;
    }

    jointTransform(computeJointTransform(m_type, m_position, m_axis));
}

// ----------------------------------------------------------------------------
void Joint::position(double p_position)
{
    if (m_type == Joint::Type::FIXED)
        return;

    if (m_type != Joint::Type::CONTINUOUS)
    {
        if (p_position < m_min)
            p_position = m_min;
        if (p_position > m_max)
            p_position = m_max;
    }

    m_position = p_position;
    jointTransform(computeJointTransform(m_type, m_position, m_axis));
}

// ----------------------------------------------------------------------------
Transform Joint::computeJointTransform(Joint::Type p_type,
                                       double p_position,
                                       const Eigen::Vector3d& p_axis) const
{
    Eigen::Matrix4d joint_transform = Eigen::Matrix4d::Identity();

    switch (p_type)
    {
        case Joint::Type::CONTINUOUS:
        case Joint::Type::REVOLUTE:
        {
            // Create the rotation matrix around the axis
            Eigen::AngleAxisd joint_rotation(p_position, p_axis);
            joint_transform.block<3, 3>(0, 0) =
                joint_rotation.toRotationMatrix();
            break;
        }
        case Joint::Type::PRISMATIC:
        {
            // Translation along the axis
            joint_transform.block<3, 1>(0, 3) = p_axis * p_position;
            break;
        }
        case Joint::Type::FIXED:
            // No transformation for fixed joints
            break;
    }

    return joint_transform;
}

} // namespace robotik