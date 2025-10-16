#include "Robotik/Core/Link.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
Geometry const& Link::geometry() const
{
    // Return the first child that shall exist.
    auto const& children = hierarchy::Node::children();
    return *dynamic_cast<Geometry const*>(children[0].get());
}

// ----------------------------------------------------------------------------
Geometry& Link::geometry()
{
    auto& children = hierarchy::Node::children();
    return *dynamic_cast<Geometry*>(children[0].get());
}

// ----------------------------------------------------------------------------
void Link::inertia(double p_mass,
                   const Eigen::Vector3d& p_center_of_mass,
                   const Eigen::Matrix3d& p_inertia_matrix)
{
    m_inertial.mass = p_mass;
    m_inertial.center_of_mass = p_center_of_mass;
    m_inertial.inertia_matrix = p_inertia_matrix;
}

// ----------------------------------------------------------------------------
double Link::mass() const
{
    return m_inertial.mass;
}

// ----------------------------------------------------------------------------
Eigen::Vector4d Link::centerOfMass() const
{
    return Eigen::Vector4d(m_inertial.center_of_mass.x(),
                           m_inertial.center_of_mass.y(),
                           m_inertial.center_of_mass.z(),
                           1.0);
}

// ----------------------------------------------------------------------------
double Link::length() const
{
    // Approximate length as the distance from origin to center of mass
    return m_inertial.center_of_mass.norm();
}

} // namespace robotik