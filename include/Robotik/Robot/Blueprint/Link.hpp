/**
 * @file Link.hpp
 * @brief Link class - Representation of a robotic link.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Common/Types.hpp"
#include "Robotik/Robot/Blueprint/Geometry.hpp"

namespace robotik
{

// *********************************************************************************
//! \brief Class representing a robotic link.
//!
//! In robotics, a link is a rigid body that connects two joints.
// *********************************************************************************
class Link: public Node
{
public:

    using Ptr = std::unique_ptr<Link>;

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_name The name of the link.
    //! \param p_visual The visual geometry of the link. Shall not be null.
    // ------------------------------------------------------------------------
    explicit Link(std::string const& p_name, Geometry::Ptr p_visual)
        : Node(p_name)
    {
        Node::addChild(std::move(p_visual));
    }

    // ------------------------------------------------------------------------
    //! \brief Set the collision data for the link.
    //! \param p_collision_center Center of collision volume in local frame.
    //! \param p_collision_orientation Orientation of collision volume in local
    //! frame.
    //! \param p_collision_params Collision parameters (dimensions).
    // ------------------------------------------------------------------------
    void setCollisionData(const Eigen::Vector3d& p_collision_center,
                          const Eigen::Matrix3d& p_collision_orientation,
                          const std::vector<double>& p_collision_params)
    {
        geometry().setCollisionData(
            p_collision_center, p_collision_orientation, p_collision_params);
    }

    // ------------------------------------------------------------------------
    //! \brief Set the inertial properties of the link.
    //! \param p_mass The mass of the link.
    //! \param p_center_of_mass The center of mass of the link.
    //! \param p_inertia_matrix The inertia matrix of the link.
    // ------------------------------------------------------------------------
    void inertia(double p_mass,
                 const Eigen::Vector3d& p_center_of_mass,
                 const Eigen::Matrix3d& p_inertia_matrix);

    // ------------------------------------------------------------------------
    //! \brief Set the inertial properties of the link.
    //! \param p_inertial The inertial properties to set.
    // ------------------------------------------------------------------------
    inline void inertia(const Inertial& p_inertial)
    {
        m_inertial = p_inertial;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the inertial properties of the link.
    //! \return The inertial properties of the link.
    // ------------------------------------------------------------------------
    inline Inertial const& inertia() const
    {
        return m_inertial;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the mass of the link.
    //! \return The mass of the link.
    // ------------------------------------------------------------------------
    double mass() const;

    // ------------------------------------------------------------------------
    //! \brief Get the center of mass of the link.
    //! \return The center of mass of the link.
    // ------------------------------------------------------------------------
    Eigen::Vector4d centerOfMass() const;

    // ------------------------------------------------------------------------
    //! \brief Get the length of the link.
    //! \return The length of the link.
    // ------------------------------------------------------------------------
    double length() const;

    // ------------------------------------------------------------------------
    //! \brief Get the visual geometry (first geometry child found).
    //! \return The visual geometry or nullptr if not found.
    // ------------------------------------------------------------------------
    Geometry const& geometry() const;
    Geometry& geometry();

    // ------------------------------------------------------------------------
    //! \brief Accept a visitor (Visitor pattern override).
    //! \param visitor The visitor to accept.
    // ------------------------------------------------------------------------
    void accept(NodeVisitor& visitor) override;

    // ------------------------------------------------------------------------
    //! \brief Accept a const visitor (Visitor pattern override).
    //! \param visitor The const visitor to accept.
    // ------------------------------------------------------------------------
    void accept(ConstNodeVisitor& visitor) const override;

private:

    //! \brief Inertial properties of the link
    Inertial m_inertial;
};

} // namespace robotik