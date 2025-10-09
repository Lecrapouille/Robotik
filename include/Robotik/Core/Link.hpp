/**
 * @file Link.hpp
 * @brief Link class - Representation of a robotic link.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Geometry.hpp"
#include "Robotik/Core/Inertial.hpp"
#include "Robotik/Core/SceneNode.hpp"

namespace robotik
{

// *********************************************************************************
//! \brief Class representing a robotic link.
//!
//! In robotics, a link is a rigid body that connects two joints.
// *********************************************************************************
class Link: public scene::Node
{
public:

    using Ptr = std::unique_ptr<Link>;

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_name The name of the link.
    //! \param p_visual The visual geometry of the link.
    // ------------------------------------------------------------------------
    explicit Link(std::string const& p_name, Geometry::Ptr p_visual)
        : scene::Node(p_name)
    {
        scene::Node::addChild(std::move(p_visual));
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_name The name of the link.
    //! \param p_visual The visual geometry of the link.
    //! \param p_collision The collision geometry of the link.
    // ------------------------------------------------------------------------
    explicit Link(std::string const& p_name,
                  Geometry::Ptr p_visual,
                  Geometry::Ptr p_collision)
        : scene::Node(p_name)
    {
        scene::Node::addChild(std::move(p_visual));
        scene::Node::addChild(std::move(p_collision));
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
    //! \brief Get the collision geometry (second geometry child found or same
    //! as visual if only one).
    //! \return The collision geometry or nullptr if not found.
    // ------------------------------------------------------------------------
    Geometry const& collision() const;
    Geometry& collision();

    // ------------------------------------------------------------------------
    //! \brief Get the link's debug string.
    //! \param p_detailed has no effect.
    //! \return The link's debug string
    // ------------------------------------------------------------------------
    // std::string debug(bool /*p_detailed*/) const override
    //{
    //    return "[" + name() + "]";
    //}

private:

    //! \brief Inertial properties of the link
    Inertial m_inertial;
};

} // namespace robotik