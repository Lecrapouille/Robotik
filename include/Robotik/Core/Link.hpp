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
#include "Robotik/Core/SceneNode.hpp"

namespace robotik
{

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
    //! \brief Get the visual geometry (first geometry child found).
    //! \return The visual geometry or nullptr if not found.
    // ------------------------------------------------------------------------
    Geometry const& geometry() const;

    // ------------------------------------------------------------------------
    //! \brief Get the collision geometry (second geometry child found or same
    //! as visual if only one).
    //! \return The collision geometry or nullptr if not found.
    // ------------------------------------------------------------------------
    Geometry const& collision() const;

    // ------------------------------------------------------------------------
    //! \brief Get the link's debug string.
    //! \param p_detailed has no effect.
    //! \return The link's debug string
    // ------------------------------------------------------------------------
    // std::string debug(bool /*p_detailed*/) const override
    //{
    //    return "[" + name() + "]";
    //}
};

} // namespace robotik