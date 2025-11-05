/**
 * @file Frame.hpp
 * @brief Frame class - Representation of a coordinate frame.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Blueprint/Node.hpp"

namespace robotik
{

// *********************************************************************************
//! \brief Class representing a coordinate frame (reference frame).
//!
//! A Frame (frame) is a coordinate system that defines position and
//! orientation in 3D space. It is used for Cartesian motion planning and to
//! attach robots to other coordinate frames (e.g., workshop frame).
//!
//! Unlike Link, Frame has no physical geometry but serves as a pure coordinate
//! transformation node in the scene graph.
// *********************************************************************************
class Frame: public Node
{
public:

    using Ptr = std::unique_ptr<Frame>;

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_name The name of the frame.
    // ------------------------------------------------------------------------
    explicit Frame(std::string const& p_name) : Node(p_name) {}

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
};

} // namespace robotik
