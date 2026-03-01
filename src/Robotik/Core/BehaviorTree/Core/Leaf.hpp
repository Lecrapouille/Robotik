/**
 * @file Leaf.hpp
 * @brief Base class for leaf nodes that have no children.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Node.hpp"

namespace robotik::bt {

// ****************************************************************************
//! \brief Base class for leaf nodes that have no children.
//! Leaf nodes are the nodes that actually do the work.
//! The blackboard is inherited from Node base class.
// ****************************************************************************
class Leaf: public Node
{
public:

    // ------------------------------------------------------------------------
    //! \brief Default constructor.
    // ------------------------------------------------------------------------
    Leaf() = default;

    // ------------------------------------------------------------------------
    //! \brief Check if the leaf node is valid.
    //! \return True if the leaf node is valid, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool isValid() const override
    {
        return true;
    }
};

} // namespace robotik::bt
