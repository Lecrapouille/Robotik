/**
 * @file Action.hpp
 * @brief Action leaf nodes: Action, SugarAction.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Leaf.hpp"

#include <functional>

namespace robotik::bt {

// ****************************************************************************
//! \brief Action node that can be used to execute custom behavior. You shall
//! override the onRunning() method to implement your behavior.
// ****************************************************************************
class Action: public Leaf
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Action".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Action";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the action. This method must be overridden by derived
    //! classes.
    //! \return The status of the action.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override = 0;

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitAction(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitAction(*this);
    }
};

// ****************************************************************************
//! \brief Action node that can be used to execute custom behavior. This class
//! should not be used directly: it is used internally to sugar the class Action
//! by hiding inheritance.
// ****************************************************************************
class SugarAction final: public Leaf
{
public:

    // ------------------------------------------------------------------------
    //! \brief Type alias for the action function.
    // ------------------------------------------------------------------------
    using Function = std::function<Status()>;

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a function to execute.
    //! \param[in] func The function to execute when the action runs.
    // ------------------------------------------------------------------------
    explicit SugarAction(Function func) : m_func(std::move(func))
    {
        m_type = "SugarAction";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a function and blackboard.
    //! \param[in] func The function to execute when the action runs.
    //! \param[in] blackboard The blackboard to use.
    // ------------------------------------------------------------------------
    SugarAction(Function func, Blackboard::Ptr blackboard)
        : m_func(std::move(func))
    {
        m_type = "SugarAction";
        setBlackboard(blackboard);
    }

    // ------------------------------------------------------------------------
    //! \brief Execute the action.
    //! \return The status of the action.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        return m_func();
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the leaf node is valid (not nullptr function).
    //! \return True if the leaf node is valid, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool isValid() const override
    {
        return m_func != nullptr;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitSugarAction(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitSugarAction(*this);
    }

private:

    //! \brief The function to execute when the action runs.
    Function m_func = nullptr;
};

} // namespace robotik::bt
