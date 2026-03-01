/**
 * @file Condition.hpp
 * @brief Condition leaf node.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Leaf.hpp"

#include <functional>

namespace robotik::bt {

// ****************************************************************************
//! \brief Condition node that can be used to evaluate a condition. This class
//! should not be used directly: it is used internally to sugar the class Action
//! by hiding inheritance.
// ****************************************************************************
class Condition final: public Leaf
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Condition".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Condition";
    }

    // ------------------------------------------------------------------------
    //! \brief Type alias for the condition function.
    // ------------------------------------------------------------------------
    using Function = std::function<bool()>;

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a function to evaluate.
    //! \param[in] p_func The function to evaluate when the condition runs.
    // ------------------------------------------------------------------------
    explicit Condition(Function p_func) : m_func(std::move(p_func))
    {
        m_type = toString();
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a function and blackboard.
    //! \param[in] p_func The function to evaluate when the condition runs.
    //! \param[in] p_blackboard The blackboard to use.
    // ------------------------------------------------------------------------
    Condition(Function p_func, Blackboard::Ptr p_blackboard)
        : m_func(std::move(p_func))
    {
        m_type = toString();
        setBlackboard(p_blackboard);
    }

    // ------------------------------------------------------------------------
    //! \brief Execute the condition.
    //! \return SUCCESS if the condition is true, FAILURE otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        return m_func() ? Status::SUCCESS : Status::FAILURE;
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
        p_visitor.visitCondition(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitCondition(*this);
    }

private:

    //! \brief The function to evaluate when the condition runs.
    Function m_func = nullptr;
};

} // namespace robotik::bt
