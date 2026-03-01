/**
 * @file SetBlackboard.hpp
 * @brief SetBlackboard leaf node.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Leaf.hpp"

#include <string>

namespace robotik::bt {

// ****************************************************************************
//! \brief The SetBlackboard leaf writes a value to the blackboard and
//! returns SUCCESS. This is useful for setting state variables during
//! tree execution.
// ****************************************************************************
class SetBlackboard final: public Leaf
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "SetBlackboard".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "SetBlackboard";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking key and value to set.
    //! \param[in] p_key The blackboard key to set.
    //! \param[in] p_value The value to set (as string).
    // ------------------------------------------------------------------------
    SetBlackboard(std::string p_key, std::string p_value)
        : m_key(std::move(p_key)), m_value(std::move(p_value))
    {
        m_type = toString();
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking key, value and blackboard.
    //! \param[in] p_key The blackboard key to set.
    //! \param[in] p_value The value to set (as string).
    //! \param[in] p_blackboard The blackboard to use.
    // ------------------------------------------------------------------------
    SetBlackboard(std::string p_key,
                  std::string p_value,
                  Blackboard::Ptr p_blackboard)
        : m_key(std::move(p_key)), m_value(std::move(p_value))
    {
        m_type = toString();
        setBlackboard(p_blackboard);
    }

    // ------------------------------------------------------------------------
    //! \brief Run the SetBlackboard leaf.
    //! \return SUCCESS after setting the value.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        if (m_blackboard)
        {
            m_blackboard->set(m_key, m_value);
        }
        return Status::SUCCESS;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the key that will be set.
    //! \return The blackboard key.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::string const& getKey() const
    {
        return m_key;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the value that will be set.
    //! \return The value as string.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::string const& getValue() const
    {
        return m_value;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitSetBlackboard(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitSetBlackboard(*this);
    }

private:

    std::string m_key;
    std::string m_value;
};

} // namespace robotik::bt
