/**
 * @file Wait.hpp
 * @brief Wait leaf node.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Leaf.hpp"

#include <chrono>

namespace robotik::bt {

// ****************************************************************************
//! \brief The Wait leaf waits for a specified duration and then returns
//! SUCCESS. This is useful for adding delays in behavior tree execution.
// ****************************************************************************
class Wait final: public Leaf
{
public:

    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::milliseconds;

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Wait".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Wait";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a wait duration.
    //! \param[in] p_milliseconds The wait duration in milliseconds.
    // ------------------------------------------------------------------------
    explicit Wait(size_t p_milliseconds = 1000)
        : m_duration(Duration(p_milliseconds))
    {
    }

    // ------------------------------------------------------------------------
    //! \brief Set up the wait - records the start time.
    //! \return RUNNING to continue with onRunning.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        m_start_time = Clock::now();
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the wait leaf.
    //! \return RUNNING until duration has passed, then SUCCESS.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        auto elapsed =
            std::chrono::duration_cast<Duration>(Clock::now() - m_start_time);

        if (elapsed >= m_duration)
        {
            return Status::SUCCESS;
        }

        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the wait duration in milliseconds.
    //! \return The wait duration.
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getMilliseconds() const
    {
        return static_cast<size_t>(m_duration.count());
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitWait(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitWait(*this);
    }

private:

    Duration m_duration;
    Clock::time_point m_start_time;
};

} // namespace robotik::bt
