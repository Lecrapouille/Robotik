/**
 * @file Parallels.hpp
 * @brief Parallel composite nodes: Parallel and ParallelAll.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Core/Composite.hpp"

#include <cassert>

namespace robotik::bt {

// ****************************************************************************
//! \brief The Parallel composite runs all children simultaneously.
//! It requires a minimum number of successful or failed children to determine
//! its own status.
// ****************************************************************************
class Parallel final: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Parallel".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Parallel";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking minimum success and failure counts.
    //! \param[in] p_minSuccess minimum successful children needed
    //! \param[in] p_minFail minimum failed children needed
    // ------------------------------------------------------------------------
    Parallel(int p_minSuccess, int p_minFail)
        : m_minSuccess(p_minSuccess), m_minFail(p_minFail)
    {
    }

    // ------------------------------------------------------------------------
    //! \brief Get the minimum number of successful children needed.
    //! \return The minimum number of successful children needed.
    // ------------------------------------------------------------------------
    [[nodiscard]] int getMinSuccess() const
    {
        return m_minSuccess;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the minimum number of failed children needed.
    //! \return The minimum number of failed children needed.
    // ------------------------------------------------------------------------
    [[nodiscard]] int getMinFail() const
    {
        return m_minFail;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the parallel composite.
    //! \return The status of the parallel composite.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        assert(hasChildren() && "Composite has no children");

        int total_success = 0;
        int total_fail = 0;

        for (auto const& child : m_children)
        {
            auto status = child->tick();
            if (status == Status::SUCCESS)
            {
                total_success++;
            }
            if (status == Status::FAILURE)
            {
                total_fail++;
            }
        }

        if (total_success >= m_minSuccess)
        {
            return Status::SUCCESS;
        }
        if (total_fail >= m_minFail)
        {
            return Status::FAILURE;
        }

        return Status::RUNNING;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitParallel(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitParallel(*this);
    }

private:

    int m_minSuccess;
    int m_minFail;
};

// ****************************************************************************
//! \brief The ParallelAll composite runs all children simultaneously.
//! It uses success/failure policies to determine its own status.
// ****************************************************************************
class ParallelAll final: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "🪜 ParallelAll".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "ParallelAll";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking success and failure policies.
    //! \param[in] p_successOnAll if true requires all children to succeed
    //! \param[in] p_failOnAll if true requires all children to fail
    // ------------------------------------------------------------------------
    explicit ParallelAll(bool p_successOnAll = true, bool p_failOnAll = true)
        : m_successOnAll(p_successOnAll), m_failOnAll(p_failOnAll)
    {
    }

    // ------------------------------------------------------------------------
    //! \brief Run the parallel all composite.
    //! \return The status of the parallel all composite.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        assert(hasChildren() && "Composite has no children");

        const size_t minimumSuccess = m_successOnAll ? m_children.size() : 1;
        const size_t minimumFail = m_failOnAll ? m_children.size() : 1;

        size_t total_success = 0;
        size_t total_fail = 0;

        for (auto const& child : m_children)
        {
            auto status = child->tick();
            if (status == Status::SUCCESS)
            {
                total_success++;
            }
            if (status == Status::FAILURE)
            {
                total_fail++;
            }
        }

        if (total_success >= minimumSuccess)
        {
            return Status::SUCCESS;
        }
        if (total_fail >= minimumFail)
        {
            return Status::FAILURE;
        }

        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the success on all flag.
    //! \return The success on all flag.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool getSuccessOnAll() const
    {
        return m_successOnAll;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the fail on all flag.
    //! \return The fail on all flag.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool getFailOnAll() const
    {
        return m_failOnAll;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitParallelAll(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitParallelAll(*this);
    }

private:

    bool m_successOnAll;
    bool m_failOnAll;
};

} // namespace robotik::bt
