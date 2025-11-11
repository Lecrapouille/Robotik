/**
 * @file BehaviorTree.hpp
 * @brief Behavior tree class.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 *
 * This project is based on this initial project:
 * BrainTree - A C++ behavior tree single header library.
 * https://github.com/arvidsson/BrainTree
 * Copyright 2015-2018 Par Arvidsson. All rights reserved.
 * Licensed under the MIT license
 */

#pragma once

#include "Robotik/Core/BehaviorTree/BehaviorTreeVisitor.hpp"
#include "Robotik/Core/BehaviorTree/BlackBoard.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <functional>

namespace bt
{

// Forward declarations
class ConstBehaviorTreeVisitor;
class BehaviorTreeVisitor;

// ****************************************************************************
//! \brief Enum representing the status of a node in the behavior tree.
// ****************************************************************************
enum class Status
{
    INVALID = 0, ///< The node is invalid (internal use only).
    RUNNING = 1, ///< The node is running.
    SUCCESS = 2, ///< The node is successful.
    FAILURE = 3, ///< The node is failed.
};

// ****************************************************************************
//! \brief Convert a node status to a string.
//! \param[in] status The status to convert.
//! \return The string representation of the status.
// ****************************************************************************
inline std::string const& to_string(Status status)
{
    static std::array<std::string, 5> const names = {
        "INVALID", "RUNNING", "SUCCESS", "FAILURE", "???"
    };
    return names[int(status) < 4 ? int(status) : 4];
}

// ****************************************************************************
//! \brief Base class for all nodes in the behavior tree.
// ****************************************************************************
class Node
{
public:

    using Ptr = std::unique_ptr<Node>;

    // ------------------------------------------------------------------------
    //! \brief Create a new node of type T.
    //! \param[in] args The arguments to pass to the constructor of T.
    //! \return A unique pointer to the new node.
    // ------------------------------------------------------------------------
    template <typename T, typename... Args>
    [[nodiscard]] static std::unique_ptr<T> create(Args&&... args)
    {
        static_assert(std::is_base_of_v<Node, T>, "T must inherit from Node");
        return std::make_unique<T>(std::forward<Args>(args)...);
    }

    // ------------------------------------------------------------------------
    //! \brief Destructor needed because of virtual methods.
    // ------------------------------------------------------------------------
    virtual ~Node() = default;

    // ------------------------------------------------------------------------
    //! \brief Execute the curent node.
    //! Call the onSetUp() method if the node was not running on previous tick.
    //! Call the onRunning() method if onSetUp() did not return FAILURE.
    //! Call the onTearDown() method if onRunning() did not return RUNNING.
    //! \return The status of the node (SUCCESS, FAILURE, RUNNING).
    // ------------------------------------------------------------------------
    [[nodiscard]] Status tick()
    {
        if (m_status != Status::RUNNING)
        {
            m_status = onSetUp();
        }
        if (m_status != Status::FAILURE)
        {
            m_status = onRunning();
            if (m_status != Status::RUNNING)
            {
                onTearDown(m_status);
            }
        }
        return m_status;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the status of the node.
    //! \return The status of the node.
    // ------------------------------------------------------------------------
    [[nodiscard]] inline Status getStatus() const
    {
        return m_status;
    }

    // ------------------------------------------------------------------------
    //! \brief Reset the status of the node to INVALID_STATUS. This will force
    //! the node to be re-initialized through the onSetUp() method on the
    //! next tick().
    // ------------------------------------------------------------------------
    virtual void reset()
    {
        m_status = Status::INVALID;
    }

    // ------------------------------------------------------------------------
    //! \brief Halt the execution of the node. This will call onHalt() if the
    //! node is currently running, then reset the node status.
    // ------------------------------------------------------------------------
    virtual void halt()
    {
        if (m_status == Status::RUNNING)
        {
            onHalt();
        }
        m_status = Status::INVALID;
    }

    // ------------------------------------------------------------------------
    //! \brief Method invoked by the method onSetUp() of the Tree class to be
    //! sure the whole tree is valid.
    //! \return True if the node is valid, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] virtual bool isValid() const = 0;

    // ------------------------------------------------------------------------
    //! \brief Accept a const visitor (read-only).
    //! \param[in] p_visitor The visitor to accept.
    // ------------------------------------------------------------------------
    virtual void accept(ConstBehaviorTreeVisitor& p_visitor) const = 0;

    // ------------------------------------------------------------------------
    //! \brief Accept a non-const visitor (read-write).
    //! \param[in] p_visitor The visitor to accept.
    // ------------------------------------------------------------------------
    virtual void accept(BehaviorTreeVisitor& p_visitor) = 0;

protected:

    // ------------------------------------------------------------------------
    //! \brief Method invoked by the method tick(), when transitioning from the
    //! state RUNNING. This is a convenient place to setup the node when needed.
    //! \details By default nothing is done, override to handle the node
    //! startup logic.
    //! \return FAILURE if the node could not be initialized, else return
    //! SUCCESS or RUNNING if the node could be initialized.
    // ------------------------------------------------------------------------
    [[nodiscard]] virtual Status onSetUp()
    {
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Method invoked by the method tick() when the action is already in
    //! the RUNNING state.
    //! \details This method shall be overridden to handle the node running
    //! logic.
    //! \return The status of the node (SUCCESS, FAILURE, RUNNING).
    // ------------------------------------------------------------------------
    [[nodiscard]] virtual Status onRunning() = 0;

    // ------------------------------------------------------------------------
    //! \brief Method invoked by the method tick() when the action is no longer
    //! RUNNING. This is a convenient place for a cleanup the node when needed.
    //! \details By default nothing is done, override to handle the node
    //! cleanup logic.
    //! \param[in] status The status of the node (SUCCESS or FAILURE).
    // ------------------------------------------------------------------------
    virtual void onTearDown(Status p_status)
    {
        (void)p_status;
    }

    // ------------------------------------------------------------------------
    //! \brief Method invoked when halt() is called on a RUNNING node.
    //! \details By default nothing is done, override to handle cleanup when
    //! the node is interrupted.
    // ------------------------------------------------------------------------
    virtual void onHalt()
    {
        // Default implementation does nothing
    }

    //! \brief The status of the node.
    Status m_status = Status::INVALID;

public:

    //! \brief The name of the node.
    std::string name;
    //! \brief The type of the node.
    std::string m_type = "unknown";
};

// ****************************************************************************
//! \brief
// ****************************************************************************
class Tree: public Node
{
public:

    using Ptr = std::unique_ptr<Tree>;

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    // ------------------------------------------------------------------------
    Tree() = default;

    // ------------------------------------------------------------------------
    //! \brief Move constructor.
    // ------------------------------------------------------------------------
    Tree(Tree&&) = default;

    // ------------------------------------------------------------------------
    //! \brief Move assignment operator.
    // ------------------------------------------------------------------------
    Tree& operator=(Tree&&) = default;

    // ------------------------------------------------------------------------
    //! \brief Delete copy constructor.
    // ------------------------------------------------------------------------
    Tree(const Tree&) = delete;

    // ------------------------------------------------------------------------
    //! \brief Delete copy assignment operator.
    // ------------------------------------------------------------------------
    Tree& operator=(const Tree&) = delete;

    // ------------------------------------------------------------------------
    //! \brief Create a new tree.
    //! \return A unique pointer to the new tree.
    // ------------------------------------------------------------------------
    static Ptr create()
    {
        return std::make_unique<Tree>();
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the tree is valid before starting the tree.
    //! \details The tree is valid if it has a root node and all nodes in the
    //! tree are valid (i.e. if decorators and composites have children).
    //! \return True if the tree is valid, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool isValid() const override
    {
        return (m_root != nullptr) && (m_root->isValid());
    }

    // ------------------------------------------------------------------------
    //! \brief Reset the tree state and recursively reset all nodes.
    // ------------------------------------------------------------------------
    void reset() override
    {
        if (m_root != nullptr)
        {
            m_root->reset();
        }
        m_status = Status::INVALID;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the tree has a root node.
    //! \return True if the tree has a root node, false otherwise.
    // ------------------------------------------------------------------------
    inline bool hasRoot() const
    {
        return m_root != nullptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Create and set the root node of the behavior tree.
    //! \param[in] p_args The arguments to pass to the constructor of T.
    //! \return A reference to the new root node.
    // ------------------------------------------------------------------------
    template <class T, typename... Args>
    [[nodiscard]] inline T& createRoot(Args&&... p_args)
    {
        static_assert(std::is_base_of_v<Node, T>, "T must inherit from Node");
        auto root = Node::create<T>(std::forward<Args>(p_args)...);
        T* ptr = root.get();
        m_root = std::move(root);
        return *ptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the root node of the behavior tree.
    //! \param[in] p_root Pointer to the root node.
    // ------------------------------------------------------------------------
    void setRoot(Node::Ptr p_root)
    {
        m_root = std::move(p_root);
    }

    // ------------------------------------------------------------------------
    //! \brief Get the root node of the behavior tree (const version).
    //! \return Const reference to the root node.
    // ------------------------------------------------------------------------
    [[nodiscard]] Node const& getRoot() const
    {
        return *m_root;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the root node of the behavior tree (non-const version).
    //! \return Reference to the root node.
    // ------------------------------------------------------------------------
    [[nodiscard]] Node& getRoot()
    {
        return *m_root;
    }

    // ------------------------------------------------------------------------
    //! \brief Halt the entire tree recursively.
    // ------------------------------------------------------------------------
    void haltTree()
    {
        if (m_root != nullptr)
        {
            m_root->halt();
        }
        m_status = Status::INVALID;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the path of the node from the root node.
    //! \return The path of the node.
    // ------------------------------------------------------------------------
    // [[nodiscard]] Node const& getPath(std::string const& p_path) const
    // {
    //     return {}; // TODO
    // }

    // ------------------------------------------------------------------------
    //! \brief Set the blackboard of the tree
    //! \param[in] p_blackboard The blackboard to set
    // ------------------------------------------------------------------------
    void setBlackboard(Blackboard::Ptr p_blackboard)
    {
        m_blackboard = p_blackboard;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the blackboard of the tree
    //! \return A pointer to the blackboard
    // ------------------------------------------------------------------------
    [[nodiscard]] inline Blackboard::Ptr blackboard() const
    {
        return m_blackboard;
    }

    // ------------------------------------------------------------------------
    //! \brief Accept a const visitor (read-only).
    //! \param[in] p_visitor The visitor to accept.
    // ------------------------------------------------------------------------
    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitTree(*this);
    }

    // ------------------------------------------------------------------------
    //! \brief Accept a non-const visitor (read-write).
    //! \param[in] p_visitor The visitor to accept.
    // ------------------------------------------------------------------------
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitTree(*this);
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Check if the tree is valid before starting the tree.
    //! \return The status of the tree: SUCCESS if the tree is valid,
    //! FAILURE otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        // We need a root node to be able to tick the tree
        if (m_root == nullptr)
        {
            return Status::FAILURE;
        }

        // Check if the root node is valid
        if (!m_root->isValid())
        {
            return Status::FAILURE;
        }

        return Status::SUCCESS;
    }

    // ------------------------------------------------------------------------
    //! \brief Tick the root node of the tree.
    //! \return The status of the root node.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        assert(m_root != nullptr && "Root node is not set");
        m_status = m_root->tick();
        return m_status;
    }

private:

    //! \brief The root node of the behavior tree.
    Node::Ptr m_root = nullptr;
    //! \brief The blackboard for the node.
    Blackboard::Ptr m_blackboard = nullptr;
};

// ****************************************************************************
//! \brief Base class for composite nodes that can have multiple children.
//! Composite nodes are used to control the flow of the behavior tree.
// ****************************************************************************
class Composite: public Node
{
public:

    // ------------------------------------------------------------------------
    //! \brief Add an existing node as a child.
    //! \param[in] p_child Pointer to the child node to add.
    // ------------------------------------------------------------------------
    void addChild(Node::Ptr p_child)
    {
        m_children.emplace_back(std::move(p_child));
    }

    // ------------------------------------------------------------------------
    //! \brief Create and add a new child node of type T.
    //! \param[in] p_args The arguments to pass to the constructor of T.
    //! \return A reference to the new child node.
    // ------------------------------------------------------------------------
    template <class T, typename... Args>
    [[nodiscard]] inline T& addChild(Args&&... p_args)
    {
        static_assert(std::is_base_of_v<Node, T>, "T must inherit from Node");
        auto child = Node::create<T>(std::forward<Args>(p_args)...);
        T* ptr = child.get();
        m_children.emplace_back(std::move(child));
        return *ptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the composite node has children.
    //! \return True if the composite node has children, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] inline bool hasChildren() const
    {
        return !m_children.empty();
    }

    // ------------------------------------------------------------------------
    //! \brief Get the children nodes (const version)
    //! \return Const reference to the vector of child nodes
    // ------------------------------------------------------------------------
    [[nodiscard]] std::vector<Node::Ptr> const& getChildren() const
    {
        return m_children;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the children nodes (non-const version)
    //! \return Reference to the vector of child nodes
    // ------------------------------------------------------------------------
    [[nodiscard]] std::vector<Node::Ptr>& getChildren()
    {
        return m_children;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the composite node is valid.
    //! \return True if the composite node is valid, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool isValid() const override
    {
        if (m_children.empty())
        {
            return false;
        }

        return std::all_of(m_children.begin(),
                           m_children.end(),
                           [](const auto& child) { return child->isValid(); });
    }

    // ------------------------------------------------------------------------
    //! \brief Reset the composite and all its children recursively.
    // ------------------------------------------------------------------------
    void reset() override
    {
        for (auto const& child : m_children)
        {
            child->reset();
        }
        m_status = Status::INVALID;
    }

    // ------------------------------------------------------------------------
    //! \brief Halt the composite and all its children recursively.
    // ------------------------------------------------------------------------
    void halt() override
    {
        for (auto const& child : m_children)
        {
            child->halt();
        }
        if (m_status == Status::RUNNING)
        {
            onHalt();
        }
        m_status = Status::INVALID;
    }

protected:

    //! \brief The children nodes of the composite node.
    std::vector<Node::Ptr> m_children;
    //! \brief The iterator to the current child node.
    std::vector<Node::Ptr>::iterator m_iterator;
};

// ****************************************************************************
//! \brief The Sequence composite ticks each child node in order. If a child
//! fails or runs, the sequence returns the same status.  In the next tick, it
//! will try to run each child in order again.  If all children succeeds, only
//! then does the sequence succeed.
// ****************************************************************************
class Sequence: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Sequence".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Sequence";
    }

    // ------------------------------------------------------------------------
    //! \brief Set up the sequence.
    //! \return The status of the sequence.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        m_iterator = m_children.begin();
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the sequence.
    //! \return The status of the sequence.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        while (m_iterator != m_children.end())
        {
            if (auto status = (*m_iterator)->tick(); status != Status::SUCCESS)
            {
                return status;
            }

            m_iterator++;
        }

        return Status::SUCCESS;
    }

    // ------------------------------------------------------------------------
    //! \brief Accept a visitor.
    // ------------------------------------------------------------------------
    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitSequence(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitSequence(*this);
    }
};

// ****************************************************************************
//! \brief The ReactiveSequence composite ticks each child node in order. If a
//! child fails or runs, the reactive sequence returns the same status.  In the
//! next tick, it will try to run each child in order again.  If all children
//! succeeds, only then does the reactive sequence succeed.
// ****************************************************************************
class ReactiveSequence final: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Run the reactive sequence.
    //! \return The status of the reactive sequence.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        m_iterator = m_children.begin();
        while (m_iterator != m_children.end())
        {
            if (auto status = (*m_iterator)->tick(); status != Status::SUCCESS)
            {
                return status;
            }

            m_iterator++;
        }

        return Status::SUCCESS;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitReactiveSequence(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitReactiveSequence(*this);
    }
};

// ****************************************************************************
//! \brief The StatefulSequence composite ticks each child node in order, and
//! remembers what child it previously tried to tick.  If a child fails or runs,
//! the stateful sequence returns the same status.  In the next tick, it will
//! try to run the next child or start from the beginning again.  If all
//! children succeeds, only then does the stateful sequence succeed.
// ****************************************************************************
class StatefulSequence final
    : public Composite // Sequence with Memory → ContinueInOrder
{
public:

    // ------------------------------------------------------------------------
    //! \brief Run the stateful sequence.
    //! \return The status of the stateful sequence.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        while (m_iterator != m_children.end())
        {
            if (auto status = (*m_iterator)->tick(); status != Status::SUCCESS)
            {
                return status;
            }

            m_iterator++;
        }

        m_iterator = m_children.begin();
        return Status::SUCCESS;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitStatefulSequence(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitStatefulSequence(*this);
    }
};

// ****************************************************************************
//! \brief The Selector (aka Fallback) composite ticks each child node in order.
//! If a child succeeds or runs, the selector returns the same status.  In the
//! next tick, it will try to run each child in order again.  If all children
//! fails, only then does the selector fail.
// ****************************************************************************
class Selector final: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Selector".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Selector";
    }

    // ------------------------------------------------------------------------
    //! \brief Set up the selector.
    //! \return The status of the selector.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        m_iterator = m_children.begin();
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the selector.
    //! \return The status of the selector.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        while (m_iterator != m_children.end())
        {
            if (Status status = (*m_iterator)->tick();
                status != Status::FAILURE)
            {
                return status;
            }

            m_iterator++;
        }

        return Status::FAILURE;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitSelector(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitSelector(*this);
    }
};

// ****************************************************************************
//! \brief The ReactiveSelector composite ticks each child node in order. If a
//! child succeeds or runs, the reactive selector returns the same status.  In
//! the next tick, it will try to run each child in order again.  If all
//! children fails, only then does the reactive selector fail.
// ****************************************************************************
class ReactiveSelector final: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Run the reactive selector.
    //! \return The status of the reactive selector.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        m_iterator = m_children.begin();
        while (m_iterator != m_children.end())
        {
            if (Status status = (*m_iterator)->tick();
                status != Status::FAILURE)
            {
                return status;
            }

            m_iterator++;
        }

        return Status::FAILURE;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitReactiveSelector(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitReactiveSelector(*this);
    }
};

// ****************************************************************************
//! \brief The StatefulSelector composite ticks each child node in order, and
//! remembers what child it previously tried to tick.  If a child succeeds or
//! runs, the stateful selector returns the same status.  In the next tick, it
//! will try to run the next child or start from the beginning again.  If all
//! children fails, only then does the stateful selector fail.
// ****************************************************************************
class StatefulSelector final: public Composite
{
public:

    // ------------------------------------------------------------------------
    //! \brief Run the stateful selector.
    //! \return The status of the stateful selector.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        while (m_iterator != m_children.end())
        {
            if (Status status = (*m_iterator)->tick();
                status != Status::FAILURE)
            {
                return status;
            }

            m_iterator++;
        }

        m_iterator = m_children.begin();
        return Status::FAILURE;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitStatefulSelector(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitStatefulSelector(*this);
    }
};

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
    //! \param[in] minSuccess minimum successful children needed
    //! \param[in] minFail minimum failed children needed
    // ------------------------------------------------------------------------
    Parallel(int minSuccess, int minFail)
        : m_minSuccess(minSuccess), m_minFail(minFail)
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
    //! \brief Constructor taking success and failure policies.
    //! \param[in] successOnAll if true requires all children to succeed
    //! \param[in] failOnAll if true requires all children to fail
    // ------------------------------------------------------------------------
    explicit ParallelAll(bool successOnAll = true, bool failOnAll = true)
        : m_successOnAll(successOnAll), m_failOnAll(failOnAll)
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

// ****************************************************************************
//! \brief Base class for decorator nodes that can have only one child.
//! Decorator nodes are used to modify the behavior of their child node.
// ****************************************************************************
class Decorator: public Node
{
public:

    // ------------------------------------------------------------------------
    //! \brief Set the child node of the decorator.
    //! \param[in] child The child node to set.
    // ------------------------------------------------------------------------
    void setChild(Node::Ptr p_child)
    {
        m_child = std::move(p_child);
    }

    // ------------------------------------------------------------------------
    //! \brief Set the child node of the decorator.
    //! \param[in] args The arguments to pass to the constructor of T
    //! \return A reference to the new child node
    // ------------------------------------------------------------------------
    template <class T, typename... Args>
    [[nodiscard]] inline T& createChild(Args&&... p_args)
    {
        auto child = Node::create<T>(std::forward<Args>(p_args)...);
        T* ptr = child.get();
        m_child = std::move(child);
        return *ptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the decorator has a child node.
    //! \return True if the decorator has a child node, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] inline bool hasChild() const
    {
        return m_child != nullptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the child node of the decorator (const version).
    //! \note The child node shall exist. Use hasChild() to check.
    //! \return The child node reference.
    // ------------------------------------------------------------------------
    [[nodiscard]] inline Node const& getChild() const
    {
        return *(m_child.get());
    }

    // ------------------------------------------------------------------------
    //! \brief Get the child node of the decorator (non-const version).
    //! \note The child node shall exist. Use hasChild() to check.
    //! \return The child node reference.
    // ------------------------------------------------------------------------
    [[nodiscard]] inline Node& getChild()
    {
        return *(m_child.get());
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the composite node is valid.
    //! \return True if the composite node is valid, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool isValid() const override
    {
        return (m_child != nullptr) && (m_child->isValid());
    }

    // ------------------------------------------------------------------------
    //! \brief Reset the decorator and its child recursively.
    // ------------------------------------------------------------------------
    void reset() override
    {
        if (m_child != nullptr)
        {
            m_child->reset();
        }
        m_status = Status::INVALID;
    }

    // ------------------------------------------------------------------------
    //! \brief Halt the decorator and its child recursively.
    // ------------------------------------------------------------------------
    void halt() override
    {
        if (m_child != nullptr)
        {
            m_child->halt();
        }
        if (m_status == Status::RUNNING)
        {
            onHalt();
        }
        m_status = Status::INVALID;
    }

protected:

    Node::Ptr m_child = nullptr;
};

// ****************************************************************************
//! \brief The ForceSuccess decorator returns RUNNING if the child is RUNNING,
//! else returns SUCCESS, regardless of what happens to the child.
// ****************************************************************************
class ForceSuccess final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "ForceSuccess".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "ForceSuccess";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the succeeder.
    //! \return The status of the succeeder.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        Status status = m_child->tick();
        return (status == Status::RUNNING) ? Status::RUNNING : Status::SUCCESS;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitForceSuccess(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitForceSuccess(*this);
    }
};

// ****************************************************************************
//! \brief The ForceFailure decorator returns RUNNING if the child is RUNNING,
//! else returns FAILURE, regardless of what happens to the child.
// ****************************************************************************
class ForceFailure final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "ForceFailure".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "ForceFailure";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the failer.
    //! \return The status of the failer.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        Status status = m_child->tick();
        return (status == Status::RUNNING) ? Status::RUNNING : Status::FAILURE;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitForceFailure(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitForceFailure(*this);
    }
};

// ****************************************************************************
//! \brief The Inverter decorator returns RUNNING if the child is RUNNING,
//! else returns the opposite of the child's status, i.e. FAILURE becomes
//! SUCCESS and SUCCESS becomes FAILURE.
// ****************************************************************************
class Inverter final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Inverter".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Inverter";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the inverter.
    //! \return The status of the inverter.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        auto s = m_child->tick();
        if (s == Status::SUCCESS)
        {
            return Status::FAILURE;
        }
        else if (s == Status::FAILURE)
        {
            return Status::SUCCESS;
        }

        return s;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitInverter(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitInverter(*this);
    }
};

// ****************************************************************************
//! \brief The Repeater decorator repeats infinitely or to a limit until the
//! child returns success.
//! \fixme the loop should be done entirely internally during the tick() method
//! ?
// ****************************************************************************
class Repeat final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Repeat".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Repeat";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a limit of repetitions.
    //! \param[in] p_repetitions The limit of repetitions.
    // ------------------------------------------------------------------------
    explicit Repeat(size_t p_repetitions = 0) : m_repetitions(p_repetitions) {}

    // ------------------------------------------------------------------------
    //! \brief Set up the repeater.
    //! \return The status of the repeater.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        m_count = 0;
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the repeater.
    //! \fixme Should be a loop inside the tick() method like done in libBT.cpp?
    //! \return The status of the repeater.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        if (Status status = m_child->tick(); status == Status::RUNNING)
        {
            return Status::RUNNING;
        }
        else if (status == Status::FAILURE)
        {
            return Status::FAILURE;
        }

        if (m_repetitions > 0)
        {
            ++m_count;
            if (m_count == m_repetitions)
            {
                m_count = m_repetitions;
                return Status::SUCCESS;
            }
        }

        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current count of repetitions.
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getCount() const
    {
        return m_count;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the limit number of repetitions.
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getRepetitions() const
    {
        return m_repetitions;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitRepeat(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitRepeat(*this);
    }

private:

    size_t m_count = 0;
    size_t m_repetitions;
};

// ****************************************************************************
//! \brief The Retry decorator retries its child a specified number of times
//! until it succeeds or the maximum number of attempts is reached.
// ****************************************************************************
class Retry final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Retry".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Retry";
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a number of attempts.
    //! \param[in] p_attempts The maximum number of attempts.
    // ------------------------------------------------------------------------
    explicit Retry(size_t p_attempts) : m_attempts(p_attempts) {}

    // ------------------------------------------------------------------------
    //! \brief Set up the retry.
    //! \return The status of the retry.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onSetUp() override
    {
        m_count = 0;
        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Run the retry.
    //! \return The status of the retry.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        if (Status status = m_child->tick(); status == Status::SUCCESS)
        {
            return Status::SUCCESS;
        }
        else if (status == Status::RUNNING)
        {
            return Status::RUNNING;
        }

        if (m_attempts > 0)
        {
            ++m_count;
            if (m_count >= m_attempts)
            {
                return Status::FAILURE;
            }
        }

        return Status::RUNNING;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current count of attempts.
    //! \return The current count of attempts.
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getCount() const
    {
        return m_count;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the limit number of attempts.
    //! \return The limit number of attempts.
    // ------------------------------------------------------------------------
    [[nodiscard]] size_t getAttempts() const
    {
        return m_attempts;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitRetry(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitRetry(*this);
    }

private:

    size_t m_count = 0;
    size_t m_attempts;
};

// ****************************************************************************
//! \brief The UntilSuccess decorator repeats until the child returns success
//! and then returns success.
// ****************************************************************************
class UntilSuccess final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "RepeatUntilSuccess".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "RepeatUntilSuccess";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the until success decorator.
    //! \return The status of the until success decorator.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        while (true)
        {
            auto status = m_child->tick();
            if (status == Status::SUCCESS)
            {
                return Status::SUCCESS;
            }
        }
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitUntilSuccess(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitUntilSuccess(*this);
    }
};

// ****************************************************************************
//! \brief The UntilFailure decorator repeats until the child returns fail and
//! then returns success.
// ****************************************************************************
class UntilFailure final: public Decorator
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "RepeatUntilFailure".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "RepeatUntilFailure";
    }

    // ------------------------------------------------------------------------
    //! \brief Execute the decorator.
    //! \return The status of the decorator.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        while (true)
        {
            auto status = m_child->tick();
            if (status == Status::FAILURE)
            {
                return Status::SUCCESS;
            }
        }
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitUntilFailure(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitUntilFailure(*this);
    }
};

// ****************************************************************************
//! \brief Base class for leaf nodes that have no children.
//! Leaf nodes are the nodes that actually do the work.
// ****************************************************************************
class Leaf: public Node
{
public:

    // ------------------------------------------------------------------------
    //! \brief Default constructor without blackboard.
    // ------------------------------------------------------------------------
    Leaf() = default;

    // ------------------------------------------------------------------------
    //! \brief Constructor with blackboard.
    // ------------------------------------------------------------------------
    explicit Leaf(Blackboard::Ptr p_blackboard) : m_blackboard(p_blackboard) {}

    // ------------------------------------------------------------------------
    //! \brief Get the blackboard for the node
    //! \return The blackboard for the node
    // ------------------------------------------------------------------------
    [[nodiscard]] inline Blackboard::Ptr getBlackboard() const
    {
        return m_blackboard;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the leaf node is valid.
    //! \return True if the leaf node is valid, false otherwise.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool isValid() const override
    {
        return true;
    }

protected:

    //! \brief The blackboard for the node
    Blackboard::Ptr m_blackboard = nullptr;
};

// ****************************************************************************
//! \brief Simple leaf that always returns SUCCESS.
// ****************************************************************************
class Success final: public Leaf
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Success".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Success";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the always success leaf.
    //! \return The status of the always success leaf.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        return Status::SUCCESS;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitSuccess(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitSuccess(*this);
    }
};

// ****************************************************************************
//! \brief Simple leaf that always returns FAILURE.
// ****************************************************************************
class Failure final: public Leaf
{
public:

    // ------------------------------------------------------------------------
    //! \brief Get the string representation of the node type.
    //! \return The string "Failure".
    // ------------------------------------------------------------------------
    [[nodiscard]] static constexpr char const* toString()
    {
        return "Failure";
    }

    // ------------------------------------------------------------------------
    //! \brief Run the always failure leaf.
    //! \return The status of the always failure leaf.
    // ------------------------------------------------------------------------
    [[nodiscard]] Status onRunning() override
    {
        return Status::FAILURE;
    }

    void accept(ConstBehaviorTreeVisitor& p_visitor) const override
    {
        p_visitor.visitFailure(*this);
    }
    void accept(BehaviorTreeVisitor& p_visitor) override
    {
        p_visitor.visitFailure(*this);
    }
};

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
    explicit SugarAction(Function func) : m_func(std::move(func)) {}

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a function and blackboard.
    //! \param[in] func The function to execute when the action runs.
    //! \param[in] blackboard The blackboard to use.
    // ------------------------------------------------------------------------
    SugarAction(Function func, Blackboard::Ptr blackboard)
        : Leaf(blackboard), m_func(std::move(func))
    {
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
    //! \param[in] func The function to evaluate when the condition runs.
    // ------------------------------------------------------------------------
    explicit Condition(Function func) : m_func(std::move(func)) {}

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a function and blackboard.
    //! \param[in] func The function to evaluate when the condition runs.
    //! \param[in] blackboard The blackboard to use.
    // ------------------------------------------------------------------------
    Condition(Function func, Blackboard::Ptr blackboard)
        : Leaf(blackboard), m_func(std::move(func))
    {
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

// ****************************************************************************
//! \brief Factory class for creating behavior tree nodes.
//! This class allows registering custom node types that can be created by name.
// ****************************************************************************
class NodeFactory
{
public:

    // ------------------------------------------------------------------------
    //! \brief Function type for creating nodes.
    // ------------------------------------------------------------------------
    using NodeCreator = std::function<std::unique_ptr<Node>()>;

    // ------------------------------------------------------------------------
    //! \brief Default destructor.
    // ------------------------------------------------------------------------
    virtual ~NodeFactory() = default;

    // ------------------------------------------------------------------------
    //! \brief Register a node type with a creation function.
    //! \param[in] p_name Name used to identify this node type.
    //! \param[in] p_creator Function that creates instances of this node type.
    // ------------------------------------------------------------------------
    void registerNode(const std::string& p_name, NodeCreator p_creator)
    {
        m_creators[p_name] = std::move(p_creator);
    }

    // ------------------------------------------------------------------------
    //! \brief Create a node instance by name.
    //! \param[in] p_name The registered name of the node type to create.
    //! \return Unique pointer to the new node instance, or nullptr if name not
    //! found.
    // ------------------------------------------------------------------------
    std::unique_ptr<Node> createNode(const std::string& p_name) const
    {
        if (auto it = m_creators.find(p_name); it != m_creators.end())
        {
            return it->second();
        }
        return nullptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if a node type is registered.
    //! \param[in] p_name The name to check.
    //! \return True if the node type is registered.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool hasNode(const std::string& p_name) const
    {
        return m_creators.find(p_name) != m_creators.end();
    }

    // ------------------------------------------------------------------------
    //! \brief Helper method to register an action with a lambda.
    //! \param[in] p_name Name used to identify this action.
    //! \param[in] p_func Lambda function implementing the action.
    // ------------------------------------------------------------------------
    void registerAction(const std::string& p_name,
                        SugarAction::Function&& p_func)
    {
        registerNode(p_name,
                     [func = std::move(p_func)]()
                     { return Node::create<SugarAction>(func); });
    }

    // ------------------------------------------------------------------------
    //! \brief Helper method to register an action with a lambda and blackboard.
    //! \param[in] name Name used to identify this action.
    //! \param[in] p_func Lambda function implementing the action.
    //! \param[in] p_blackboard The blackboard to use.
    // ------------------------------------------------------------------------
    void registerAction(const std::string& p_name,
                        SugarAction::Function&& p_func,
                        Blackboard::Ptr p_blackboard)
    {
        registerNode(p_name,
                     [func = std::move(p_func), p_blackboard]()
                     { return Node::create<SugarAction>(func, p_blackboard); });
    }

    // ------------------------------------------------------------------------
    //! \brief Helper template method to register a node with blackboard.
    //! \tparam T The node type to register.
    //! \param[in] p_name Name used to identify this node type.
    //! \param[in] p_blackboard The blackboard to use.
    // ------------------------------------------------------------------------
    template <typename T>
    void registerNode(const std::string& p_name, Blackboard::Ptr p_blackboard)
    {
        registerNode(
            p_name, [p_blackboard]() { return Node::create<T>(p_blackboard); });
    }

    // ------------------------------------------------------------------------
    //! \brief Helper template method to register a node without blackboard.
    //! \tparam T The node type to register.
    //! \param[in] p_name Name used to identify this node type.
    // ------------------------------------------------------------------------
    template <typename T>
    void registerNode(const std::string& p_name)
    {
        registerNode(p_name, []() { return Node::create<T>(); });
    }

    // ------------------------------------------------------------------------
    //! \brief Helper method to register a condition with a lambda.
    //! \param[in] p_name Name used to identify this condition.
    //! \param[in] p_func Lambda function implementing the condition.
    // ------------------------------------------------------------------------
    void registerCondition(const std::string& p_name,
                           Condition::Function&& p_func)
    {
        registerNode(p_name,
                     [func = std::move(p_func)]()
                     { return Node::create<Condition>(func); });
    }

    // ------------------------------------------------------------------------
    //! \brief Helper method to register a condition with a lambda and
    //! blackboard.
    //! \param[in] p_name Name used to identify this condition.
    //! \param[in] p_func Lambda function implementing the condition.
    //! \param[in] p_blackboard The blackboard to use.
    // ------------------------------------------------------------------------
    void registerCondition(const std::string& p_name,
                           Condition::Function&& p_func,
                           Blackboard::Ptr p_blackboard)
    {
        registerNode(p_name,
                     [func = std::move(p_func), p_blackboard]()
                     { return Node::create<Condition>(func, p_blackboard); });
    }

protected:

    //! \brief Map of node names to their creation functions
    std::unordered_map<std::string, NodeCreator> m_creators;
};

} // namespace bt