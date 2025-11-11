/**
 * @file BehaviorTreeManager.cpp
 * @brief Manager for behavior tree lifecycle and execution implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Managers/BehaviorTreeManager.hpp"

#include "Robotik/Core/BehaviorTree/Actions/RobotActions.hpp"
#include "Robotik/Core/BehaviorTree/Builder.hpp"
#include "Robotik/Core/Robot/Robot.hpp"
#include "Robotik/Core/Robot/TeachPendant.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"

#include <iostream>

namespace robotik
{

// ----------------------------------------------------------------------------
BehaviorTreeManager::BehaviorTreeManager()
    : m_factory(std::make_unique<bt::NodeFactory>()),
      m_blackboard(std::make_shared<bt::Blackboard>())
{
}

// ----------------------------------------------------------------------------
bool BehaviorTreeManager::loadTree(std::string const& p_filepath)
{
    m_error.clear();

    // Load tree from file using the builder
    auto result = bt::Builder::fromFile(*m_factory, p_filepath, m_blackboard);
    if (!result)
    {
        m_error = "Failed to load behavior tree: " + result.getError();
        return false;
    }

    // Store the tree
    m_tree = result.moveValue();
    m_playing = false;

    // Emit signal
    onTreeLoaded(m_tree.get());

    return true;
}

// ----------------------------------------------------------------------------
bt::Tree* BehaviorTreeManager::getTree()
{
    return m_tree.get();
}

// ----------------------------------------------------------------------------
bt::Tree const* BehaviorTreeManager::getTree() const
{
    return m_tree.get();
}

// ----------------------------------------------------------------------------
bt::Blackboard::Ptr BehaviorTreeManager::getBlackboard()
{
    return m_blackboard;
}

// ----------------------------------------------------------------------------
bt::Blackboard::Ptr BehaviorTreeManager::getBlackboard() const
{
    return m_blackboard;
}

// ----------------------------------------------------------------------------
bt::NodeFactory& BehaviorTreeManager::getFactory()
{
    return *m_factory;
}

// ----------------------------------------------------------------------------
bt::NodeFactory const& BehaviorTreeManager::getFactory() const
{
    return *m_factory;
}

// ----------------------------------------------------------------------------
bt::Status BehaviorTreeManager::tick(float /*p_dt*/)
{
    if (!m_tree || !m_playing)
    {
        return bt::Status::INVALID;
    }

    // Execute one step of the tree
    bt::Status status = m_tree->tick();

    // Emit signals
    if (m_tree->hasRoot())
    {
        onNodeTicked(&m_tree->getRoot(), status);
    }

    // Stop if tree completed
    if (status != bt::Status::RUNNING)
    {
        m_playing = false;
    }

    return status;
}

// ----------------------------------------------------------------------------
void BehaviorTreeManager::reset()
{
    if (m_tree)
    {
        m_tree->reset();
    }
    m_playing = false;
}

// ----------------------------------------------------------------------------
bool BehaviorTreeManager::isPlaying() const
{
    return m_playing;
}

// ----------------------------------------------------------------------------
void BehaviorTreeManager::play()
{
    if (!m_tree)
    {
        m_error = "No tree loaded";
        return;
    }
    m_playing = true;
    onTreeExecuting();
}

// ----------------------------------------------------------------------------
void BehaviorTreeManager::stop()
{
    m_playing = false;
}

// ----------------------------------------------------------------------------
bt::Status BehaviorTreeManager::step(float /*p_dt*/)
{
    if (!m_tree)
    {
        return bt::Status::INVALID;
    }

    // Execute one single tick regardless of playing state
    bt::Status status = m_tree->tick();

    // Emit signals
    if (m_tree->hasRoot())
    {
        onNodeTicked(&m_tree->getRoot(), status);
    }

    return status;
}

// ----------------------------------------------------------------------------
void BehaviorTreeManager::registerRobotActions(
    Robot& p_robot,
    TeachPendant& p_teach_pendant,
    IKSolver& p_ik_solver,
    TrajectoryController& p_trajectory_controller)
{
    // Use the helper function from RobotActions
    robotik::registerRobotActions(*m_factory,
                                  p_robot,
                                  p_teach_pendant,
                                  p_ik_solver,
                                  p_trajectory_controller,
                                  m_blackboard);
}

// ----------------------------------------------------------------------------
std::string const& BehaviorTreeManager::error() const
{
    return m_error;
}

// ----------------------------------------------------------------------------
bool BehaviorTreeManager::hasTree() const
{
    return m_tree != nullptr;
}

} // namespace robotik
