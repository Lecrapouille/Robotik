/**
 * @file BehaviorTreeManager.hpp
 * @brief Manager for behavior tree lifecycle and execution.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/BehaviorTree/BehaviorTree.hpp"
#include "Robotik/Core/Common/Signal.hpp"

#include <memory>
#include <string>

namespace robotik
{

// Forward declarations
class Robot;
class TeachPendant;
class IKSolver;
class TrajectoryController;

// ****************************************************************************
//! \brief Manager for behavior tree lifecycle and execution.
//!
//! This class manages behavior trees loaded from YAML files, handles execution
//! control, and provides a NodeFactory for registering custom actions.
//!
//! Key features:
//! - Load behavior trees from YAML files
//! - Manage NodeFactory with custom action registration
//! - Control tree execution (play, stop, step, reset)
//! - Signal system for tree events
//! - Error handling
// ****************************************************************************
class BehaviorTreeManager
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    // ------------------------------------------------------------------------
    BehaviorTreeManager();

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~BehaviorTreeManager() = default;

    // ------------------------------------------------------------------------
    //! \brief Signal emitted when a tree is loaded.
    //! \param p_tree Pointer to the loaded tree.
    // ------------------------------------------------------------------------
    Signal<bt::Tree*> onTreeLoaded;

    // ------------------------------------------------------------------------
    //! \brief Signal emitted when tree starts executing.
    // ------------------------------------------------------------------------
    Signal<> onTreeExecuting;

    // ------------------------------------------------------------------------
    //! \brief Signal emitted when a node is ticked.
    //! \param p_node Pointer to the ticked node.
    //! \param p_status Status returned by the node.
    // ------------------------------------------------------------------------
    Signal<bt::Node const*, bt::Status> onNodeTicked;

    // ------------------------------------------------------------------------
    //! \brief Load a behavior tree from a YAML file.
    //! \param p_filepath Path to the YAML file.
    //! \return true if successful, false otherwise.
    // ------------------------------------------------------------------------
    bool loadTree(std::string const& p_filepath);

    // ------------------------------------------------------------------------
    //! \brief Get the current tree.
    //! \return Pointer to the tree, nullptr if not loaded.
    // ------------------------------------------------------------------------
    bt::Tree* getTree();

    // ------------------------------------------------------------------------
    //! \brief Get the current tree (const version).
    //! \return Pointer to the tree, nullptr if not loaded.
    // ------------------------------------------------------------------------
    bt::Tree const* getTree() const;

    // ------------------------------------------------------------------------
    //! \brief Get the blackboard.
    //! \return Pointer to the blackboard.
    // ------------------------------------------------------------------------
    bt::Blackboard::Ptr getBlackboard();

    // ------------------------------------------------------------------------
    //! \brief Get the blackboard (const version).
    //! \return Pointer to the blackboard.
    // ------------------------------------------------------------------------
    bt::Blackboard::Ptr getBlackboard() const;

    // ------------------------------------------------------------------------
    //! \brief Get the node factory.
    //! \return Reference to the node factory.
    // ------------------------------------------------------------------------
    bt::NodeFactory& getFactory();

    // ------------------------------------------------------------------------
    //! \brief Get the node factory (const version).
    //! \return Reference to the node factory.
    // ------------------------------------------------------------------------
    bt::NodeFactory const& getFactory() const;

    // ------------------------------------------------------------------------
    //! \brief Tick the tree (execute one step).
    //! \param p_dt Delta time in seconds.
    //! \return Status of the root node.
    // ------------------------------------------------------------------------
    bt::Status tick(float p_dt);

    // ------------------------------------------------------------------------
    //! \brief Reset the tree to initial state.
    // ------------------------------------------------------------------------
    void reset();

    // ------------------------------------------------------------------------
    //! \brief Check if the tree is playing.
    //! \return true if playing, false otherwise.
    // ------------------------------------------------------------------------
    bool isPlaying() const;

    // ------------------------------------------------------------------------
    //! \brief Start playing the tree.
    // ------------------------------------------------------------------------
    void play();

    // ------------------------------------------------------------------------
    //! \brief Stop playing the tree.
    // ------------------------------------------------------------------------
    void stop();

    // ------------------------------------------------------------------------
    //! \brief Execute one single tick (step-by-step execution).
    //! \param p_dt Delta time in seconds.
    //! \return Status of the root node.
    // ------------------------------------------------------------------------
    bt::Status step(float p_dt);

    // ------------------------------------------------------------------------
    //! \brief Register robot-specific actions in the factory.
    //! \param p_robot Robot to control.
    //! \param p_teach_pendant Teach pendant for robot control.
    //! \param p_ik_solver IK solver for cartesian movements.
    //! \param p_trajectory_controller Trajectory controller for smooth movements.
    // ------------------------------------------------------------------------
    void registerRobotActions(Robot& p_robot,
                              TeachPendant& p_teach_pendant,
                              IKSolver& p_ik_solver,
                              TrajectoryController& p_trajectory_controller);

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    std::string const& error() const;

    // ------------------------------------------------------------------------
    //! \brief Check if a tree is loaded.
    //! \return true if a tree is loaded, false otherwise.
    // ------------------------------------------------------------------------
    bool hasTree() const;

private:

    //! \brief Node factory for creating custom nodes.
    std::unique_ptr<bt::NodeFactory> m_factory;
    //! \brief Blackboard for sharing data between nodes.
    bt::Blackboard::Ptr m_blackboard;
    //! \brief Current behavior tree.
    bt::Tree::Ptr m_tree;
    //! \brief Playing state.
    bool m_playing = false;
    //! \brief Last error message.
    std::string m_error;
};

} // namespace robotik
