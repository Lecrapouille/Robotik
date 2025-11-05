/**
 * @file Robot.hpp
 * @brief Robot class - Orchestrates kinematics computations on Blueprint +
 * State.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Blueprint/Blueprint.hpp"
#include "Robotik/Core/Robot/State.hpp"

namespace robotik
{

// Forward declarations
class IKSolver;
class Frame;

// *********************************************************************************
//! \brief Class that performs kinematic computations on a robot model.
//!
//! This class operates on a Blueprint (static model) and State (dynamic data)
//! to perform forward kinematics, inverse kinematics, and Jacobian
//! computations.
//!
//! This design follows Pinocchio's Model/Data separation pattern:
//! - Blueprint == Model (read-only structure)
//! - State == Data (dynamic state)
//! - Robot == Algorithms (kinematics computations)
//!
//! Benefits of this architecture:
//! - Multiple states can be computed in parallel for the same robot
//! - Clean separation between model and state
//! - Efficient memory usage
//! - Reusable algorithms across different robot models
//!
//! Main capabilities:
//!
//! 1. FORWARD KINEMATICS: Given joint angles → compute link/joint transforms
//!    * Updates State with world transforms of all links and joints
//!    * Essential for: Visualization, collision detection, Jacobian computation
//!
//! 2. INVERSE KINEMATICS: Given desired end-effector pose → compute joint
//! angles
//!    * Uses pluggable IKSolver implementations
//!    * Updates State with computed joint positions
//!
//! 3. JACOBIAN COMPUTATION: Relationship between joint velocities and
//! end-effector velocity
//!    * Computes and caches Jacobian in State
//!    * J = ∂pose/∂joints (6×n matrix for n joints)
//!
//! Example usage:
//! \code
//!   // Parse URDF to get blueprint
//!   URDFLoader parser;
//!   Blueprint blueprint = parser.parse("robot.urdf");
//!
//!   // Create robot and state
//!   Robot robot(blueprint);
//!   State state = robot.createState();
//!
//!   // Set joint positions and compute forward kinematics
//!   state.jointPositions() = {0.0, 1.57, -0.5};
//!   robot.setJointPositions(state);
//!
//!   // Compute Jacobian for end effector
//!   auto& end_eff = blueprint.link("end_effector");
//!   robot.computeJacobian(state, end_eff);
//! \endcode
// *********************************************************************************
class Robot
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor taking a Blueprint (robot model).
    //! \param p_name The name of the robot.
    //! \param p_blueprint The robot's kinematic structure.
    // ------------------------------------------------------------------------
    Robot(std::string const& p_name, Blueprint&& p_blueprint);

    Robot(std::string const& p_name, Node::Ptr p_root);

    // ------------------------------------------------------------------------
    //! \brief Virtual destructor for proper cleanup of derived classes.
    //!
    //! This is essential because Robot is used polymorphically (e.g.,
    //! ControlledRobot inherits from Robot). Without a virtual destructor,
    //! deleting a derived class through a base class pointer would result
    //! in undefined behavior and memory leaks.
    // ------------------------------------------------------------------------
    virtual ~Robot() = default;

    // ------------------------------------------------------------------------
    //! \brief Get the name of the robot.
    //! \return The name of the robot.
    // ------------------------------------------------------------------------
    inline std::string const& name() const
    {
        return m_name;
    }

    // ------------------------------------------------------------------------
    //! \brief Get access to the robot's state (dynamic model).
    //! \return Reference to the state.
    // ------------------------------------------------------------------------
    inline State& states()
    {
        return m_states;
    }

    // ------------------------------------------------------------------------
    //! \brief Get access to the robot's state (dynamic model).
    //! \return Const reference to the state.
    // ------------------------------------------------------------------------
    inline State const& states() const
    {
        return m_states;
    }

    // ------------------------------------------------------------------------
    //! \brief Get access to the robot's blueprint (model).
    //! \return Reference to the blueprint.
    // ------------------------------------------------------------------------
    inline Blueprint& blueprint()
    {
        return m_blueprint;
    }

    // ------------------------------------------------------------------------
    //! \brief Get access to the robot's blueprint (model).
    //! \return Const reference to the blueprint.
    // ------------------------------------------------------------------------
    inline Blueprint const& blueprint() const
    {
        return m_blueprint;
    }

    // ------------------------------------------------------------------------
    //! \brief Set all joints to neutral position (0).
    // ------------------------------------------------------------------------
    void setNeutralPosition();

    // ------------------------------------------------------------------------
    //! \brief Set all joints to the given joint positions.
    //! \param p_joint_positions The joint positions to set.
    // ------------------------------------------------------------------------
    void setJointPositions(const JointValues& p_joint_positions);

    // ------------------------------------------------------------------------
    //! \brief Set the home position values (store without applying).
    //! \param p_home_position The home position joint values to store.
    // ------------------------------------------------------------------------
    void setHomePosition(const JointValues& p_home_position);

    // ------------------------------------------------------------------------
    //! \brief Apply the stored home position to the robot.
    //! If no home position is set, does nothing.
    // ------------------------------------------------------------------------
    void setHomePosition();

    // ------------------------------------------------------------------------
    //! \brief Apply target joint positions with velocity limits.
    //!
    //! This method smoothly moves joints toward target positions while
    //! respecting maximum velocity constraints. Each joint moves at most
    //! maxVelocity * dt per call.
    //!
    //! \param q_target Target joint positions vector.
    //! \param dt Time step for velocity-limited motion.
    // ------------------------------------------------------------------------
    void applyJointTargetsWithSpeedLimit(const JointValues& q_target,
                                         double dt);

    // ------------------------------------------------------------------------
    //! \brief Perform forward kinematics computation.
    //!
    //! Computes the world transformations of all links and joints based on
    //! the current joint positions in the state. This updates:
    //! - state.linkTransforms()
    //! - state.jointTransforms()
    //! - The world transforms in the blueprint's nodes
    //!
    //! ALGORITHM:
    //! Traverses the kinematic tree from root to leaves, accumulating
    //! transformations: T_world = T_parent * T_local
    //!
    //! \param p_state The state containing joint positions (input) and where
    //! transforms will be stored (output).
    // ------------------------------------------------------------------------
    void setJointPositions(State& p_state);

    // ------------------------------------------------------------------------
    //! \brief Perform forward kinematics computation on the robot's state.
    // ------------------------------------------------------------------------
    inline void forwardKinematics()
    {
        setJointPositions(m_states);
    }

    // ------------------------------------------------------------------------
    //! \brief Compute the Jacobian matrix for a given end effector.
    //!
    //! Computes the 6×n Jacobian matrix relating joint velocities to
    //! end-effector twist (linear and angular velocity).
    //!
    //! The Jacobian is computed based on:
    //! - Current joint and link transforms (from forward kinematics)
    //! - Joint types (revolute vs prismatic)
    //! - Joint axes
    //!
    //! The result is cached in the state. If forward kinematics has been
    //! called after the last Jacobian computation, it will be recomputed.
    //!
    //! \param p_state The state with current transforms and where Jacobian
    //! will be stored.
    //! \param p_end_effector The end effector node for which to compute
    //! Jacobian.
    //! \return Const reference to the computed Jacobian in the state.
    // ------------------------------------------------------------------------
    Jacobian const& computeJacobian(State& p_state, Node const& p_end_effector);

    // ------------------------------------------------------------------------
    //! \brief Create and add a new frames (coordinate frame) to the root of
    //! the blueprint.
    //!
    //! \param p_name The name of the frames.
    //! \param p_transform The local transformation of the frames (default:
    //! identity).
    //! \return Reference to the created frames.
    // ------------------------------------------------------------------------
    Frame& addFrame(std::string const& p_name,
                    Transform const& p_transform = Transform::Identity());

    // ------------------------------------------------------------------------
    //! \brief Create and add a new frames (coordinate frame) to a specific
    //! parent node.
    //!
    //! \param p_parent The parent node to attach the frames to.
    //! \param p_name The name of the frames.
    //! \param p_transform The local transformation of the frames (default:
    //! identity).
    //! \return Reference to the created frames.
    // ------------------------------------------------------------------------
    Frame& addFrameToNode(Node& p_parent,
                          std::string const& p_name,
                          Transform const& p_transform = Transform::Identity());

    // ------------------------------------------------------------------------
    //! \brief Attach the robot's root to a frames (coordinate frame).
    //!
    //! This allows positioning the robot in a different coordinate system
    //! (e.g., workshop frame). The current root becomes a child of the frames.
    //!
    //! \param p_frame The frames to attach the robot to.
    // ------------------------------------------------------------------------
    void attachToFrame(Frame& p_frame);

private:

    //! \brief The name of the robot.
    std::string m_name;
    //! \brief Reference to the robot's kinematic blueprint (static model)
    Blueprint m_blueprint;
    //! \brief State of the robot (dynamic model)
    State m_states;
    //! \brief Stored home position (joint values)
    JointValues m_home_position;
};

} // namespace robotik
