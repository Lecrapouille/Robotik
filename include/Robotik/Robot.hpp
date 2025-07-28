#pragma once

#include "Robotik/private/Joint.hpp"
#include "Robotik/private/Link.hpp"

#include <string>
#include <vector>

namespace robotik
{

// *********************************************************************************
//! \brief Class representing a complete robotic arm.
//!
//! This class encapsulates an entire robotic manipulator system, combining
//! the kinematic chain (joints and links) with high-level control and
//! analysis capabilities. It serves as the main interface for robot control
//! and simulation.
//!
//! Key components:
//! - Kinematic chain: Hierarchical structure of joints and links.
//! - Root node: Base frame of the robot (typically fixed to ground/table).
//! - End-effector: Final link where tools/grippers are attached.
//! - Joint collection: All actuated joints that define robot configuration.
//!
//! Main capabilities:
//!
//! 1. FORWARD KINEMATICS: Given joint angles → compute end-effector pose
//!    * Essential for: Path planning, collision avoidance, visualization
//!    * Formula: T_end = T_base * ∏(T_joint_i) from base to end-effector
//!
//! 2. INVERSE KINEMATICS: Given desired end-effector pose → compute joint
//! angles
//!    * Essential for: Task-space control, trajectory following
//!    * Challenging due to: Multiple solutions, singularities, joint limits
//!
//! 3. JACOBIAN COMPUTATION: Relationship between joint velocities and
//! end-effector velocity
//!    * Essential for: Velocity control, force control, singularity
//!    analysis
//!    * J = ∂pose/∂joints (6×n matrix for n joints)
//!
//! 4. CONFIGURATION MANAGEMENT: Set/get joint positions, enforce limits
//!    * Essential for: Robot control, safety, trajectory execution
//!
//! This class bridges the gap between low-level joint control and
//! high-level task planning, providing the mathematical foundation for
//! robot manipulation.
// *********************************************************************************
class Robot
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor. Perform no action, just set the name.
    //! \param p_name Name of the robot arm.
    // ------------------------------------------------------------------------
    explicit Robot(std::string_view const& p_name) : m_name(p_name) {}

    // ------------------------------------------------------------------------
    //! \brief Constructor. Perform no action, just set the name.
    //! \param p_name Name of the robot arm.
    //! \param p_end_effector Reference to the end effector node.
    // ------------------------------------------------------------------------
    Robot(std::string_view const& p_name, scene::Node::Ptr p_root);

    // ------------------------------------------------------------------------
    //! \brief Get the name of the robot arm.
    //! \return The name of the robot arm.
    // ------------------------------------------------------------------------
    inline std::string const& name() const
    {
        return m_name;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the robot arm has a root node.
    //! \return True if the robot arm has a root node, false otherwise.
    // ------------------------------------------------------------------------
    inline bool hasRoot() const
    {
        return m_root_node != nullptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Set and replace the root node of the robot arm.
    //! \param p_root Unique pointer to the root node.
    //! \return Reference to the root node.
    // ------------------------------------------------------------------------
    void root(scene::Node::Ptr p_root);

    // ------------------------------------------------------------------------
    //! \brief Get the root node of the robot arm.
    //! \note Call init() before calling this method, otherwise the root node
    //! is not set. Call hasRoot() to check if the root node is set.
    //! \return Reference to the root node.
    // ------------------------------------------------------------------------
    inline scene::Node const& root() const
    {
        return *m_root_node.get();
    }

    // ------------------------------------------------------------------------
    //! \brief Find and return a link by its name.
    //! \param p_name Name of the link.
    //! \return Pointer to the link, or nullptr if not found.
    // ------------------------------------------------------------------------
    // Link* link(std::string_view const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Find and return a joint by its name.
    //! \param p_name Name of the joint.
    //! \return Pointer to the joint, or nullptr if not found.
    // ------------------------------------------------------------------------
    Joint const& joint(std::string_view const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Get the joint values of the robot arm.
    //!
    //! PHYSICS: Returns the current configuration of all joints in the
    //! kinematic chain. This vector represents the robot's pose in joint
    //! space, where each element corresponds to a joint's generalized
    //! coordinate.
    //!
    //! For a serial manipulator with n joints:
    //! - Joint space: Q = [q₁, q₂, ..., qₙ]
    //! - Each qᵢ represents the i-th joint's angular position (revolute)
    //!   or linear position (prismatic)
    //! - The configuration space is typically n-dimensional
    //!
    //! This joint configuration uniquely determines:
    //! - End-effector position and orientation (via forward kinematics)
    //! - Robot's workspace accessibility
    //! - Potential singularities and joint limits
    //!
    //! \return Vector of joint configuration values in order
    // ------------------------------------------------------------------------
    // FIXME: retourner std::vector<Joint::Value>
    std::vector<double> jointValues() const;

    // ------------------------------------------------------------------------
    //! \brief Get the names of the actuable joints in the order they appear
    //! in the joint values vector.
    //!
    //! This method provides a mapping between joint indices and joint names,
    //! allowing you to know which joint corresponds to which index in the
    //! joint values vector returned by jointValues().
    //!
    //! \return Vector of joint names in the same order as joint values.
    // ------------------------------------------------------------------------
    std::vector<std::string> jointNames() const;

    // ------------------------------------------------------------------------
    //! \brief Set joint values for all actuable joints in order.
    //!
    //! This method provides a convenient way to set joint values using a simple
    //! vector of doubles, where each value corresponds to the joint in the same
    //! order as returned by jointNames().
    //!
    //! \param p_values Vector of joint values to set.
    //! \return True if successful, false if the number of values doesn't match
    //! the number of joints.
    // ------------------------------------------------------------------------
    bool setJointValues(std::vector<double> const& p_values);

    // ------------------------------------------------------------------------
    //! \brief Compute the inverse kinematics of the robot arm by the Jacobian
    //! method.
    //!
    //! PHYSICS: Inverse kinematics solves for the joint configuration that
    //! achieves a desired end-effector pose. This is mathematically complex
    //! as it involves solving a system of nonlinear equations.
    //!
    //! MATHEMATICAL FORMULATION:
    //! Given target pose P_target, find joint values Q such that:
    //! P_target = f(Q) where f is the forward kinematics function
    //!
    //! SOLUTION CHALLENGES:
    //! - Multiple solutions: A given pose may have several valid joint configs
    //! - No solutions: Target pose may be outside robot's workspace
    //! - Infinite solutions: At kinematic singularities
    //! - Numerical stability: Iterative methods may not converge
    //!
    //! COMMON ALGORITHMS:
    //! - Jacobian-based methods (Newton-Raphson, Levenberg-Marquardt)
    //! - Geometric methods (for specific robot geometries)
    //! - Optimization-based approaches
    //!
    //! WORKSPACE CONSTRAINTS:
    //! - Reachable workspace: All poses that can be achieved
    //! - Dexterous workspace: Poses achievable with arbitrary orientations
    //! - Joint limits, singularities, and obstacles affect solvability
    //!
    //! \param p_target_pose Desired 6D pose of the end-effector
    //! [x,y,z,rx,ry,rz].
    //! \param p_end_effector Reference to the end effector node.
    //! \param p_max_iterations Maximum number of iterations.
    //! \param p_epsilon Tolerance for convergence.
    //! \param p_damping Damping factor.
    //!
    //! \return Vector of joint values if a valid solution is found, empty
    //! vector otherwise.
    // ------------------------------------------------------------------------
    std::vector<double> inverseKinematics(Pose const& p_target_pose,
                                          scene::Node const& p_end_effector,
                                          size_t const p_max_iterations = 500,
                                          double const p_epsilon = 1e-4,
                                          double const p_damping = 0.01);

    // ------------------------------------------------------------------------
    //! \brief Compute the Jacobian matrix of the robot arm.
    //! \param p_end_effector Reference to the end effector node.
    //! \return The Jacobian matrix.
    // ------------------------------------------------------------------------
    Jacobian calculateJacobian(scene::Node const& p_end_effector) const;

protected:

    // ------------------------------------------------------------------------
    //! \brief Check if the robot arm is setup correctly.
    //! \throw std::runtime_error if the robot arm is not setup correctly.
    // ------------------------------------------------------------------------
    void checkRobotSetupValidity() const;

    // ------------------------------------------------------------------------
    //! \brief Cache the list of joints in the robot arm. Shall be called
    //! each time the scene graph is modified.
    //! \note This method is called automatically when needed, but can be
    //! called explicitly to force cache update.
    // ------------------------------------------------------------------------
    void cacheListOfJoints();

private:

    std::string m_name;
    scene::Node::Ptr m_root_node;
    std::vector<Joint*> m_joints;
    std::unordered_map<std::string, Joint*> m_joints_map;
};

} // namespace robotik
