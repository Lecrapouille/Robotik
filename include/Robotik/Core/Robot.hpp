/**
 * @file Robot.hpp
 * @brief Robot class - Representation of a complete robotic system.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Component.hpp"
#include "Robotik/Core/Joint.hpp"
#include "Robotik/Core/Link.hpp"

namespace robotik
{

class Robot;

namespace debug
{

// ****************************************************************************
//! \brief Print the robot as scene graph to a string.
//! \param p_robot The robot to print.
//! \param p_detailed If false, show only the tree structure (default).
//!  If true, show detailed information (local and world transforms, geometry).
//! \return A string containing the formatted robot representation.
// ****************************************************************************
std::string printRobot(Robot const& p_robot, bool p_detailed = false);

} // namespace debug

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
    //! \param p_name Name of the robot.
    // ------------------------------------------------------------------------
    explicit Robot(std::string_view const& p_name) : m_name(p_name) {}

    // ------------------------------------------------------------------------
    //! \brief Constructor. Perform no action, just set the name.
    //! \param p_name Name of the robot.
    //! \param p_end_effector Reference to the end effector node.
    // ------------------------------------------------------------------------
    Robot(std::string_view const& p_name, scene::Node::Ptr p_root);

    // ------------------------------------------------------------------------
    //! \brief Get the name of the robot.
    //! \return The name of the robot.
    // ------------------------------------------------------------------------
    inline std::string const& name() const
    {
        return m_name;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the robot has a root node.
    //! \return True if the robot has a root node, false otherwise.
    // ------------------------------------------------------------------------
    inline bool hasRoot() const
    {
        return m_root_node != nullptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Set and replace the root node of the robot.
    //! \param p_root Unique pointer to the root node.
    //! \return Reference to the root node.
    // ------------------------------------------------------------------------
    void root(scene::Node::Ptr p_root); // FIXME mettre un Link

    // ------------------------------------------------------------------------
    //! \brief Get the root node of the robot.
    //! \note Call init() before calling this method, otherwise the root node
    //! is not set. Call hasRoot() to check if the root node is set.
    //! \return Reference to the root node.
    // ------------------------------------------------------------------------
    inline scene::Node const& root() const // FIXME retourne un Link
    {
        return *m_root_node.get();
    }

    // ------------------------------------------------------------------------
    //! \brief Set a prefix path for each mesh geometry of the robot.
    //!
    //! For example, if the URDF has:
    //! <geometry><mesh filename="Body.STL"/></geometry>
    //! then the mesh path will be set to p_prefix_path + "/Body.STL".
    //! \param p_prefix The prefix to add to the mesh path.
    // ------------------------------------------------------------------------
    void setMeshPrefixPath(std::string const& p_prefix) const;

    // ------------------------------------------------------------------------
    //! \brief Find and return a link by its name.
    //! \param p_name Name of the link.
    //! \return Pointer to the link, or nullptr if not found.
    // ------------------------------------------------------------------------
    Link const& link(std::string_view const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Find and return a joint by its name.
    //! \param p_name Name of the joint.
    //! \return Reference to the joint.
    //! \throw RobotikException if the joint is not found.
    // ------------------------------------------------------------------------
    Joint const& joint(std::string_view const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Find all end effectors of the robot.
    //!
    //! This method automatically identifies all end effectors by finding links
    //! that have no child joints and whose children are links. An end effector
    //! is typically the last joint in a kinematic chain where tools or grippers
    //! are attached.
    //!
    //! \param p_end_effectors Output vector to store references to end effector
    //! links
    //! \return Number of end effectors found
    //! \note Call hasRoot() before calling this method to ensure the robot
    //! is properly initialized and the scene graph cache is up to date
    // ------------------------------------------------------------------------
    size_t findEndEffectors(
        std::vector<std::reference_wrapper<Link const>>& p_end_effectors) const;

    // ------------------------------------------------------------------------
    //! \brief Get the joint values of the robot.
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
    std::vector<double> jointPositions() const;

    // ------------------------------------------------------------------------
    //! \brief Get the names of the actuable joints in the order they appear
    //! in the joint values vector.
    //!
    //! This method provides a mapping between joint indices and joint names,
    //! allowing you to know which joint corresponds to which index in the
    //! joint values vector returned by jointPositions().
    //!
    //! \return Vector of joint names in the same order as joint values.
    // ------------------------------------------------------------------------
    std::vector<std::string> jointNames() const;

    // ------------------------------------------------------------------------
    //! \brief Get the names of the links in the robot.
    //! \return Vector of link names.
    // ------------------------------------------------------------------------
    std::vector<std::string> linkNames() const;

    // ------------------------------------------------------------------------
    //! \brief Get the names of the sensors in the robot.
    //! \return Vector of sensor names.
    // ------------------------------------------------------------------------
    std::vector<std::string> sensorNames() const;

    // ------------------------------------------------------------------------
    //! \brief Get the names of the actuators in the robot.
    //! \return Vector of actuator names.
    // ------------------------------------------------------------------------
    std::vector<std::string> actuatorNames() const;

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
    //! \brief Set the robot to its neutral position.
    //!
    //! This method sets all joint values to 0.
    // ------------------------------------------------------------------------
    void setNeutralPosition();

    // ------------------------------------------------------------------------
    //! \brief Compute the Jacobian matrix of the robot.
    //! \param p_end_effector Reference to the end effector node.
    //! \return The Jacobian matrix.
    // ------------------------------------------------------------------------
    Jacobian
    jacobian(scene::Node const& p_end_effector) const; // FIXME a deplacer

protected:

    // ------------------------------------------------------------------------
    //! \brief Check if the robot is setup correctly.
    //! \throw std::runtime_error if the robot is not setup correctly.
    // ------------------------------------------------------------------------
    void checkRobotSetupValidity() const;

    // ------------------------------------------------------------------------
    //! \brief Cache the tree of the robot.
    //! This is now called automatically when needed (lazy evaluation).
    //! \note This method is called automatically when needed, but can be
    //! called explicitly to force cache update.
    // ------------------------------------------------------------------------
    void cacheSceneGraph();

    // ------------------------------------------------------------------------
    //! \brief Mark the joints cache as dirty (needs refresh).
    //! Called automatically when scene graph is modified.
    // ------------------------------------------------------------------------
    void markSceneGraphCacheDirty()
    {
        m_scene_graph_cache_dirty = true;
    }

    // ------------------------------------------------------------------------
    //! \brief Calculate proper pose error between target and current pose.
    //! Handles orientation error correctly using rotation matrices.
    // ------------------------------------------------------------------------
    Pose calculatePoseError(Pose const& p_target_pose,
                            Pose const& p_current_pose) const;

private:

    //! \brief Name of the robot.
    std::string m_name;
    //! \brief Root node of the scene graph representing the robot.
    scene::Node::Ptr m_root_node;
    //! \brief List of joints in the robot from the scene graph.
    std::vector<std::reference_wrapper<Joint>> m_joints;
    //! \brief Map of joints in the robot from the scene graph.
    std::unordered_map<std::string, std::reference_wrapper<Joint>> m_joints_map;
    //! \brief Map of links from the scene graph.
    std::unordered_map<std::string, std::reference_wrapper<Link>> m_links_map;
    //! \brief Map of sensors in the robot from the scene graph.
    std::unordered_map<std::string, std::reference_wrapper<Sensor>>
        m_sensors_map;
    //! \brief Map of actuators in the robot from the scene graph.
    std::unordered_map<std::string, std::reference_wrapper<Actuator>>
        m_actuators_map;
    //! \brief Flag to indicate if the scene graph cache is dirty.
    bool m_scene_graph_cache_dirty = true;
};

} // namespace robotik
