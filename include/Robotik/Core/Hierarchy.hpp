/**
 * @file Hierarchy.hpp
 * @brief Hierarchy class - Representation of a robot's kinematic structure
 * (model).
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

class Hierarchy;

namespace debug
{

// ****************************************************************************
//! \brief Print the robot hierarchy as a tree to a string.
//! \param p_hierarchy The hierarchy to print.
//! \param p_detailed If false, show only the tree structure (default).
//!  If true, show detailed information (local and world transforms, geometry).
//! \return A string containing the formatted hierarchy representation.
// ****************************************************************************
std::string printRobot(Hierarchy const& p_hierarchy, bool p_detailed = false);

// Alias for backward compatibility
inline std::string printHierarchy(Hierarchy const& p_hierarchy,
                                  bool p_detailed = false)
{
    return printRobot(p_hierarchy, p_detailed);
}

} // namespace debug

// *********************************************************************************
//! \brief Class representing a robot's kinematic structure (static model).
//!
//! This class encapsulates the structural information of a robotic manipulator:
//! the topology of joints and links, their geometric relationships, and the
//! hierarchical tree structure. It represents the "Model" in Pinocchio's
//! Model/Data separation pattern.
//!
//! Key components:
//! - Kinematic tree: Hierarchical structure of joints and links
//! - Root node: Base frame of the robot (typically fixed to ground/table)
//! - Joint/Link caches: Fast lookup tables for components by name
//! - Sensor/Actuator registries: Additional components attached to the
//! structure
//!
//! What Hierarchy contains (static model):
//! - Topology: Which joints connect which links
//! - Geometry: Link dimensions, visual meshes, collision shapes
//! - Inertial properties: Mass, center of mass, inertia tensors
//! - Joint types and axes: Revolute, prismatic, fixed joints
//! - Motion limits: Joint position, velocity, and effort limits
//!
//! What Hierarchy does NOT contain (dynamic state):
//! - Current joint positions, velocities, accelerations
//! - Current transformations of links and joints
//! - Jacobian matrices
//! - Forward/inverse kinematics results
//!
//! For dynamic computations, use the Robot class which takes a Hierarchy
//! and operates on State objects.
//!
//! This separation allows:
//! - Multiple state instances for the same robot model
//! - Parallel computations on different configurations
//! - Efficient memory usage when working with robot fleets
//! - Clear separation between model and state
// *********************************************************************************
class Hierarchy
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor. Creates an empty hierarchy with a name.
    //! \param p_name Name of the robot/hierarchy.
    // ------------------------------------------------------------------------
    explicit Hierarchy(std::string_view const& p_name) : m_name(p_name) {}

    // ------------------------------------------------------------------------
    //! \brief Constructor with root node.
    //! \param p_name Name of the robot/hierarchy.
    //! \param p_root Root node of the kinematic tree.
    // ------------------------------------------------------------------------
    Hierarchy(std::string_view const& p_name, hierarchy::Node::Ptr p_root);

    // ------------------------------------------------------------------------
    //! \brief Get the name of the hierarchy.
    //! \return The name of the hierarchy.
    // ------------------------------------------------------------------------
    inline std::string const& name() const
    {
        return m_name;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the hierarchy has a root node.
    //! \return True if the hierarchy has a root node, false otherwise.
    // ------------------------------------------------------------------------
    inline bool hasRoot() const
    {
        return m_root_node != nullptr;
    }

    // ------------------------------------------------------------------------
    //! \brief Set and replace the root node of the hierarchy.
    //! \param p_root Unique pointer to the root node.
    // ------------------------------------------------------------------------
    void root(hierarchy::Node::Ptr p_root);

    // ------------------------------------------------------------------------
    //! \brief Get the root node of the hierarchy.
    //! \note Call hasRoot() before calling this method to ensure the root
    //! node is set.
    //! \return Reference to the root node.
    // ------------------------------------------------------------------------
    inline hierarchy::Node const& root() const
    {
        return *m_root_node.get();
    }

    // ------------------------------------------------------------------------
    //! \brief Set a prefix path for each mesh geometry in the hierarchy.
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
    //! \return Const reference to the link.
    //! \throw RobotikException if the link is not found.
    // ------------------------------------------------------------------------
    Link const& link(std::string_view const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Find and return a joint by its name.
    //! \param p_name Name of the joint.
    //! \return Const reference to the joint.
    //! \throw RobotikException if the joint is not found.
    // ------------------------------------------------------------------------
    Joint const& joint(std::string_view const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Get mutable access to a joint by name.
    //! \param p_name Name of the joint.
    //! \return Reference to the joint.
    //! \throw RobotikException if the joint is not found.
    // ------------------------------------------------------------------------
    Joint& joint(std::string_view const& p_name);

    // ------------------------------------------------------------------------
    //! \brief Find all end effectors in the hierarchy.
    //!
    //! This method automatically identifies all end effectors by finding links
    //! that have no child joints. An end effector is typically the last link
    //! in a kinematic chain where tools or grippers are attached.
    //!
    //! \param p_end_effectors Output vector to store references to end effector
    //! links.
    //! \return Number of end effectors found.
    // ------------------------------------------------------------------------
    size_t findEndEffectors(
        std::vector<std::reference_wrapper<Link const>>& p_end_effectors) const;

    // ------------------------------------------------------------------------
    //! \brief Get the names of the actuable joints in order.
    //!
    //! This method provides a mapping between joint indices and joint names,
    //! useful for understanding the order of joints in state vectors.
    //!
    //! \return Vector of joint names in the same order as in internal storage.
    // ------------------------------------------------------------------------
    std::vector<std::string> jointNames() const;

    // ------------------------------------------------------------------------
    //! \brief Get the names of the links in the hierarchy.
    //! \return Vector of link names.
    // ------------------------------------------------------------------------
    std::vector<std::string> linkNames() const;

    // ------------------------------------------------------------------------
    //! \brief Get the names of the sensors in the hierarchy.
    //! \return Vector of sensor names.
    // ------------------------------------------------------------------------
    std::vector<std::string> sensorNames() const;

    // ------------------------------------------------------------------------
    //! \brief Get the names of the actuators in the hierarchy.
    //! \return Vector of actuator names.
    // ------------------------------------------------------------------------
    std::vector<std::string> actuatorNames() const;

    // ------------------------------------------------------------------------
    //! \brief Get the number of actuable joints.
    //! \return Number of joints (excluding fixed joints).
    // ------------------------------------------------------------------------
    inline size_t numJoints() const
    {
        return m_joints.size();
    }

    // ------------------------------------------------------------------------
    //! \brief Get the number of links.
    //! \return Number of links.
    // ------------------------------------------------------------------------
    inline size_t numLinks() const
    {
        return m_links_map.size();
    }

    // ------------------------------------------------------------------------
    //! \brief Get const access to all joints.
    //! \return Vector of const references to joints.
    // ------------------------------------------------------------------------
    inline std::vector<std::reference_wrapper<Joint>> const& joints() const
    {
        return m_joints;
    }

    // ------------------------------------------------------------------------
    //! \brief Get mutable access to all joints.
    //! \return Vector of references to joints.
    // ------------------------------------------------------------------------
    inline std::vector<std::reference_wrapper<Joint>>& joints()
    {
        return m_joints;
    }

protected:

    // ------------------------------------------------------------------------
    //! \brief Cache the tree structure of the hierarchy.
    //!
    //! This method traverses the kinematic tree and populates internal lookup
    //! tables for efficient access to joints, links, sensors, and actuators.
    //! Called automatically when the root is set or modified.
    // ------------------------------------------------------------------------
    void cacheHierarchyTree();

    // ------------------------------------------------------------------------
    //! \brief Mark the hierarchy cache as dirty (needs refresh).
    //! Called automatically when the tree structure is modified.
    // ------------------------------------------------------------------------
    void markCacheDirty()
    {
        m_cache_dirty = true;
    }

private:

    //! \brief Name of the robot/hierarchy
    std::string m_name;

    //! \brief Root node of the kinematic hierarchy
    hierarchy::Node::Ptr m_root_node;

    //! \brief List of actuable joints in the hierarchy
    std::vector<std::reference_wrapper<Joint>> m_joints;

    //! \brief Map of joints by name for fast lookup
    std::unordered_map<std::string, std::reference_wrapper<Joint>> m_joints_map;

    //! \brief Map of links by name for fast lookup
    std::unordered_map<std::string, std::reference_wrapper<Link>> m_links_map;

    //! \brief Map of sensors by name for fast lookup
    std::unordered_map<std::string, std::reference_wrapper<Sensor>>
        m_sensors_map;

    //! \brief Map of actuators by name for fast lookup
    std::unordered_map<std::string, std::reference_wrapper<Actuator>>
        m_actuators_map;

    //! \brief Flag to indicate if the hierarchy cache is dirty
    bool m_cache_dirty = true;
};

} // namespace robotik
