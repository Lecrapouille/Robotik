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

// Forward declaration for friend
class CacheHierarchyVisitor;

// *********************************************************************************
//! \brief Class managing a robot's tree hierarchy as static robot model.
//!
//! This class encapsulates the structural information for describing a robot:
//! the topology of joints and links, their geometric relationships, and the
//! hierarchical tree structure. It represents the Pinocchio's Model separation
//! pattern (Model/Data) where Data is the structure holding robot's states.
//!
//! Key components:
//! - Kinematic tree: Hierarchical structure of joints and links.
//! - Store other objects: sensors, actuators, geometries, ...
//! - Root node: Base link of the robot (i.e. fixed to ground/table).
//! - Joint/Link caches: Fast lookup tables for components by name.
//! - Sensor/Actuator registries: access to components attached to the
//! robot tree.
//!
//! For dynamic computations, use the Robot class which takes a Hierarchy
//! and operates on State objects.
// *********************************************************************************
class Hierarchy
{
    // Friend declaration for the visitor that caches hierarchy
    friend class CacheHierarchyVisitor;

public:

    // ------------------------------------------------------------------------
    //! \brief Constructor with root node.
    //! \param p_root Root node of the kinematic tree.
    // ------------------------------------------------------------------------
    explicit Hierarchy(Node::Ptr p_root);

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
    void root(Node::Ptr p_root); // FIXME: Link

    // ------------------------------------------------------------------------
    //! \brief Get the root node of the hierarchy.
    //! \note Call hasRoot() before calling this method to ensure the root
    //! node is set.
    //! \return Reference to the root node.
    // ------------------------------------------------------------------------
    inline Node const& root() const // FIXME: Link
    {
        return *m_root_node.get();
    }

    // ------------------------------------------------------------------------
    //! \brief Find and return a link by its name.
    //! \param p_name Name of the link.
    //! \return Const reference to the link.
    //! \throw RobotikException if the link is not found.
    // ------------------------------------------------------------------------
    Link const&
    link(std::string const& p_name) const; // FIXME attacher une std::fucntion

    // ------------------------------------------------------------------------
    //! \brief Find and return a joint by its name.
    //! \param p_name Name of the joint.
    //! \return Const reference to the joint.
    //! \throw RobotikException if the joint is not found.
    // ------------------------------------------------------------------------
    Joint const& joint(std::string const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Get mutable access to a joint by name.
    //! \param p_name Name of the joint.
    //! \return Reference to the joint.
    //! \throw RobotikException if the joint is not found.
    // ------------------------------------------------------------------------
    Joint& joint(std::string const& p_name);

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
    //! \brief Get the number of end effectors.
    //! \return Number of end effectors.
    // ------------------------------------------------------------------------
    inline std::vector<std::reference_wrapper<Link>> const& endEffectors() const
    {
        return m_end_effectors;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the number of end effectors.
    //! \return Number of end effectors.
    // ------------------------------------------------------------------------
    inline std::vector<std::reference_wrapper<Link>>& endEffectors()
    {
        return m_end_effectors;
    }

    // ------------------------------------------------------------------------
    //! \brief Iterate over all joints.
    //! \param p_callback Callback function to call for each joint.
    // ------------------------------------------------------------------------
    template <typename Callback>
    void forEachJoint(Callback&& p_callback) const
    {
        for (auto const& joint : m_joints)
        {
            p_callback(joint.get(), joint.get().index());
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Iterate over all joints.
    //! \param p_callback Callback function to call for each joint.
    // ------------------------------------------------------------------------
    template <typename Callback>
    void forEachJoint(Callback&& p_callback)
    {
        for (auto const& joint : m_joints)
        {
            p_callback(joint.get(), joint.get().index());
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Iterate over all links.
    //! \param p_callback Callback function to call for each link.
    // ------------------------------------------------------------------------
    template <typename Callback>
    void forEachLink(Callback&& p_callback) const
    {
        for (auto const& [_, link] : m_links_map)
        {
            p_callback(link.get());
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Iterate over all links.
    //! \param p_callback Callback function to call for each link.
    // ------------------------------------------------------------------------
    template <typename Callback>
    void forEachLink(Callback&& p_callback)
    {
        for (auto const& [_, link] : m_links_map)
        {
            p_callback(link.get());
        }
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Cache the tree structure of the hierarchy.
    //!
    //! This method traverses the kinematic tree and populates internal lookup
    //! tables for efficient access to joints, links, sensors, and actuators.
    //! Called automatically when the root is set or modified.
    // ------------------------------------------------------------------------
    void cacheHierarchyTree();

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
        std::vector<std::reference_wrapper<Link>>& p_end_effectors);

private:

    //! \brief Root node of the kinematic hierarchy
    Node::Ptr m_root_node;
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
    //! \brief List of end effectors in the hierarchy
    std::vector<std::reference_wrapper<Link>> m_end_effectors;
};

} // namespace robotik
