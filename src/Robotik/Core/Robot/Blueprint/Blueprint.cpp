/**
 * @file Blueprint.cpp
 * @brief Implementation of Blueprint class.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/Blueprint/Blueprint.hpp"
#include "Robotik/Core/Common/Exception.hpp"
#include "Robotik/Core/Robot/Blueprint/NodeVisitor.hpp"

namespace robotik
{

// Forward declare for friend
class CacheBlueprintVisitor;

} // namespace robotik

namespace robotik
{

// ****************************************************************************
//! \brief Visitor for caching the blueprint tree structure.
//!
//! This visitor traverses the robot blueprint and populates the internal
//! lookup tables for joints, links, sensors, and actuators. It also assigns
//! indices to actuable joints for efficient array-based access.
// ****************************************************************************
class CacheBlueprintVisitor: public NodeVisitor
{
private:

    Blueprint* m_blueprint;
    size_t m_joint_index = 0;

public:

    explicit CacheBlueprintVisitor(Blueprint* p_blueprint)
        : m_blueprint(p_blueprint)
    {
    }

    void visit(Joint& joint) override
    {
        // Only cache actuable joints (exclude FIXED joints)
        if (joint.type() != Joint::Type::FIXED)
        {
            m_blueprint->m_joints_map.try_emplace(joint.name(),
                                                  std::ref(joint));
            m_blueprint->m_joints.emplace_back(std::ref(joint));
            joint.setIndex(m_joint_index++);
        }
    }

    void visit(Link& link) override
    {
        m_blueprint->m_links_map.try_emplace(link.name(), std::ref(link));
    }

    void visit(Sensor& sensor) override
    {
        m_blueprint->m_sensors_map.try_emplace(sensor.name(), std::ref(sensor));
    }

    void visit(Actuator& actuator) override
    {
        m_blueprint->m_actuators_map.try_emplace(actuator.name(),
                                                 std::ref(actuator));
    }

    void visit(Geometry& geometry) override
    {
        m_blueprint->m_geometries.emplace_back(std::ref(geometry));
    }

    void visit(Node&) override
    {
        // Ignore other node types
    }
};

// ----------------------------------------------------------------------------
Blueprint::Blueprint(Node::Ptr p_root) : m_root_node(std::move(p_root))
{
    cacheBlueprintTree();
}

// ----------------------------------------------------------------------------
void Blueprint::root(Node::Ptr p_root)
{
    m_root_node = std::move(p_root);
    cacheBlueprintTree();
}

// ----------------------------------------------------------------------------
void Blueprint::cacheBlueprintTree()
{
    m_joints.clear();
    m_joints_map.clear();
    m_links_map.clear();
    m_sensors_map.clear();
    m_actuators_map.clear();
    m_end_effectors.clear();
    m_geometries.clear();

    if (m_root_node == nullptr)
        return;

    // Use the Visitor pattern to traverse and cache the blueprint
    CacheBlueprintVisitor visitor(this);
    m_root_node->traverse(visitor);

    // To be called after all joints are cached
    findEndEffectors(m_end_effectors);
}

// ----------------------------------------------------------------------------
Joint const& Blueprint::joint(std::string const& p_name) const
{
    auto const& it = m_joints_map.find(std::string(p_name));
    if (it == m_joints_map.end())
    {
        throw RobotikException("Joint not found: " + std::string(p_name));
    }
    return it->second.get();
}

// ----------------------------------------------------------------------------
Joint& Blueprint::joint(std::string const& p_name)
{
    auto const& it = m_joints_map.find(std::string(p_name));
    if (it == m_joints_map.end())
    {
        throw RobotikException("Joint not found: " + std::string(p_name));
    }
    return it->second.get();
}

// ----------------------------------------------------------------------------
Link const& Blueprint::link(std::string const& p_name) const
{
    auto const& it = m_links_map.find(std::string(p_name));
    if (it == m_links_map.end())
    {
        throw RobotikException("Link not found: " + std::string(p_name));
    }
    return it->second.get();
}

// ----------------------------------------------------------------------------
size_t Blueprint::findEndEffectors(
    std::vector<std::reference_wrapper<Link>>& p_end_effectors)
{
    p_end_effectors.clear();
    p_end_effectors.reserve(m_links_map.size());

    for (auto& [_, link_ref] : m_links_map)
    {
        Link& link = link_ref.get();
        bool has_child_joints = false;
        for (auto& child : link.children())
        {
            if (dynamic_cast<Joint*>(child.get()))
            {
                has_child_joints = true;
                break;
            }
        }

        if (!has_child_joints)
        {
            p_end_effectors.emplace_back(std::ref(link));
        }
    }

    return p_end_effectors.size();
}

} // namespace robotik
