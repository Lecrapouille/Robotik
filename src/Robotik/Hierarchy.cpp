/**
 * @file Hierarchy.cpp
 * @brief Implementation of Hierarchy class.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Hierarchy.hpp"
#include "Robotik/Core/Exception.hpp"

#include <algorithm>
#include <functional>

namespace robotik
{

// ----------------------------------------------------------------------------
Hierarchy::Hierarchy(std::string_view const& p_name,
                     hierarchy::Node::Ptr p_root)
    : m_name(p_name)
{
    root(std::move(p_root));
}

// ----------------------------------------------------------------------------
void Hierarchy::root(hierarchy::Node::Ptr p_root)
{
    m_root_node = std::move(p_root);
    cacheHierarchyTree();
}

// ----------------------------------------------------------------------------
void Hierarchy::cacheHierarchyTree()
{
    // If the cache is not dirty, we can return early
    if (!m_cache_dirty)
    {
        return;
    }

    m_joints.clear();
    m_joints_map.clear();
    m_links_map.clear();
    m_sensors_map.clear();
    m_actuators_map.clear();

    if (m_root_node != nullptr)
    {
        m_root_node->traverse(
            [this](hierarchy::Node& p_node, size_t /*p_depth*/)
            {
                // Only cache actuable joints
                if (auto joint = dynamic_cast<Joint*>(&p_node))
                {
                    if (joint->type() != Joint::Type::FIXED)
                    {
                        m_joints_map.try_emplace(joint->name(),
                                                 std::ref(*joint));
                        m_joints.emplace_back(std::ref(*joint));
                    }
                }
                // Cache links
                else if (auto link = dynamic_cast<Link*>(&p_node))
                {
                    m_links_map.try_emplace(link->name(), std::ref(*link));
                }
                // Cache sensors
                else if (auto sensor = dynamic_cast<Sensor*>(&p_node))
                {
                    m_sensors_map.try_emplace(sensor->name(),
                                              std::ref(*sensor));
                }
                // Cache actuators
                else if (auto actuator = dynamic_cast<Actuator*>(&p_node))
                {
                    m_actuators_map.try_emplace(actuator->name(),
                                                std::ref(*actuator));
                }
            });
    }

    m_cache_dirty = false;
}

// ----------------------------------------------------------------------------
Joint const& Hierarchy::joint(std::string_view const& p_name) const
{
    auto const& it = m_joints_map.find(std::string(p_name));
    if (it == m_joints_map.end())
    {
        throw RobotikException("Joint not found: " + std::string(p_name));
    }
    return it->second.get();
}

// ----------------------------------------------------------------------------
Joint& Hierarchy::joint(std::string_view const& p_name)
{
    auto const& it = m_joints_map.find(std::string(p_name));
    if (it == m_joints_map.end())
    {
        throw RobotikException("Joint not found: " + std::string(p_name));
    }
    return it->second.get();
}

// ----------------------------------------------------------------------------
Link const& Hierarchy::link(std::string_view const& p_name) const
{
    auto const& it = m_links_map.find(std::string(p_name));
    if (it == m_links_map.end())
    {
        throw RobotikException("Link not found: " + std::string(p_name));
    }
    return it->second.get();
}

// ----------------------------------------------------------------------------
std::vector<std::string> Hierarchy::jointNames() const
{
    std::vector<std::string> names;
    names.reserve(m_joints.size());

    for (const auto& joint_ref : m_joints)
    {
        names.push_back(joint_ref.get().name());
    }

    return names;
}

// ----------------------------------------------------------------------------
std::vector<std::string> Hierarchy::linkNames() const
{
    std::vector<std::string> names;
    names.reserve(m_links_map.size());
    for (const auto& [name, _] : m_links_map)
    {
        names.push_back(name);
    }
    return names;
}

// ----------------------------------------------------------------------------
std::vector<std::string> Hierarchy::sensorNames() const
{
    std::vector<std::string> names;
    names.reserve(m_sensors_map.size());
    for (const auto& [name, _] : m_sensors_map)
    {
        names.push_back(name);
    }
    return names;
}

// ----------------------------------------------------------------------------
std::vector<std::string> Hierarchy::actuatorNames() const
{
    std::vector<std::string> names;
    names.reserve(m_actuators_map.size());
    for (const auto& [name, _] : m_actuators_map)
    {
        names.push_back(name);
    }
    return names;
}

// ----------------------------------------------------------------------------
size_t Hierarchy::findEndEffectors(
    std::vector<std::reference_wrapper<Link const>>& p_end_effectors) const
{
    p_end_effectors.clear();
    p_end_effectors.reserve(m_links_map.size());

    for (const auto& [_, link_ref] : m_links_map)
    {
        const Link& link = link_ref.get();
        bool has_child_joints = false;
        for (const auto& child : link.children())
        {
            if (dynamic_cast<Joint const*>(child.get()))
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

// ----------------------------------------------------------------------------
void Hierarchy::setMeshPrefixPath(std::string const& p_prefix_path) const
{
    if (p_prefix_path.empty())
    {
        return;
    }

    for (auto const& [_, link] : m_links_map)
    {
        auto& geometry = link.get().geometry();

        if (geometry.type() == Geometry::Type::MESH)
        {
            geometry.meshPath(p_prefix_path + "/" + geometry.meshPath());
        }
    }
}

} // namespace robotik
