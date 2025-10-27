/**
 * @file Node.cpp
 * @brief Scene graph node class - Representation of a node in the scene graph.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Robot/Blueprint/Node.hpp"
#include "Robotik/Robot/Blueprint/NodeVisitor.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
Node::Node(std::string const& p_name) : m_name(p_name)
{
    m_local_transform = Transform::Identity();
    m_world_transform = Transform::Identity();
}

// ----------------------------------------------------------------------------
Node const* Node::child(std::string const& p_name) const
{
    auto it = std::find_if(m_children.begin(),
                           m_children.end(),
                           [&p_name](std::unique_ptr<Node> const& p_node)
                           { return p_node->name() == p_name; });

    if (it != m_children.end())
    {
        return it->get();
    }

    return nullptr;
}

// ----------------------------------------------------------------------------
Node* Node::child(std::string const& p_name)
{
    auto it = std::find_if(m_children.begin(),
                           m_children.end(),
                           [&p_name](std::unique_ptr<Node> const& p_node)
                           { return p_node->name() == p_name; });

    if (it != m_children.end())
    {
        return it->get();
    }

    return nullptr;
}

// ----------------------------------------------------------------------------
Node* Node::find(Node& p_root, std::string const& p_name)
{
    // Check if the root node itself has the name
    if (p_root.name() == p_name)
    {
        return &p_root;
    }

    // Check if any direct child has the name
    if (Node* result = p_root.child(p_name); result != nullptr)
    {
        return result;
    }

    // If not found in direct children, search recursively
    for (const auto& child : p_root.m_children)
    {
        if (Node* result = find(*child, p_name); result != nullptr)
        {
            return result;
        }
    }

    return nullptr;
}

// ----------------------------------------------------------------------------
Node const* Node::find(Node const& p_root, std::string const& p_name)
{
    // Check if the root node itself has the name
    if (p_root.name() == p_name)
    {
        return &p_root;
    }

    // Check if any direct child has the name
    if (Node const* result = p_root.child(p_name); result != nullptr)
    {
        return result;
    }

    // If not found in direct children, search recursively
    for (const auto& child : p_root.m_children)
    {
        if (Node const* result = find(*child, p_name); result != nullptr)
        {
            return result;
        }
    }

    return nullptr;
}

// ----------------------------------------------------------------------------
void Node::localTransform(Transform const& p_transform)
{
    m_local_transform = p_transform;
    updateWorldTransforms(); // FIXME find a lazy way to do this
}

// ----------------------------------------------------------------------------
void Node::updateWorldTransforms()
{
    if (m_parent)
    {
        m_world_transform = m_parent->worldTransform() * localTransform();
    }
    else
    {
        m_world_transform = localTransform();
    }

    for (auto const& child : m_children)
    {
        child->updateWorldTransforms();
    }
}

// ----------------------------------------------------------------------------
void Node::accept(NodeVisitor& visitor)
{
    visitor.visit(*this);
}

// ----------------------------------------------------------------------------
void Node::accept(ConstNodeVisitor& visitor) const
{
    visitor.visit(*this);
}

// ----------------------------------------------------------------------------
void Node::traverse(NodeVisitor& visitor)
{
    accept(visitor);
    visitor.incrementDepth();
    for (auto& child : m_children)
    {
        child->traverse(visitor);
    }
    visitor.decrementDepth();
}

// ----------------------------------------------------------------------------
void Node::traverse(ConstNodeVisitor& visitor) const
{
    accept(visitor);
    visitor.incrementDepth();
    for (auto const& child : m_children)
    {
        child->traverse(visitor);
    }
    visitor.decrementDepth();
}

} // namespace robotik