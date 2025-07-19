#include "Robotik/Node.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
Node::Node(const std::string_view& p_name) : m_name(p_name)
{
    m_local_transform = Transform::Identity();
    m_world_transform = Transform::Identity();
}

// ----------------------------------------------------------------------------
Node const* Node::child(const std::string_view& p_name) const
{
    auto it = std::find_if(m_children.begin(),
                           m_children.end(),
                           [&p_name](const std::unique_ptr<Node>& p_node)
                           { return p_node->name() == p_name; });

    if (it != m_children.end())
    {
        return it->get();
    }

    return nullptr;
}

// ----------------------------------------------------------------------------
Node* Node::find(Node& p_root, const std::string_view& p_name)
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
void Node::localTransform(const Transform& p_transform)
{
    m_local_transform = p_transform;

    // Force update of the world transform of the node and its children
    update();
}

// ----------------------------------------------------------------------------
void Node::update()
{
    if (m_parent)
    {
        m_world_transform = m_parent->worldTransform() * m_local_transform;
    }
    else
    {
        m_world_transform = m_local_transform;
    }

    for (auto const& child : m_children)
    {
        child->update();
    }
}

} // namespace robotik