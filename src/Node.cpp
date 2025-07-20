#include "Robotik/private/Node.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
Node::Node(std::string_view const& p_name) : m_name(p_name)
{
    m_local_transform = Transform::Identity();
    m_world_transform = Transform::Identity();
    m_joint_transform = Transform::Identity();
}

// ----------------------------------------------------------------------------
Node const* Node::child(std::string_view const& p_name) const
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
Node* Node::find(Node& p_root, std::string_view const& p_name)
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
Node const* Node::find(Node const& p_root, std::string_view const& p_name)
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
void Node::localTransform(const Eigen::Vector3d& p_origin_xyz,
                          const Eigen::Vector3d& p_origin_rpy)
{
    m_local_transform = Eigen::Matrix4d::Identity();

    // Apply translation
    m_local_transform.block<3, 1>(0, 3) = p_origin_xyz;

    // Apply RPY rotation
    Eigen::Matrix3d rotation =
        (Eigen::AngleAxisd(p_origin_rpy.z(), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(p_origin_rpy.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(p_origin_rpy.x(), Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    m_local_transform.block<3, 3>(0, 0) = rotation;

    updateWorldTransforms();
}

// ----------------------------------------------------------------------------
void Node::localTransform(Transform const& p_transform)
{
    m_local_transform = p_transform;
    updateWorldTransforms();
}

// ----------------------------------------------------------------------------
// FIXME: A ameliorer
void Node::jointTransform(Transform const& p_transform)
{
    m_joint_transform = p_transform;
    updateWorldTransforms();
}

// ----------------------------------------------------------------------------
// FIXME: A ameliorer
void Node::updateWorldTransforms()
{
    if (m_parent)
    {
        m_world_transform =
            m_parent->worldTransform() * m_local_transform * m_joint_transform;
    }
    else
    {
        m_world_transform = m_local_transform * m_joint_transform;
    }

    for (auto const& child : m_children)
    {
        child->updateWorldTransforms();
    }
}

} // namespace robotik