#include "Robotik/Debug.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

namespace robotik::debug
{

static void printNode(Node const& p_node, bool p_isLast, size_t p_depth);
static std::string printJointType(Joint::Type p_type);
static std::string printTransform(const Transform& p_transform);
static std::string printGeometryType(Geometry::Type p_type);

// ----------------------------------------------------------------------------
void printRobot(const Robot& p_robot)
{
    std::cout << "Robot: " << p_robot.name() << std::endl;
    if (!p_robot.hasRoot())
    {
        std::cout << "  No root node found!" << std::endl;
        return;
    }

    // Print root node
    printNode(p_robot.root(), true, 0);

#if 0
    std::cout << "[" << root.name() << "]" << std::endl;

    // Print children with proper tree structure
    const auto& children = root.children();
    for (size_t i = 0; i < children.size(); ++i)
    {
        bool isLast = (i == children.size() - 1);
        printNode(*children[i], isLast, 0);
    }
#endif
}

// ----------------------------------------------------------------------------
static void printNode(Node const& p_node, bool p_isLast, size_t p_depth)
{
    // Create base indentation for current depth
    std::string base_indent;
    for (size_t i = 0; i < p_depth; ++i)
    {
        base_indent += "    ";
    }

    // Print tree connector
    std::string connector = p_isLast ? "└── " : "├── ";
    std::string joint_indent = base_indent + (p_isLast ? "    " : "│   ");

    // Print node name and type
    if (auto joint = dynamic_cast<Joint const*>(&p_node))
    {
        std::cout << base_indent << connector << p_node.name() << " ("
                  << printJointType(joint->type()) << ")" << std::endl;

        // Print T_origin
        std::cout << joint_indent << "├── T_origin:     "
                  << printTransform(p_node.localTransform()) << std::endl;

        // Print T_joint
        if (joint->type() == Joint::Type::FIXED)
        {
            std::cout << joint_indent
                      << "├── T_joint:      identity (fixed joint)"
                      << std::endl;
        }
        else
        {
            // FIXME deplacer rotate(axis = 0 0 1, θ ∈ [-3.14159, 3.14159]) a
            // cote de (type:) Afficher la valeur de l'angle
            if (joint->type() == Joint::Type::PRISMATIC)
            {
                std::cout << joint_indent
                          << "├── T_joint:      translate(axis = "
                          << joint->axis().x() << " " << joint->axis().y()
                          << " " << joint->axis().z() << ", d ∈ ["
                          << joint->limits().first << ", "
                          << joint->limits().second << "]"
                          << ")" << std::endl;
            }
            else if (joint->type() == Joint::Type::CONTINUOUS)
            {
                std::cout << joint_indent << "├── T_joint:      rotate(axis = "
                          << joint->axis().x() << " " << joint->axis().y()
                          << " " << joint->axis().z() << ")" << std::endl;
            }
            else if (joint->type() == Joint::Type::REVOLUTE)
            {
                std::cout << joint_indent << "├── T_joint:      rotate(axis = "
                          << joint->axis().x() << " " << joint->axis().y()
                          << " " << joint->axis().z() << ", θ ∈ ["
                          << joint->limits().first << ", "
                          << joint->limits().second << "]"
                          << ")" << std::endl;
            }
            else
            {
                std::cout << joint_indent << "├── T_joint:      unknown"
                          << std::endl;
            }
        }

        // Print child link if exists
        if (!p_node.children().empty())
        {
            const Node& child = *p_node.children()[0];

            // Check if this link has child joints (continue the chain)
            const auto& grand_children = child.children();
            bool has_child_joints = !grand_children.empty();

            std::string link_connector = has_child_joints ? "├── " : "└── ";
            std::cout << joint_indent << link_connector << "[" << child.name()
                      << "] " // << printGeometryType(child.geometry().type())
                      << std::endl;

            // Recursively print the children of the child link (next joints in
            // the kinematic chain)
            if (has_child_joints)
            {
                for (size_t i = 0; i < grand_children.size(); ++i)
                {
                    bool is_child_last = (i == grand_children.size() - 1);
                    printNode(*grand_children[i], is_child_last, p_depth + 1);
                }
            }
        }

        // Add empty line between joints for readability at the same level (only
        // at root level)
        if (!p_isLast && p_depth == 0)
        {
            std::cout << "│" << std::endl;
        }
    }
    else
    {
        // Handle regular nodes (links without being joints)
        std::cout << base_indent << connector << "[" << p_node.name() << "]"
                  << std::endl;

        // Recursively print children
        const auto& children = p_node.children();
        for (size_t i = 0; i < children.size(); ++i)
        {
            bool is_child_last = (i == children.size() - 1);
            printNode(*children[i], is_child_last, p_depth + 1);
        }
    }
}

// ----------------------------------------------------------------------------
static std::string printGeometryType(Geometry::Type p_type)
{
    switch (p_type)
    {
        case Geometry::Type::BOX:
            return "box";
        case Geometry::Type::CYLINDER:
            return "cylinder";
        case Geometry::Type::SPHERE:
            return "sphere";
        case Geometry::Type::MESH:
            return "mesh";
        default:
            return "unknown";
    }
}

// ----------------------------------------------------------------------------
static std::string printJointType(Joint::Type p_type)
{
    switch (p_type)
    {
        case Joint::Type::REVOLUTE:
            return "revolute";
        case Joint::Type::CONTINUOUS:
            return "continuous";
        case Joint::Type::PRISMATIC:
            return "prismatic";
        case Joint::Type::FIXED:
            return "fixed";
        default:
            return "unknown";
    }
}

// ----------------------------------------------------------------------------
static std::string printTransform(const Transform& p_transform)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);

    // Extract translation
    double tx = p_transform(0, 3);
    double ty = p_transform(1, 3);
    double tz = p_transform(2, 3);

    // Extract rotation (simplified - assuming rotation around principal axes)
    // This is a simplified approach - in practice you'd want to extract Euler
    // angles
    double rx = 0.0, ry = 0.0, rz = 0.0;

    // Simple heuristic to detect rotations around principal axes
    if (std::abs(p_transform(0, 0) - 1.0) < 1e-6 &&
        std::abs(p_transform(1, 1) - 1.0) < 1e-6)
    {
        // Rotation around Z-axis
        rz = std::atan2(p_transform(1, 0), p_transform(0, 0));
    }
    else if (std::abs(p_transform(0, 0) - 1.0) < 1e-6 &&
             std::abs(p_transform(2, 2) - 1.0) < 1e-6)
    {
        // Rotation around Y-axis
        ry = std::atan2(p_transform(2, 0), p_transform(0, 0));
    }
    else if (std::abs(p_transform(1, 1) - 1.0) < 1e-6 &&
             std::abs(p_transform(2, 2) - 1.0) < 1e-6)
    {
        // Rotation around X-axis
        rx = std::atan2(p_transform(2, 1), p_transform(1, 1));
    }

    oss << "translate(" << tx << ", " << ty << ", " << tz << ")";

    if (std::abs(rx) > 1e-6 || std::abs(ry) > 1e-6 || std::abs(rz) > 1e-6)
    {
        oss << " · rotate(" << rx << ", " << ry << ", " << rz << ")";
    }

    return oss.str();
}

} // namespace robotik::debug