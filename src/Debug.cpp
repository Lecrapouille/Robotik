#include "Robotik/Debug.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

namespace robotik::debug
{

// ----------------------------------------------------------------------------
void printRobot(const Robot& p_robot)
{
    std::cout << "=== Robot: " << p_robot.name() << " ===" << std::endl;
    std::cout << "Hierarchy with transformation matrices:" << std::endl;
    std::cout << std::endl;

    if (p_robot.hasRoot())
    {
        p_robot.root().traverse([](Node const& p_node, size_t p_depth)
                                { printNode(p_node, p_depth); });
    }
    else
    {
        std::cout << "  No root node found!" << std::endl;
    }
}

// ----------------------------------------------------------------------------
void printSceneGraph(const Robot& p_robot)
{
    std::cout << "=== Scene Graph: " << p_robot.name() << " ===" << std::endl;
    std::cout << std::endl;

    if (p_robot.hasRoot())
    {
        // Print root node
        const Node& root = p_robot.root();
        std::cout << "[" << root.name() << "] (T_world = Identity)"
                  << std::endl;

        // Print children with proper tree structure
        const auto& children = root.children();
        for (size_t i = 0; i < children.size(); ++i)
        {
            bool isLast = (i == children.size() - 1);
            printSceneNodeSimple(*children[i], isLast);
        }
    }
    else
    {
        std::cout << "  No root node found!" << std::endl;
    }
}

// ----------------------------------------------------------------------------
void printSceneNodeSimple(Node const& p_node, bool p_isLast)
{
    // Print tree connector
    std::string connector = p_isLast ? "└── " : "├── ";
    std::string indent = p_isLast ? "    " : "│   ";

    // Print node name and type
    if (auto joint = dynamic_cast<Joint const*>(&p_node))
    {
        std::cout << connector << p_node.name()
                  << " (type: " << jointTypeStringCompact(joint->type()) << ")"
                  << std::endl;

        // Print T_origin
        std::cout << indent << "├── T_origin:     "
                  << formatTransform(p_node.localTransform()) << std::endl;

        // Print T_joint
        if (joint->type() == Joint::Type::FIXED)
        {
            std::cout << indent << "├── T_joint:      identity (fixed joint)"
                      << std::endl;
        }
        else
        {
            std::string jointName = p_node.name();
            // Remove "_joint" suffix if present for cleaner variable name
            if (jointName.length() >= 6 &&
                jointName.substr(jointName.length() - 6) == "_joint")
            {
                jointName = jointName.substr(0, jointName.length() - 6);
            }
            std::string jointVar = "θ_" + jointName;
            std::cout << indent
                      << "├── T_joint:      rotate(axis = " << joint->axis().x()
                      << " " << joint->axis().y() << " " << joint->axis().z()
                      << ", θ = " << jointVar << ")   ← variable" << std::endl;
        }

        // Print child link if exists
        if (!p_node.children().empty())
        {
            const Node& child = *p_node.children()[0];
            std::cout << indent << "└── [" << child.name()
                      << "]  (T_world = T_base × T_origin";
            if (joint->type() != Joint::Type::FIXED)
            {
                std::cout << " × T_joint";
            }
            std::cout << ")" << std::endl;
        }

        // Add empty line between joints for readability
        if (!p_isLast)
        {
            std::cout << "│" << std::endl;
        }
    }
}

// ----------------------------------------------------------------------------
void printSceneNode(Node const& p_node, size_t p_depth, bool p_isLast)
{
    // Create indentation based on depth
    std::string indent(p_depth * 3, ' ');

    // Print tree connector
    if (p_depth > 0)
    {
        if (p_isLast)
        {
            std::cout << indent.substr(0, indent.length() - 3) << "└── ";
        }
        else
        {
            std::cout << indent.substr(0, indent.length() - 3) << "├── ";
        }
    }

    // Print node name and type
    if (auto joint = dynamic_cast<Joint const*>(&p_node))
    {
        std::cout << p_node.name()
                  << " (type: " << jointTypeStringCompact(joint->type()) << ")"
                  << std::endl;

        // Print joint-specific information
        std::string jointIndent = indent + (p_isLast ? "    " : "│   ");

        // Print T_origin
        std::cout << jointIndent << "├── T_origin:     "
                  << formatTransform(p_node.localTransform()) << std::endl;

        // Print T_joint
        if (joint->type() == Joint::Type::FIXED)
        {
            std::cout << jointIndent
                      << "├── T_joint:      identity (fixed joint)"
                      << std::endl;
        }
        else
        {
            std::string jointVar = "θ_" + p_node.name();
            std::cout << jointIndent
                      << "├── T_joint:      rotate(axis = " << joint->axis().x()
                      << " " << joint->axis().y() << " " << joint->axis().z()
                      << ", θ = " << jointVar << ")   ← variable" << std::endl;
        }

        // Print child link if exists
        if (!p_node.children().empty())
        {
            const Node& child = *p_node.children()[0];
            std::cout << jointIndent << "└── [" << child.name()
                      << "]  (T_world = T_base × T_origin";
            if (joint->type() != Joint::Type::FIXED)
            {
                std::cout << " × T_joint";
            }
            std::cout << ")" << std::endl;
        }
    }
    else
    {
        std::cout << p_node.name() << " (Node)" << std::endl;
    }

    // Recursively print children
    const auto& children = p_node.children();
    for (size_t i = 0; i < children.size(); ++i)
    {
        bool isLast = (i == children.size() - 1);
        printSceneNode(*children[i], p_depth + 1, isLast);
    }
}

// ----------------------------------------------------------------------------
void printNode(Node const& p_node, size_t p_depth)
{
    // Create indentation based on depth
    std::string indent(p_depth * 2, ' ');

    // Print node name and type
    std::cout << indent << "└─ " << p_node.name();

    // Check if it's a joint and print joint-specific info
    if (auto joint = dynamic_cast<Joint const*>(&p_node))
    {
        std::cout << " (Joint - " << jointTypeString(joint->type()) << ")";
        std::cout << " [Position: " << std::fixed << std::setprecision(3)
                  << joint->position() << "]";
        switch (joint->type())
        {
            case Joint::Type::REVOLUTE:
                std::cout << " [rad]";
                break;
            case Joint::Type::PRISMATIC:
                std::cout << " [m]";
                break;
            case Joint::Type::FIXED:
            default:
                break;
        }
    }
    else
    {
        std::cout << " (Node)";
    }

    std::cout << std::endl;

    // Print local transformation matrix
    std::cout << indent << "   Local Transform:" << std::endl;
    printTransform(p_node.localTransform(), indent + "   ");

    // Print world transformation matrix
    std::cout << indent << "   World Transform:" << std::endl;
    printTransform(p_node.worldTransform(), indent + "   ");

    std::cout << std::endl;
}

// ----------------------------------------------------------------------------
void printJoint(const Joint& p_joint)
{
    std::cout << "Joint: " << p_joint.name() << std::endl;
}

// ----------------------------------------------------------------------------
// void printLink(const Link& p_link)
//{
//    std::cout << "Link: " << p_link.name << std::endl;
//}

// ----------------------------------------------------------------------------
void printTransform(Transform const& p_transform, const std::string& p_indent)
{
    std::cout << std::fixed << std::setprecision(4);

    for (int i = 0; i < 4; ++i)
    {
        std::cout << p_indent;
        for (int j = 0; j < 4; ++j)
        {
            std::cout << std::setw(10) << p_transform(i, j);
            if (j < 3)
                std::cout << " ";
        }
        std::cout << std::endl;
    }
}

// ----------------------------------------------------------------------------
std::string jointTypeString(Joint::Type p_type)
{
    switch (p_type)
    {
        case Joint::Type::REVOLUTE:
            return "REVOLUTE";
        case Joint::Type::PRISMATIC:
            return "PRISMATIC";
        case Joint::Type::FIXED:
            return "FIXED";
        default:
            return "UNKNOWN";
    }
}

// ----------------------------------------------------------------------------
std::string jointTypeStringCompact(Joint::Type p_type)
{
    switch (p_type)
    {
        case Joint::Type::REVOLUTE:
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
std::string formatTransform(const Transform& p_transform)
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