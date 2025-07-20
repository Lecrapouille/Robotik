#include "Robotik/Debug.hpp"

#include <iomanip>
#include <iostream>

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

} // namespace robotik::debug