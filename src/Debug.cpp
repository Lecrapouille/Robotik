#include "Robotik/Debug.hpp"
#include "Robotik/private/Conversions.hpp"
#include "Robotik/private/Joint.hpp"
#include "Robotik/private/Link.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace robotik::debug
{

// ----------------------------------------------------------------------------
static std::string getNodeName(const scene::Node& p_node, bool p_detailed)
{
    if (auto joint = dynamic_cast<const Joint*>(&p_node))
    {
        std::string type_name;
        switch (joint->type())
        {
            case Joint::Type::REVOLUTE:
                type_name = "revolute";
                break;
            case Joint::Type::CONTINUOUS:
                type_name = "continuous";
                break;
            case Joint::Type::PRISMATIC:
                type_name = "prismatic";
                break;
            case Joint::Type::FIXED:
                type_name = "fixed";
                break;
            default:
                type_name = "unknown";
                break;
        }

        std::string name = p_node.name() + " (" + type_name;
        if (p_detailed && joint->type() != Joint::Type::FIXED)
        {
            // Add axis information
            name += ", axis = (" + std::to_string(joint->axis().x()) + ", " +
                    std::to_string(joint->axis().y()) + ", " +
                    std::to_string(joint->axis().z()) + "), ";
            if (joint->type() == Joint::Type::REVOLUTE)
            {
                name += "θ ∈ [" + std::to_string(joint->limits().first) + ", " +
                        std::to_string(joint->limits().second) + "]";
            }
            else if (joint->type() == Joint::Type::PRISMATIC)
            {
                name += "d ∈ [" + std::to_string(joint->limits().first) + ", " +
                        std::to_string(joint->limits().second) + "]";
            }
        }
        name += ")";
        return name;
    }
    else if (dynamic_cast<const Link*>(&p_node))
    {
        return "[" + p_node.name() + "]";
    }
    else
    {
        return p_node.name();
    }
}

// ----------------------------------------------------------------------------
static std::string printTransform(const Transform& p_transform)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);

    Eigen::Vector3d xyz = robotik::utils::getTranslation(p_transform);
    Eigen::Matrix3d rot = robotik::utils::getRotation(p_transform);
    Eigen::Vector3d rpy = robotik::utils::rotationToEuler(rot);

    oss << "xyz = (" << xyz.x() << ", " << xyz.y() << ", " << xyz.z() << ")";
    oss << ", rpy = (" << rpy.x() << ", " << rpy.y() << ", " << rpy.z() << ")";

    return oss.str();
}

// ----------------------------------------------------------------------------
static std::string printGeometryDetails(const Geometry& p_geometry)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4);

    switch (p_geometry.type())
    {
        case Geometry::Type::BOX:
            oss << "box";
            if (p_geometry.parameters().size() >= 3)
            {
                oss << "(x=" << p_geometry.parameters()[0]
                    << ", y=" << p_geometry.parameters()[1]
                    << ", z=" << p_geometry.parameters()[2] << ")";
            }
            break;
        case Geometry::Type::CYLINDER:
            oss << "cylinder";
            if (p_geometry.parameters().size() >= 2)
            {
                oss << "(r=" << p_geometry.parameters()[0]
                    << ", h=" << p_geometry.parameters()[1] << ")";
            }
            break;
        case Geometry::Type::SPHERE:
            oss << "sphere";
            if (p_geometry.parameters().size() >= 1)
            {
                oss << "(r=" << p_geometry.parameters()[0] << ")";
            }
            break;
        case Geometry::Type::MESH:
            oss << "mesh";
            if (!p_geometry.meshPath().empty())
            {
                oss << "(" << p_geometry.meshPath() << ")";
            }
            break;
        default:
            break;
    }

    oss << " color(" << p_geometry.color.x() << ", " << p_geometry.color.y()
        << ", " << p_geometry.color.z() << ")";

    return oss.str();
}

// ----------------------------------------------------------------------------
static void printJointDetails(const Joint* p_joint,
                              const std::string& p_detail_base,
                              bool p_end_connector)
{
    std::cout << p_detail_base
              << "├── T_origin: " << printTransform(p_joint->localTransform())
              << std::endl;
    std::cout << p_detail_base << "├── T_joint:  ";
    if (p_joint->type() == Joint::Type::FIXED)
    {
        std::cout << "identity (fixed joint)" << std::endl;
    }
    else if (p_joint->type() == Joint::Type::PRISMATIC)
    {
        std::cout << "d = " << p_joint->position() << std::endl;
    }
    else if ((p_joint->type() == Joint::Type::REVOLUTE) ||
             (p_joint->type() == Joint::Type::CONTINUOUS))
    {
        std::cout << "θ = " << p_joint->position() << std::endl;
    }
    else
    {
        std::cout << "unknown" << std::endl;
    }
    std::cout << p_detail_base << (p_end_connector ? "└──" : "├──")
              << " T_world:  " << printTransform(p_joint->worldTransform())
              << std::endl;
}

// ----------------------------------------------------------------------------
static void printLinkDetails(const Link* p_link,
                             const std::string& p_detail_base,
                             bool p_end_connector)
{
    std::cout << p_detail_base
              << "├── geometry: " << printGeometryDetails(p_link->geometry())
              << std::endl;
    std::cout << p_detail_base
              << "├── T_local:  " << printTransform(p_link->localTransform())
              << std::endl;
    std::cout << p_detail_base << (p_end_connector ? "└──" : "├──")
              << " T_world:  " << printTransform(p_link->worldTransform())
              << std::endl;
}

// ----------------------------------------------------------------------------
static void printNodeRecursive(const scene::Node& p_node,
                               size_t p_depth,
                               std::vector<bool>& p_is_last_at_depth,
                               bool p_detailed)
{
    // Build indentation string
    std::string indent;
    for (size_t i = 0; i < p_depth; ++i)
    {
        if (i == p_depth - 1)
        {
            // Last level - use connector
            indent += p_is_last_at_depth[i] ? "└── " : "├── ";
        }
        else
        {
            // Intermediate levels
            indent += p_is_last_at_depth[i] ? "    " : "│   ";
        }
    }

    // Print the node
    std::cout << indent << getNodeName(p_node, p_detailed) << std::endl;

    // Print detailed information if requested
    if (p_detailed)
    {
        std::string detail_base;
        for (size_t i = 0; i < p_depth; ++i)
        {
            detail_base += p_is_last_at_depth[i] ? "    " : "│   ";
        }

        bool end_connector = p_node.children().empty();
        if (auto joint = dynamic_cast<const Joint*>(&p_node))
        {
            printJointDetails(joint, detail_base, end_connector);
        }
        else if (auto link = dynamic_cast<const Link*>(&p_node))
        {
            printLinkDetails(link, detail_base, end_connector);
        }
    }

    // Process children
    const auto& children = p_node.children();
    for (size_t i = 0; i < children.size(); ++i)
    {
        bool is_last_child = (i == children.size() - 1);

        // Ensure vector is large enough
        if (p_depth >= p_is_last_at_depth.size())
        {
            p_is_last_at_depth.resize(p_depth + 1);
        }
        p_is_last_at_depth[p_depth] = is_last_child;

        printNodeRecursive(
            *children[i], p_depth + 1, p_is_last_at_depth, p_detailed);
    }
}

// ----------------------------------------------------------------------------
void printRobot(const Robot& p_robot, bool p_detailed)
{
    std::cout << "Robot: " << p_robot.name() << std::endl;
    if (!p_robot.hasRoot())
    {
        std::cout << "  No root node found!" << std::endl;
        return;
    }

    // Print the tree structure
    std::vector<bool> is_last_at_depth;
    printNodeRecursive(p_robot.root(), 0, is_last_at_depth, p_detailed);
}

} // namespace robotik::debug