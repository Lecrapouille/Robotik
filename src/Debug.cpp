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
static std::string generateIndentation(size_t p_depth,
                                       const std::vector<bool>& p_last_flags,
                                       bool p_is_detailed = false)
{
    std::string indent;

    if (p_is_detailed)
    {
        // spacing for the detail of the node
        for (size_t i = 0; i < p_depth; ++i)
        {
            indent += p_last_flags[i] ? "    " : "│   ";
        }
    }
    else
    {
        // spacing for displaying the tree
        for (size_t i = 0; i < p_depth; ++i)
        {
            if (i == p_depth - 1)
            {
                indent += p_last_flags[i] ? "└── " : "├── ";
            }
            else
            {
                indent += p_last_flags[i] ? "    " : "│   ";
            }
        }
    }
    return indent;
}

// ----------------------------------------------------------------------------
static std::string getJointTypeName(Joint::Type p_type)
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
static std::string getJointName(const Joint& p_joint, bool p_detailed)
{
    std::string name = p_joint.name() + " (" + getJointTypeName(p_joint.type());
    if (p_detailed && p_joint.type() != Joint::Type::FIXED)
    {
        // Add axis information
        name += ", axis = (" + std::to_string(p_joint.axis().x()) + ", " +
                std::to_string(p_joint.axis().y()) + ", " +
                std::to_string(p_joint.axis().z()) + "), ";

        // Add angle limits information
        if (p_joint.type() == Joint::Type::REVOLUTE)
        {
            name += "θ ∈ [" + std::to_string(p_joint.limits().first) + ", " +
                    std::to_string(p_joint.limits().second) + "]";
        }

        // Add distance limits information
        else if (p_joint.type() == Joint::Type::PRISMATIC)
        {
            name += "d ∈ [" + std::to_string(p_joint.limits().first) + ", " +
                    std::to_string(p_joint.limits().second) + "]";
        }
    }
    name += ")";
    return name;
}
// ----------------------------------------------------------------------------
static std::string printTransform(Transform const& p_transform)
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
static std::string printGeometryDetails(Geometry const& p_geometry)
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
            oss << "unknown";
            break;
    }

    oss << " color(" << p_geometry.color.x() << ", " << p_geometry.color.y()
        << ", " << p_geometry.color.z() << ")";

    return oss.str();
}

// ----------------------------------------------------------------------------
static std::string printJointDetails(Joint const& p_joint,
                                     std::string const& p_detail_base,
                                     bool const p_end_connector)
{
    std::ostringstream oss;

    // Print the joint local transform
    oss << p_detail_base
        << "├── T_origin: " << printTransform(p_joint.localTransform())
        << std::endl;

    // Print the joint position
    if (p_joint.type() != Joint::Type::FIXED)
    {
        oss << p_detail_base << "├── T_joint:  ";
        if (p_joint.type() == Joint::Type::PRISMATIC)
        {
            oss << "d = " << p_joint.position() << std::endl;
        }
        else if ((p_joint.type() == Joint::Type::REVOLUTE) ||
                 (p_joint.type() == Joint::Type::CONTINUOUS))
        {
            oss << "θ = " << p_joint.position() << std::endl;
        }
    }

    // Print the joint world transform
    oss << p_detail_base << (p_end_connector ? "└──" : "├──")
        << " T_world:  " << printTransform(p_joint.worldTransform())
        << std::endl;

    return oss.str();
}

// ----------------------------------------------------------------------------
static std::string printLinkDetails(Link const& p_link,
                                    const std::string& p_detail_base,
                                    bool const p_end_connector)
{
    std::ostringstream oss;

    oss << p_detail_base
        << "├── geometry: " << printGeometryDetails(p_link.geometry())
        << std::endl;
    oss << p_detail_base
        << "├── T_local:  " << printTransform(p_link.localTransform())
        << std::endl;
    oss << p_detail_base << (p_end_connector ? "└──" : "├──")
        << " T_world:  " << printTransform(p_link.worldTransform())
        << std::endl;

    return oss.str();
}

// ----------------------------------------------------------------------------
static std::string printNodeRecursive(scene::Node const& p_node,
                                      size_t p_depth,
                                      std::vector<bool>& p_last_flags,
                                      bool const p_detailed)
{
    std::ostringstream oss;

    // Print the appropriate indentation
    oss << generateIndentation(p_depth, p_last_flags);

    // Print the node name and the appropriate details if required
    if (auto joint = dynamic_cast<const Joint*>(&p_node))
    {
        oss << getJointName(*joint, p_detailed) << std::endl;
        if (p_detailed)
        {
            bool end_connector = p_node.children().empty();
            auto indentation = generateIndentation(p_depth, p_last_flags, true);
            oss << printJointDetails(*joint, indentation, end_connector);
        }
    }
    else if (auto link = dynamic_cast<const Link*>(&p_node))
    {
        oss << "[" + link->name() + "]" << std::endl;
        if (p_detailed)
        {
            bool end_connector = p_node.children().empty();
            auto indentation = generateIndentation(p_depth, p_last_flags, true);
            // Print the link details with the appropriate indentation
            oss << printLinkDetails(*link, indentation, end_connector);
        }
    }
    else
    {
        oss << p_node.name() << std::endl;
    }

    // Process children
    const auto& children = p_node.children();
    for (size_t i = 0; i < children.size(); ++i)
    {
        if (p_depth >= p_last_flags.size())
        {
            p_last_flags.resize(p_depth + 1);
        }

        bool is_last_child = (i == children.size() - 1);
        p_last_flags[p_depth] = is_last_child;

        oss << printNodeRecursive(
            *children[i], p_depth + 1, p_last_flags, p_detailed);
    }

    return oss.str();
}

// ----------------------------------------------------------------------------
std::string printRobot(Robot const& p_robot, bool const p_detailed)
{
    std::ostringstream oss;

    oss << "Robot: " << p_robot.name() << std::endl;
    if (!p_robot.hasRoot())
    {
        oss << "  No root node found!" << std::endl;
        return oss.str();
    }

    std::vector<bool> last_flags;
    oss << printNodeRecursive(p_robot.root(), 0, last_flags, p_detailed);

    return oss.str();
}

} // namespace robotik::debug