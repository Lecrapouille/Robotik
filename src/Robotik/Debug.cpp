/**
 * @file Debug.cpp
 * @brief Debug printing functions for debugging the robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Conversions.hpp"
#include "Robotik/Core/Robot.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>

// ANSI color codes
namespace
{
const std::string RESET = "\033[0m";
const std::string BOLD = "\033[1m";
const std::string DIM = "\033[2m";

// Colors
const std::string RED = "\033[31m";
const std::string GREEN = "\033[32m";
const std::string YELLOW = "\033[33m";
const std::string BLUE = "\033[34m";
const std::string MAGENTA = "\033[35m";
const std::string CYAN = "\033[36m";
const std::string WHITE = "\033[37m";

// Background colors
const std::string BG_BLUE = "\033[44m";
const std::string BG_GREEN = "\033[42m";
const std::string BG_YELLOW = "\033[43m";
const std::string BG_RED = "\033[41m";

// Emojis for different elements
const std::string ROBOT_EMOJI = "🤖";
const std::string JOINT_EMOJI = "🔗";
const std::string LINK_EMOJI = "📦";
const std::string TRANSFORM_EMOJI = "🔄";
const std::string GEOMETRY_EMOJI = "📐";
const std::string AXIS_EMOJI = "📏";
const std::string LIMITS_EMOJI = "📊";
const std::string POSITION_EMOJI = "📍";
const std::string WORLD_EMOJI = "🌍";
const std::string LOCAL_EMOJI = "🏠";
} // namespace

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
            return BLUE + "🔄 revolute" + RESET;
        case Joint::Type::CONTINUOUS:
            return CYAN + "🔄 continuous" + RESET;
        case Joint::Type::PRISMATIC:
            return GREEN + "↔️ prismatic" + RESET;
        case Joint::Type::FIXED:
            return DIM + "🔒 fixed" + RESET;
        default:
            return RED + "❓ unknown" + RESET;
    }
}

// ----------------------------------------------------------------------------
static std::string getJointName(const Joint& p_joint, bool p_detailed)
{
    std::string name = JOINT_EMOJI + " " + BOLD + YELLOW + p_joint.name() +
                       RESET + " (" + getJointTypeName(p_joint.type());
    if (p_detailed && p_joint.type() != Joint::Type::FIXED)
    {
        // Add axis information
        name += ", " + AXIS_EMOJI + " " + CYAN + "axis = (" +
                std::to_string(p_joint.axis().x()) + ", " +
                std::to_string(p_joint.axis().y()) + ", " +
                std::to_string(p_joint.axis().z()) + ")" + RESET + ", ";

        // Add angle limits information
        if (p_joint.type() == Joint::Type::REVOLUTE)
        {
            name += POSITION_EMOJI + " " + MAGENTA + "θ ∈ [" +
                    std::to_string(p_joint.limits().first) + ", " +
                    std::to_string(p_joint.limits().second) + "]" + RESET;
        }

        // Add distance limits information
        else if (p_joint.type() == Joint::Type::PRISMATIC)
        {
            name += POSITION_EMOJI + " " + MAGENTA + "d ∈ [" +
                    std::to_string(p_joint.limits().first) + ", " +
                    std::to_string(p_joint.limits().second) + "]" + RESET;
        }
    }
    name += ")";
    return name;
}
// ----------------------------------------------------------------------------
static std::string printTransform(Transform const& p_transform)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);

    Eigen::Vector3d xyz = robotik::utils::getTranslation(p_transform);
    Eigen::Matrix3d rot = robotik::utils::getRotation(p_transform);
    Eigen::Vector3d rpy = robotik::utils::rotationToEuler(rot);

    oss << POSITION_EMOJI << " " << GREEN << "xyz = (" << xyz.x() << ", "
        << xyz.y() << ", " << xyz.z() << ")" << RESET;
    oss << ", " << TRANSFORM_EMOJI << " " << BLUE << "rpy = (" << rpy.x()
        << ", " << rpy.y() << ", " << rpy.z() << ")" << RESET;

    return oss.str();
}

// ----------------------------------------------------------------------------
static std::string printGeometryDetails(Geometry const& p_geometry)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);

    switch (p_geometry.type())
    {
        case Geometry::Type::BOX:
            oss << YELLOW << "📦 box" << RESET;
            if (p_geometry.parameters().size() >= 3)
            {
                oss << "(" << CYAN << "x=" << p_geometry.parameters()[0]
                    << ", y=" << p_geometry.parameters()[1]
                    << ", z=" << p_geometry.parameters()[2] << RESET << ")";
            }
            break;
        case Geometry::Type::CYLINDER:
            oss << YELLOW << "🥫 cylinder" << RESET;
            if (p_geometry.parameters().size() >= 2)
            {
                oss << "(" << CYAN << "r=" << p_geometry.parameters()[0]
                    << ", h=" << p_geometry.parameters()[1] << RESET << ")";
            }
            break;
        case Geometry::Type::SPHERE:
            oss << YELLOW << "⚪ sphere" << RESET;
            if (p_geometry.parameters().size() >= 1)
            {
                oss << "(" << CYAN << "r=" << p_geometry.parameters()[0]
                    << RESET << ")";
            }
            break;
        case Geometry::Type::MESH:
            oss << YELLOW << "🔷 mesh" << RESET;
            if (!p_geometry.meshPath().empty())
            {
                oss << "(" << CYAN << p_geometry.meshPath() << RESET << ")";
            }
            break;
        default:
            oss << RED << "❓ unknown" << RESET;
            break;
    }

    oss << " " << MAGENTA << "🎨 color(" << p_geometry.color.x() << ", "
        << p_geometry.color.y() << ", " << p_geometry.color.z() << ")" << RESET;

    return oss.str();
}

// ----------------------------------------------------------------------------
static std::string printGeometryDetails(Geometry const& p_geometry,
                                        std::string const& p_indentation)
{
    std::ostringstream oss;

    oss << p_indentation << "└─ " << printGeometryDetails(p_geometry)
        << std::endl;

    return oss.str();
}

// ----------------------------------------------------------------------------
static std::string printJointDetails(Joint const& p_joint,
                                     std::string const& p_indentation,
                                     bool const p_end_connector)
{
    std::ostringstream oss;

    // Print the joint local transform
    oss << p_indentation << "│    ├─ " << LOCAL_EMOJI << " " << CYAN << "origin"
        << RESET << ": " << printTransform(p_joint.localTransform())
        << std::endl;

    // Print the joint position
    if (p_joint.type() != Joint::Type::FIXED)
    {
        oss << p_indentation << "│    ├─ " << POSITION_EMOJI << " " << GREEN
            << "joint" << RESET << ":  ";
        if (p_joint.type() == Joint::Type::PRISMATIC)
        {
            oss << MAGENTA << "d = " << p_joint.position() << RESET
                << std::endl;
        }
        else if ((p_joint.type() == Joint::Type::REVOLUTE) ||
                 (p_joint.type() == Joint::Type::CONTINUOUS))
        {
            oss << MAGENTA << "θ = " << p_joint.position() << RESET
                << std::endl;
        }
    }

    // Print the joint world transform
    oss << p_indentation << (p_end_connector ? "│    └─ " : "│    │   ")
        << WORLD_EMOJI << " " << BLUE << "world" << RESET << ":  "
        << printTransform(p_joint.worldTransform()) << std::endl;

    return oss.str();
}

// ----------------------------------------------------------------------------
static std::string printLinkDetails(Link const& p_link,
                                    const std::string& p_indentation,
                                    bool const p_end_connector)
{
    std::ostringstream oss;

    oss << p_indentation << "│    ├─ " << LOCAL_EMOJI << " " << CYAN << "local"
        << RESET << ":  " << printTransform(p_link.localTransform())
        << std::endl;
    oss << p_indentation << (p_end_connector ? "│    └─ " : "│    │   ")
        << WORLD_EMOJI << " " << BLUE << "world" << RESET << ":  "
        << printTransform(p_link.worldTransform()) << std::endl;

    return oss.str();
}

// ----------------------------------------------------------------------------
static std::string printNodeRecursive(scene::Node const& p_node,
                                      size_t p_depth,
                                      std::vector<bool>& p_last_flags,
                                      bool const p_detailed)
{
    std::ostringstream oss;

    oss << generateIndentation(p_depth, p_last_flags, false);

    // Print information depending on the appropriate node type
    if (auto joint = dynamic_cast<const Joint*>(&p_node))
    {
        oss << getJointName(*joint, p_detailed) << std::endl;
        if (p_detailed)
        {
            bool end_connector = !p_node.children().empty();
            auto indentation = generateIndentation(p_depth, p_last_flags, true);
            oss << printJointDetails(*joint, indentation, end_connector);
        }
    }
    else if (auto link = dynamic_cast<const Link*>(&p_node))
    {
        oss << LINK_EMOJI << " " << BOLD << GREEN << "[" + link->name() + "]"
            << RESET << std::endl;
        if (p_detailed)
        {
            bool end_connector = !p_node.children().empty();
            auto indentation = generateIndentation(p_depth, p_last_flags, true);
            oss << printLinkDetails(*link, indentation, end_connector);
        }
    }
    else if (auto geometry = dynamic_cast<const Geometry*>(&p_node))
    {
        oss << GEOMETRY_EMOJI << " " << BOLD << GREEN << geometry->name()
            << RESET << std::endl;
        if (p_detailed)
        {
            auto indentation = generateIndentation(p_depth, p_last_flags, true);
            oss << printGeometryDetails(*geometry, indentation);
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

    oss << ROBOT_EMOJI << " " << BOLD << MAGENTA << "Robot" << RESET << ": "
        << BOLD << WHITE << p_robot.name() << RESET << std::endl;
    if (!p_robot.hasRoot())
    {
        oss << "  " << RED << "❌ No root node found!" << RESET << std::endl;
        return oss.str();
    }

    std::vector<bool> last_flags;
    oss << printNodeRecursive(p_robot.root(), 0, last_flags, p_detailed);

    return oss.str();
}

} // namespace robotik::debug