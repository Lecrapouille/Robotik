/**
 * @file Parser.cpp
 * @brief Parser for URDF files to create a robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/URDFParser.hpp"

#include "Robotik/Core/Conversions.hpp"
#include "Robotik/Core/Robot.hpp"

#include <tinyxml2/tinyxml2.h>

#include <algorithm>
#include <fstream>
#include <sstream>

namespace robotik
{

// ------------------------------------------------------------------------
//! \brief Private Link data extracted from the URDF file. Used temporarily
//! to be converted into the definitive robotik::Link object in the method
//! buildSceneGraph().
// ------------------------------------------------------------------------
struct URDFParserLink
{
    explicit URDFParserLink(std::string_view const& p_name) : name(p_name) {}

    std::string name;
    Geometry::Ptr geometry;
    Geometry::Ptr collision;
    Eigen::Vector3f color = Eigen::Vector3f(0.5, 0.5, 0.5);
    Inertial inertial;
    Joint* parent_joint = nullptr;
    std::vector<Joint*> child_joints;
};

// ----------------------------------------------------------------------------
URDFParser::URDFParser() = default;

// ----------------------------------------------------------------------------
URDFParser::~URDFParser() = default;

// ----------------------------------------------------------------------------
URDFParser::URDFParser(URDFParser&&) noexcept = default;

// ----------------------------------------------------------------------------
URDFParser& URDFParser::operator=(URDFParser&&) noexcept = default;

// ----------------------------------------------------------------------------
template <typename Func>
static void forEachChildElement(tinyxml2::XMLElement* p_parent,
                                const char* p_child_name,
                                Func p_func)
{
    for (auto element = p_parent->FirstChildElement(p_child_name); element;
         element = element->NextSiblingElement(p_child_name))
    {
        p_func(element);
    }
}

// ----------------------------------------------------------------------------
static Joint::Type jointType(std::string_view const& p_str_type)
{
    if (p_str_type == "revolute")
        return Joint::Type::REVOLUTE;
    else if (p_str_type == "continuous")
        return Joint::Type::CONTINUOUS;
    else if (p_str_type == "prismatic")
        return Joint::Type::PRISMATIC;
    else
        return Joint::Type::FIXED;
}

// ----------------------------------------------------------------------------
template <int N>
static Eigen::Matrix<double, N, 1> parseVector(const std::string& p_str)
{
    std::istringstream iss(p_str);
    Eigen::Matrix<double, N, 1> vec;
    for (int i = 0; i < N; ++i)
    {
        if (!(iss >> vec[i]))
            vec[i] = 0.0;
    }
    return vec;
}

// ----------------------------------------------------------------------------
static std::string getAttributeOrDefault(tinyxml2::XMLElement const* p_element,
                                         const char* p_attr_name,
                                         const std::string& p_default)
{
    const char* attr = p_element->Attribute(p_attr_name);
    return attr ? std::string(attr) : p_default;
}

// ----------------------------------------------------------------------------
static Eigen::Vector3d
parseOptionalVector3(tinyxml2::XMLElement const* p_element,
                     const char* p_attr_name,
                     const std::string& p_default = "0 0 0")
{
    return parseVector<3>(
        getAttributeOrDefault(p_element, p_attr_name, p_default));
}

// ----------------------------------------------------------------------------
std::unique_ptr<Robot> URDFParser::load(const std::string& p_filename)
{
    if (std::ifstream file(p_filename); !file)
    {
        m_error = "Failed opening '" + p_filename + "'. Reason was '" +
                  strerror(errno) + "'";
        return nullptr;
    }

    tinyxml2::XMLDocument xml;
    if (xml.LoadFile(p_filename.c_str()) != tinyxml2::XML_SUCCESS)
    {
        m_error = "Failed to parse URDF XML: " + std::string(xml.ErrorStr());
        return nullptr;
    }

    auto robot_element = xml.FirstChildElement("robot");
    if (!robot_element)
    {
        m_error = "No <robot> element found in URDF";
        return nullptr;
    }

    return parseRobot(robot_element);
}

// ----------------------------------------------------------------------------
std::unique_ptr<Robot>
URDFParser::parseRobot(tinyxml2::XMLElement* p_robot_element)
{
    std::string robot_name = p_robot_element->Attribute("name");

    // Parse the links before the joints.
    forEachChildElement( //
        p_robot_element,
        "link",
        [this](auto* xml)
        {
            std::string name = xml->Attribute("name");
            auto link = std::make_unique<URDFParserLink>(name);

            parseVisualProperties(xml, *link);
            parseCollisionProperties(xml, *link);
            parseInertialProperties(xml, *link);

            m_links[name] = std::move(link);
        });

    if (m_links.empty())
    {
        m_error = "No links found in URDF";
        return nullptr;
    }

    // Parse the joints.
    forEachChildElement( //
        p_robot_element,
        "joint",
        [this](auto* xml)
        {
            std::string name = xml->Attribute("name");
            Joint::Type type = jointType(xml->Attribute("type"));

            Eigen::Vector3d axis = parseAxis(xml);
            auto [origin_xyz, origin_rpy] = parseOriginTransform(xml);
            auto joint = std::make_unique<Joint>(name, type, axis);
            joint->localTransform(utils::createTransform(
                origin_xyz, origin_rpy.x(), origin_rpy.y(), origin_rpy.z()));

            parseLimits(xml, *joint);
            parseParentChildLinks(xml, *joint);

            m_joints[name] = std::move(joint);
        });

    if (m_joints.empty())
    {
        m_error = "No joints found in URDF";
        return nullptr;
    }

    // Build the scene graph from parsed links and joints.
    return buildSceneGraph(robot_name);
}

// ----------------------------------------------------------------------------
std::unique_ptr<Robot>
URDFParser::buildSceneGraph(std::string const& p_robot_name)
{
    // Since in URDF file, a joint is always associated with a link parent
    // and a link child, we need to find the root link (link with no parent
    // joint) to start building the scene graph.
    auto root_link = findRootLink();
    if (!root_link)
    {
        m_error = "Could not find root link (link with no parent joint)";
        return nullptr;
    }

    // Create the scene graph root node.
    auto root_node = createLinkFromURDFData(*root_link);
    if (!root_node)
    {
        m_error = "We need at least a visual geometry to create a link for " +
                  root_link->name;
        return nullptr;
    }

    // Build the scene graph recursively.
    for (Joint const* child_joint : root_link->child_joints)
    {
        if (auto joint_tree = buildJointTree(child_joint))
        {
            root_node->addChild(std::move(joint_tree));
        }
    }

    // Create the robot.
    auto robot = std::make_unique<Robot>(p_robot_name);
    robot->root(std::move(root_node));
    return robot;
}

// ----------------------------------------------------------------------------
URDFParserLink* URDFParser::findRootLink() const
{
    for (const auto& [name, link] : m_links)
    {
        if (link->parent_joint == nullptr)
        {
            return link.get();
        }
    }
    return nullptr;
}

// ----------------------------------------------------------------------------
std::unique_ptr<Joint> URDFParser::buildJointTree(Joint const* p_current_joint)
{
    // Find the joint in the container of parsed joints.
    auto joint_it =
        std::find_if(m_joints.begin(),
                     m_joints.end(),
                     [p_current_joint](const auto& pair)
                     { return pair.second.get() == p_current_joint; });

    if (joint_it == m_joints.end())
        return nullptr;

    // Use the original joint instead of creating a copy.
    auto joint = std::move(joint_it->second);
    m_joints.erase(joint_it);

    // Find the child link of the current joint.
    URDFParserLink* child_link = nullptr;
    for (const auto& [name, link] : m_links)
    {
        if (link->parent_joint == p_current_joint)
        {
            child_link = link.get();
            break;
        }
    }

    // If the child link is not found, return the joint.
    if (!child_link)
        return joint;

    // Create a link node from the child link.
    auto child_link_node = createLinkFromURDFData(*child_link);
    if (!child_link_node)
    {
        m_error = "We need at least a visual geometry to create a link for " +
                  child_link->name;
        return nullptr;
    }

    // Build the joint tree for the child link.
    for (Joint const* child_joint : child_link->child_joints)
    {
        if (auto child_joint_tree = buildJointTree(child_joint))
        {
            child_link_node->addChild(std::move(child_joint_tree));
        }
    }

    joint->addChild(std::move(child_link_node));

    return joint;
}

// ----------------------------------------------------------------------------
void URDFParser::parseInertialProperties(tinyxml2::XMLElement* p_link_element,
                                         URDFParserLink& p_link) const
{
    if (auto inertial_element = p_link_element->FirstChildElement("inertial"))
    {
        p_link.inertial = parseInertial(inertial_element);
    }
}

// ----------------------------------------------------------------------------
Inertial
URDFParser::parseInertial(tinyxml2::XMLElement* p_inertial_element) const
{
    Inertial inert;
    if (!p_inertial_element)
        return inert;

    // Parse mass directly
    if (auto mass_element = p_inertial_element->FirstChildElement("mass"))
    {
        mass_element->QueryDoubleAttribute("value", &inert.mass);
    }

    if (auto originElement = p_inertial_element->FirstChildElement("origin"))
    {
        inert.center_of_mass = parseOptionalVector3(originElement, "xyz");
    }

    if (auto inertiaElement = p_inertial_element->FirstChildElement("inertia"))
    {
        double ixx = 0, ixy = 0, ixz = 0, iyy = 0, iyz = 0, izz = 0;

        inertiaElement->QueryDoubleAttribute("ixx", &ixx);
        inertiaElement->QueryDoubleAttribute("ixy", &ixy);
        inertiaElement->QueryDoubleAttribute("ixz", &ixz);
        inertiaElement->QueryDoubleAttribute("iyy", &iyy);
        inertiaElement->QueryDoubleAttribute("iyz", &iyz);
        inertiaElement->QueryDoubleAttribute("izz", &izz);

        inert.inertia_matrix << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;
    }

    return inert;
}

// ----------------------------------------------------------------------------
void URDFParser::parseVisualProperties(tinyxml2::XMLElement* p_link_element,
                                       URDFParserLink& p_link) const
{
    auto visual_element = p_link_element->FirstChildElement("visual");
    if (!visual_element)
        return;

    if (auto geometry_element = visual_element->FirstChildElement("geometry"))
    {
        // Create geometry with "_visual" suffix
        std::string visual_name = p_link.name + "_visual";
        p_link.geometry = parseGeometry(geometry_element, visual_name);

        // Apply visual transform and color directly
        if (p_link.geometry)
        {
            Transform visual_origin = parseOriginFromElement(visual_element);
            p_link.geometry->localTransform(visual_origin);

            // Parse and apply material color
            parseMaterial(visual_element, p_link);
            if (auto* geom = dynamic_cast<Geometry*>(p_link.geometry.get()))
            {
                geom->color = p_link.color;
            }
        }
    }
}

// ----------------------------------------------------------------------------
void URDFParser::parseCollisionProperties(tinyxml2::XMLElement* p_link_element,
                                          URDFParserLink& p_link) const
{
    auto collision_element = p_link_element->FirstChildElement("collision");
    if (!collision_element)
        return;

    if (auto geometry_element =
            collision_element->FirstChildElement("geometry"))
    {
        // Create collision geometry with "_collision" suffix
        std::string collision_name = p_link.name + "_collision";
        p_link.collision = parseGeometry(geometry_element, collision_name);
    }
}

// ----------------------------------------------------------------------------
std::unique_ptr<Geometry>
URDFParser::parseGeometry(tinyxml2::XMLElement* p_geometry_element,
                          const std::string& p_name) const
{
    if (!p_geometry_element)
        return nullptr;

    if (auto boxElement = p_geometry_element->FirstChildElement("box"))
    {
        auto size = parseOptionalVector3(boxElement, "size");
        return std::make_unique<robotik::Box>(
            p_name, size.x(), size.y(), size.z());
    }
    else if (auto cylinderElement =
                 p_geometry_element->FirstChildElement("cylinder"))
    {
        double radius = 0.1, length = 0.2;
        cylinderElement->QueryDoubleAttribute("radius", &radius);
        cylinderElement->QueryDoubleAttribute("length", &length);
        return std::make_unique<robotik::Cylinder>(p_name, radius, length);
    }
    else if (auto sphereElement =
                 p_geometry_element->FirstChildElement("sphere"))
    {
        double radius = 0.1;
        sphereElement->QueryDoubleAttribute("radius", &radius);
        return std::make_unique<robotik::Sphere>(p_name, radius);
    }
    else if (auto meshElement = p_geometry_element->FirstChildElement("mesh"))
    {
        std::string mesh_path =
            getAttributeOrDefault(meshElement, "filename", "");
        return std::make_unique<robotik::Mesh>(p_name, mesh_path);
    }

    return nullptr;
}

// ----------------------------------------------------------------------------
Transform
URDFParser::parseOriginFromElement(tinyxml2::XMLElement* p_element) const
{
    auto origin_element = p_element->FirstChildElement("origin");
    if (!origin_element)
        return Transform::Identity();

    Eigen::Vector3d translation = parseOptionalVector3(origin_element, "xyz");
    Eigen::Vector3d rotation = parseOptionalVector3(origin_element, "rpy");

    return utils::createTransform(
        translation, rotation.x(), rotation.y(), rotation.z());
}

// ----------------------------------------------------------------------------
void URDFParser::parseMaterial(tinyxml2::XMLElement* p_visual_element,
                               URDFParserLink& p_link) const
{
    auto material_element = p_visual_element->FirstChildElement("material");
    if (!material_element)
        return;

    auto color_element = material_element->FirstChildElement("color");
    if (!color_element)
        return;

    if (const char* rgba = color_element->Attribute("rgba"))
    {
        p_link.color = parseVector<4>(rgba).head<3>().cast<float>();
    }
}

// ----------------------------------------------------------------------------
void URDFParser::parseParentChildLinks(tinyxml2::XMLElement* p_joint_element,
                                       Joint& p_joint) const
{
    auto parent_element = p_joint_element->FirstChildElement("parent");
    auto child_element = p_joint_element->FirstChildElement("child");

    if (!parent_element || !child_element)
        return;

    const char* parent_name = parent_element->Attribute("link");
    const char* child_name = child_element->Attribute("link");

    if (!parent_name || !child_name)
        return;

    auto parent_it = m_links.find(parent_name);
    auto child_it = m_links.find(child_name);

    if (parent_it != m_links.end() && child_it != m_links.end())
    {
        parent_it->second->child_joints.push_back(&p_joint);
        child_it->second->parent_joint = &p_joint;
    }
}

// ----------------------------------------------------------------------------
Eigen::Vector3d
URDFParser::parseAxis(tinyxml2::XMLElement* p_joint_element) const
{
    auto axis_element = p_joint_element->FirstChildElement("axis");
    return axis_element ? parseOptionalVector3(axis_element, "xyz")
                        : Eigen::Vector3d::UnitZ();
}

// ----------------------------------------------------------------------------
void URDFParser::parseLimits(tinyxml2::XMLElement* p_joint_element,
                             Joint& p_joint) const
{
    auto limit_element = p_joint_element->FirstChildElement("limit");
    if (!limit_element)
        return;

    double lower = 0, upper = 0;
    if (limit_element->QueryDoubleAttribute("lower", &lower) ==
            tinyxml2::XML_SUCCESS &&
        limit_element->QueryDoubleAttribute("upper", &upper) ==
            tinyxml2::XML_SUCCESS)
    {
        p_joint.limits(lower, upper);
    }
}

// ----------------------------------------------------------------------------
std::pair<Eigen::Vector3d, Eigen::Vector3d>
URDFParser::parseOriginTransform(tinyxml2::XMLElement* p_element) const
{
    auto origin_element = p_element->FirstChildElement("origin");
    if (!origin_element)
        return { Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero() };

    return { parseOptionalVector3(origin_element, "xyz"),
             parseOptionalVector3(origin_element, "rpy") };
}

// ----------------------------------------------------------------------------
robotik::Link::Ptr
URDFParser::createLinkFromURDFData(URDFParserLink& p_urdf_link) const
{
    if (p_urdf_link.geometry && p_urdf_link.collision)
    {
        return scene::Node::create<robotik::Link>(
            p_urdf_link.name,
            std::move(p_urdf_link.geometry),
            std::move(p_urdf_link.collision));
    }
    else if (p_urdf_link.geometry)
    {
        return scene::Node::create<robotik::Link>(
            p_urdf_link.name, std::move(p_urdf_link.geometry));
    }

    // We need at least a visual geometry to create a link
    return nullptr;
}

} // namespace robotik