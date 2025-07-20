#include "Robotik/Parser.hpp"
#include "Robotik/private/Conversions.hpp"
#include "Robotik/private/Link.hpp"

#include <tinyxml2/tinyxml2.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <unordered_map>

namespace robotik
{

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

    // Parse the robot name and create a new Robot object
    m_robot = std::make_unique<Robot>(robot_element->Attribute("name"));

    // Parse the robot links before parsing joints to ensure that the links are
    // created before the joints are added to them.
    parseLinks(robot_element);

    // Parse the robot joints
    parseJoints(robot_element);

    if (m_links.empty() || m_joints.empty())
    {
        m_error = "No links or joints found in URDF";
        return nullptr;
    }

    // Build the scene graph
    if (!buildSceneGraph())
        return nullptr;

    return std::move(m_robot);
}

// ----------------------------------------------------------------------------
void URDFParser::parseLinks(tinyxml2::XMLElement* p_robot_element)
{
    for (auto link_element = p_robot_element->FirstChildElement("link");
         link_element;
         link_element = link_element->NextSiblingElement("link"))
    {
        std::string name = getRequiredAttribute(link_element, "name");
        auto link = std::make_unique<URDFParser::Link>(name);

        // Parse visual and inertial properties
        parseVisualProperties(link_element, *link);
        parseInertialProperties(link_element, *link);

        m_links[name] = std::move(link);
    }
}

// ----------------------------------------------------------------------------
void URDFParser::parseInertialProperties(tinyxml2::XMLElement* p_link_element,
                                         Link& p_link) const
{
    auto inertial_element = p_link_element->FirstChildElement("inertial");
    if (!inertial_element)
        return;

    tinyxml2::XMLPrinter printer;
    inertial_element->Accept(&printer);
    p_link.inertial = parseInertial(printer.CStr());
}

// ----------------------------------------------------------------------------
Inertial URDFParser::parseInertial(const std::string& p_xml) const
{
    Inertial inert;

    tinyxml2::XMLDocument doc;
    if (doc.Parse(p_xml.c_str()) != tinyxml2::XML_SUCCESS)
    {
        return inert;
    }

    auto inertialElement = doc.FirstChildElement("inertial");
    if (!inertialElement)
    {
        return inert;
    }

    // Parse mass
    if (auto massElement = inertialElement->FirstChildElement("mass"))
    {
        massElement->QueryDoubleAttribute("value", &inert.mass);
    }

    // Parse center of mass from origin
    if (auto originElement = inertialElement->FirstChildElement("origin"))
    {
        const char* xyzAttr = originElement->Attribute("xyz");
        if (xyzAttr)
        {
            inert.center_of_mass = parseVector3(xyzAttr);
        }
    }

    // Parse inertia matrix
    if (auto inertiaElement = inertialElement->FirstChildElement("inertia"))
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
                                       Link& p_link) const
{
    auto visual_element = p_link_element->FirstChildElement("visual");
    if (!visual_element)
        return;

    // Parse geometry
    if (auto geometry_element = visual_element->FirstChildElement("geometry"))
    {
        p_link.geometry = parseGeometryFromElement(geometry_element);
    }

    // Parse visual origin
    p_link.geometry.origin = parseOriginFromElement(visual_element);

    // Parse material
    parseMaterial(visual_element, p_link.geometry);
}

// ----------------------------------------------------------------------------
Geometry URDFParser::parseGeometryFromElement(
    tinyxml2::XMLElement const* p_geometry_element) const
{
    tinyxml2::XMLPrinter printer;
    p_geometry_element->Accept(&printer);
    return parseGeometry(printer.CStr());
}

// ----------------------------------------------------------------------------
Geometry URDFParser::parseGeometry(const std::string& p_xml) const
{
    Geometry geom;

    tinyxml2::XMLDocument doc;
    if (doc.Parse(p_xml.c_str()) != tinyxml2::XML_SUCCESS)
    {
        // If parsing fails, return default geometry
        return geom;
    }

    // Look for geometry elements
    auto geometryElement = doc.FirstChildElement("geometry");
    if (!geometryElement)
    {
        return geom;
    }

    // Check for different geometry types
    if (auto boxElement = geometryElement->FirstChildElement("box"))
    {
        geom.type = Geometry::Type::BOX;
        const char* sizeAttr = boxElement->Attribute("size");
        if (sizeAttr)
        {
            Eigen::Vector3d size = parseVector3(sizeAttr);
            geom.parameters = { size.x(), size.y(), size.z() };
        }
    }
    else if (auto cylinderElement =
                 geometryElement->FirstChildElement("cylinder"))
    {
        geom.type = Geometry::Type::CYLINDER;
        double radius = 0.1, length = 0.2;
        cylinderElement->QueryDoubleAttribute("radius", &radius);
        cylinderElement->QueryDoubleAttribute("length", &length);
        geom.parameters = { radius, length };
    }
    else if (auto sphereElement = geometryElement->FirstChildElement("sphere"))
    {
        geom.type = Geometry::Type::SPHERE;
        double radius = 0.1;
        sphereElement->QueryDoubleAttribute("radius", &radius);
        geom.parameters = { radius };
    }
    else if (auto meshElement = geometryElement->FirstChildElement("mesh"))
    {
        geom.type = Geometry::Type::MESH;
        const char* filename = meshElement->Attribute("filename");
        if (filename)
        {
            geom.mesh_path = filename;
        }
    }

    return geom;
}

// ----------------------------------------------------------------------------
Transform
URDFParser::parseOriginFromElement(tinyxml2::XMLElement* p_element) const
{
    auto origin_element = p_element->FirstChildElement("origin");
    if (!origin_element)
        return Transform::Identity();

    std::string xyz = getAttributeOrDefault(origin_element, "xyz", "0 0 0");
    Eigen::Vector3d translation = parseVector3(xyz);

    std::string rpy = getAttributeOrDefault(origin_element, "rpy", "0 0 0");
    Eigen::Vector3d rotation = parseVector3(rpy);

    return utils::createTransform(
        translation, rotation.x(), rotation.y(), rotation.z());
}

// ----------------------------------------------------------------------------
void URDFParser::parseMaterial(tinyxml2::XMLElement* p_visual_element,
                               Geometry& p_geometry) const
{
    auto material_element = p_visual_element->FirstChildElement("material");
    if (!material_element)
        return;

    auto color_element = material_element->FirstChildElement("color");
    if (!color_element)
        return;

    const char* rgba = color_element->Attribute("rgba");
    if (rgba)
    {
        p_geometry.color = parseVector4(rgba);
    }
}

// ----------------------------------------------------------------------------
void URDFParser::parseJoints(tinyxml2::XMLElement* p_robot_element)
{
    for (auto joint_element = p_robot_element->FirstChildElement("joint");
         joint_element;
         joint_element = joint_element->NextSiblingElement("joint"))
    {
        std::string name = getRequiredAttribute(joint_element, "name");
        Joint::Type type =
            parseJointType(getRequiredAttribute(joint_element, "type"));

        // Parse axis, origin, and create joint
        Eigen::Vector3d axis = parseAxis(joint_element);
        auto [origin_xyz, origin_rpy] = parseOriginTransform(joint_element);
        auto joint = std::make_unique<Joint>(name, type, axis);
        joint->localTransform(origin_xyz, origin_rpy);

        // Parse limits and link relationships
        parseLimits(joint_element, *joint);
        parseParentChildLinks(joint_element, *joint);

        m_joints[name] = std::move(joint);
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
Joint::Type URDFParser::parseJointType(std::string_view const& p_str_type) const
{
    if (p_str_type == "revolute" || p_str_type == "continuous")
        return Joint::Type::REVOLUTE;
    else if (p_str_type == "prismatic")
        return Joint::Type::PRISMATIC;
    else
        return Joint::Type::FIXED;
}

// ----------------------------------------------------------------------------
Eigen::Vector3d
URDFParser::parseAxis(tinyxml2::XMLElement* p_joint_element) const
{
    auto axis_element = p_joint_element->FirstChildElement("axis");
    if (!axis_element)
        return Eigen::Vector3d::UnitZ();

    const char* xyz_attr = axis_element->Attribute("xyz");
    return xyz_attr ? parseVector3(xyz_attr) : Eigen::Vector3d::UnitZ();
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

    std::string xyz = getAttributeOrDefault(origin_element, "xyz", "0 0 0");
    std::string rpy = getAttributeOrDefault(origin_element, "rpy", "0 0 0");

    return { parseVector3(xyz), parseVector3(rpy) };
}

// ----------------------------------------------------------------------------
std::string
URDFParser::getRequiredAttribute(tinyxml2::XMLElement const* p_element,
                                 const char* p_attr_name) const
{
    const char* attr = p_element->Attribute(p_attr_name);
    return attr ? std::string(attr) : std::string();
}

// ----------------------------------------------------------------------------
std::string
URDFParser::getAttributeOrDefault(tinyxml2::XMLElement const* p_element,
                                  const char* p_attr_name,
                                  const std::string& p_default) const
{
    const char* attr = p_element->Attribute(p_attr_name);
    return attr ? std::string(attr) : p_default;
}

// ----------------------------------------------------------------------------
Eigen::Vector3d URDFParser::parseVector3(const std::string& p_str) const
{
    std::istringstream iss(p_str);
    double x, y, z;
    iss >> x >> y >> z;
    return Eigen::Vector3d(x, y, z);
}

// ----------------------------------------------------------------------------
Eigen::Vector4d URDFParser::parseVector4(const std::string& p_str) const
{
    std::istringstream iss(p_str);
    double x, y, z, w;
    iss >> x >> y >> z >> w;
    return Eigen::Vector4d(x, y, z, w);
}

// ----------------------------------------------------------------------------
std::string URDFParser::findRootLinkName() const
{
    // Find the link that has no parent joint
    for (const auto& [name, link] : m_links)
    {
        if (link->parent_joint == nullptr)
        {
            return name;
        }
    }
    return {};
}

// ----------------------------------------------------------------------------
bool URDFParser::buildSceneGraph()
{
    // Find the root link (link with no parent joint)
    std::string root_link_name = findRootLinkName();
    if (root_link_name.empty())
    {
        m_error = "Could not find root link (link with no parent joint)";
        return false;
    }

    // Find the root link in the links map
    auto root_link_it = m_links.find(root_link_name);
    if (root_link_it == m_links.end())
    {
        m_error = "Root link '" + root_link_name + "' not found in links map";
        return false;
    }

    // Create the root link node
    auto root_link_node =
        std::make_unique<robotik::Link>(root_link_it->second->name,
                                        root_link_it->second->geometry,
                                        root_link_it->second->inertial);

    // Build the joint tree starting from the root link
    for (Joint* child_joint : root_link_it->second->child_joints)
    {
        auto joint_tree = buildJointTree(child_joint);
        if (joint_tree)
        {
            root_link_node->addChild(std::move(joint_tree));
        }
    }

    // Set the root node in the robot
    m_robot->root(std::move(root_link_node));

    return true;
}

// ----------------------------------------------------------------------------
std::unique_ptr<Joint> URDFParser::buildJointTree(Joint* current_joint)
{
    if (!current_joint)
        return nullptr;

    // Find the joint in our map
    auto joint_it =
        std::find_if(m_joints.begin(),
                     m_joints.end(),
                     [current_joint](const auto& pair)
                     { return pair.second.get() == current_joint; });

    if (joint_it == m_joints.end())
        return nullptr;

    // Create a copy of the joint
    auto joint_copy = std::make_unique<Joint>(
        current_joint->name(), current_joint->type(), current_joint->axis());

    // Copy joint properties
    joint_copy->localTransform(current_joint->localTransform());
    joint_copy->limits(current_joint->limits().first,
                       current_joint->limits().second);
    joint_copy->position(current_joint->position());

    // Find the child link for this joint
    std::string child_link_name;
    for (const auto& link_pair : m_links)
    {
        if (link_pair.second->parent_joint == current_joint)
        {
            child_link_name = link_pair.first;
            break;
        }
    }

    if (!child_link_name.empty())
    {
        auto child_link_it = m_links.find(child_link_name);
        if (child_link_it != m_links.end())
        {
            // Create the child link node
            auto child_link_node = std::make_unique<robotik::Link>(
                child_link_it->second->name,
                child_link_it->second->geometry,
                child_link_it->second->inertial);

            // Recursively build the subtree for the child joints
            for (Joint* child_joint : child_link_it->second->child_joints)
            {
                auto child_joint_tree = buildJointTree(child_joint);
                if (child_joint_tree)
                {
                    child_link_node->addChild(std::move(child_joint_tree));
                }
            }

            // Add the child link to the joint
            joint_copy->addChild(std::move(child_link_node));
        }
    }

    return joint_copy;
}

} // namespace robotik