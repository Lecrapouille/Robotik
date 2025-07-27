#include "Robotik/Parser.hpp"
#include "Robotik/private/Conversions.hpp"
#include "Robotik/private/Geometry.hpp"
#include "Robotik/private/Link.hpp"

#include <tinyxml2/tinyxml2.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <unordered_map>

namespace robotik
{

// ----------------------------------------------------------------------------
//! \brief Iterate over all child elements of a parent element.
//! \param p_parent The parent element.
//! \param p_child_name The name of the child elements to iterate over.
//! \param p_func The function to call for each child element.
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
    if (p_str_type == "revolute" || p_str_type == "continuous")
        return Joint::Type::REVOLUTE;
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
static std::string xmlElementToString(tinyxml2::XMLElement const* p_element)
{
    tinyxml2::XMLPrinter printer;
    p_element->Accept(&printer);
    return printer.CStr();
}

// ----------------------------------------------------------------------------
std::unique_ptr<tinyxml2::XMLDocument>
parseXMLFromString(const std::string& p_xml)
{
    auto doc = std::make_unique<tinyxml2::XMLDocument>();
    if (doc->Parse(p_xml.c_str()) != tinyxml2::XML_SUCCESS)
        return nullptr;
    return doc;
}

// ----------------------------------------------------------------------------
static void queryDoubleAttribute(tinyxml2::XMLElement* p_parent,
                                 const char* p_child_name,
                                 const char* p_attr_name,
                                 double& p_value)
{
    if (auto element = p_parent->FirstChildElement(p_child_name))
    {
        element->QueryDoubleAttribute(p_attr_name, &p_value);
    }
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

    std::string robot_name = robot_element->Attribute("name");

    parseLinks(robot_element);
    parseJoints(robot_element);

    if (m_links.empty() || m_joints.empty())
    {
        m_error = "No links or joints found in URDF";
        return nullptr;
    }

    return buildSceneGraph(robot_name);
}

// ----------------------------------------------------------------------------
void URDFParser::parseLinks(tinyxml2::XMLElement* p_robot_element)
{
    forEachChildElement( //
        p_robot_element,
        "link",
        [this](auto* xml)
        {
            std::string name = xml->Attribute("name");
            auto link = std::make_unique<URDFParser::Link>(name);

            parseVisualProperties(xml, *link);
            parseCollisionProperties(xml, *link);
            parseInertialProperties(xml, *link);

            m_links[name] = std::move(link);
        });
}

// ----------------------------------------------------------------------------
void URDFParser::parseJoints(tinyxml2::XMLElement* p_robot_element)
{
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
            joint->localTransform(origin_xyz, origin_rpy);

            parseLimits(xml, *joint);
            parseParentChildLinks(xml, *joint);

            m_joints[name] = std::move(joint);
        });
}

// ----------------------------------------------------------------------------
void URDFParser::parseInertialProperties(tinyxml2::XMLElement* p_link_element,
                                         URDFParser::Link& p_link) const
{
    if (auto inertial_element = p_link_element->FirstChildElement("inertial"))
    {
        p_link.inertial = parseInertial(xmlElementToString(inertial_element));
    }
}

// ----------------------------------------------------------------------------
Inertial URDFParser::parseInertial(const std::string& p_xml) const
{
    Inertial inert;
    auto doc = parseXMLFromString(p_xml);
    if (!doc)
        return inert;

    auto inertialElement = doc->FirstChildElement("inertial");
    if (!inertialElement)
        return inert;

    queryDoubleAttribute(inertialElement, "mass", "value", inert.mass);

    if (auto originElement = inertialElement->FirstChildElement("origin"))
    {
        inert.center_of_mass = parseOptionalVector3(originElement, "xyz");
    }

    if (auto inertiaElement = inertialElement->FirstChildElement("inertia"))
    {
        double ixx = 0, ixy = 0, ixz = 0, iyy = 0, iyz = 0, izz = 0;
        auto queryInertia = [&](const char* attr, double& val)
        { inertiaElement->QueryDoubleAttribute(attr, &val); };

        queryInertia("ixx", ixx);
        queryInertia("ixy", ixy);
        queryInertia("ixz", ixz);
        queryInertia("iyy", iyy);
        queryInertia("iyz", iyz);
        queryInertia("izz", izz);

        inert.inertia_matrix << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;
    }

    return inert;
}

// ----------------------------------------------------------------------------
void URDFParser::parseVisualProperties(tinyxml2::XMLElement* p_link_element,
                                       URDFParser::Link& p_link) const
{
    auto visual_element = p_link_element->FirstChildElement("visual");
    if (!visual_element)
        return;

    if (auto geometry_element = visual_element->FirstChildElement("geometry"))
    {
        // Create geometry with "_visual" suffix
        std::string visual_name = p_link.name + "_visual";
        p_link.geometry =
            parseGeometry(xmlElementToString(geometry_element), visual_name);

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
                                          URDFParser::Link& p_link) const
{
    auto collision_element = p_link_element->FirstChildElement("collision");
    if (!collision_element)
        return;

    if (auto geometry_element =
            collision_element->FirstChildElement("geometry"))
    {
        // Create collision geometry with "_collision" suffix
        std::string collision_name = p_link.name + "_collision";
        p_link.collision =
            parseGeometry(xmlElementToString(geometry_element), collision_name);
    }
}

// ----------------------------------------------------------------------------
std::unique_ptr<Geometry>
URDFParser::parseGeometry(const std::string& p_xml,
                          const std::string& p_name) const
{
    auto doc = parseXMLFromString(p_xml);
    if (!doc)
        return nullptr;

    auto geometryElement = doc->FirstChildElement("geometry");
    if (!geometryElement)
        return nullptr;

    if (auto boxElement = geometryElement->FirstChildElement("box"))
    {
        auto size = parseOptionalVector3(boxElement, "size");
        auto params = std::vector<double>{ size.x(), size.y(), size.z() };
        return std::make_unique<robotik::Box>(p_name, std::move(params));
    }
    else if (auto cylinderElement =
                 geometryElement->FirstChildElement("cylinder"))
    {
        double radius = 0.1, length = 0.2;
        cylinderElement->QueryDoubleAttribute("radius", &radius);
        cylinderElement->QueryDoubleAttribute("length", &length);
        auto params = std::vector<double>{ radius, length };
        return std::make_unique<robotik::Cylinder>(p_name, std::move(params));
    }
    else if (auto sphereElement = geometryElement->FirstChildElement("sphere"))
    {
        double radius = 0.1;
        sphereElement->QueryDoubleAttribute("radius", &radius);
        auto params = std::vector<double>{ radius };
        return std::make_unique<robotik::Sphere>(p_name, std::move(params));
    }
    else if (auto meshElement = geometryElement->FirstChildElement("mesh"))
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
                               URDFParser::Link& p_link) const
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
URDFParser::createLinkFromURDFData(URDFParser::Link& p_urdf_link) const
{
    // Transformations and colors are now applied directly in
    // parseVisualProperties
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

// ----------------------------------------------------------------------------
std::unique_ptr<Robot>
URDFParser::buildSceneGraph(std::string_view p_robot_name)
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
URDFParser::Link* URDFParser::findRootLink() const
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
    //
    auto joint_it =
        std::find_if(m_joints.begin(),
                     m_joints.end(),
                     [p_current_joint](const auto& pair)
                     { return pair.second.get() == p_current_joint; });

    if (joint_it == m_joints.end())
        return nullptr;

    auto joint_copy = std::make_unique<Joint>(p_current_joint->name(),
                                              p_current_joint->type(),
                                              p_current_joint->axis());

    joint_copy->localTransform(p_current_joint->localTransform());
    joint_copy->limits(p_current_joint->limits().first,
                       p_current_joint->limits().second);
    joint_copy->position(p_current_joint->position());

    std::string child_link_name;
    for (const auto& [name, link] : m_links)
    {
        if (link->parent_joint == p_current_joint)
        {
            child_link_name = name;
            break;
        }
    }

    if (!child_link_name.empty())
    {
        auto child_link_it = m_links.find(child_link_name);
        if (child_link_it != m_links.end())
        {
            auto child_link_node =
                createLinkFromURDFData(*child_link_it->second);
            if (!child_link_node)
            {
                m_error = "Failed to create child link node";
                return nullptr;
            }

            for (Joint const* child_joint : child_link_it->second->child_joints)
            {
                if (auto child_joint_tree = buildJointTree(child_joint))
                {
                    child_link_node->addChild(std::move(child_joint_tree));
                }
            }

            joint_copy->addChild(std::move(child_link_node));
        }
    }

    return joint_copy;
}

} // namespace robotik