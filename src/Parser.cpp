#include "Robotik/Parser.hpp"

#include <tinyxml2/tinyxml2.h>

#include <fstream>
#include <sstream>
#include <unordered_map>

namespace robotik
{

// ----------------------------------------------------------------------------
URDFParser::URDFParser()
{
    m_robot = std::make_unique<RobotArm>("URDF_Robot");
}

// ----------------------------------------------------------------------------
std::unique_ptr<RobotArm> URDFParser::load(const std::string& p_filename)
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

    parseLinks(robot_element);
    parseJoints(robot_element);
    if (!setRootAndEndEffector())
        return nullptr;
    setRobotLinks();

    return std::move(m_robot);
}

// ----------------------------------------------------------------------------
void URDFParser::parseLinks(tinyxml2::XMLElement* p_robot_element)
{
    for (auto link_element = p_robot_element->FirstChildElement("link");
         link_element;
         link_element = link_element->NextSiblingElement("link"))
    {
        std::string name = link_element->Attribute("name");
        auto link = std::make_unique<Link>(name);

        // Parse visual geometry if present
        if (auto visual_element = link_element->FirstChildElement("visual"))
        {
            if (auto geometry_element =
                    visual_element->FirstChildElement("geometry"))
            {
                tinyxml2::XMLPrinter printer;
                geometry_element->Accept(&printer);
                link->geometry = parseGeometry(printer.CStr());
            }
        }

        // Parse inertial properties if present
        if (auto inertial_element = link_element->FirstChildElement("inertial"))
        {
            tinyxml2::XMLPrinter printer;
            inertial_element->Accept(&printer);
            link->inertial = parseInertial(printer.CStr());
        }

        m_links[name] = std::move(link);
    }
}

// ----------------------------------------------------------------------------
void URDFParser::parseJoints(tinyxml2::XMLElement* p_robot_element)
{
    for (auto joint_element = p_robot_element->FirstChildElement("joint");
         joint_element;
         joint_element = joint_element->NextSiblingElement("joint"))
    {
        std::string name = joint_element->Attribute("name");
        Joint::Type type = parseJointType(joint_element->Attribute("type"));

        // Parse axis
        Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
        if (auto axis_element = joint_element->FirstChildElement("axis"))
        {
            if (const char* xyz_attr = axis_element->Attribute("xyz"); xyz_attr)
            {
                axis = parseVector3(xyz_attr);
            }
        }

        // Parse origin
        Eigen::Vector3d origin_xyz = Eigen::Vector3d::Zero();
        Eigen::Vector3d origin_rpy = Eigen::Vector3d::Zero();
        if (auto origin_element = joint_element->FirstChildElement("origin"))
        {
            std::string xyz = "0 0 0";
            std::string rpy = "0 0 0";

            if (const char* xyz_attr = origin_element->Attribute("xyz");
                xyz_attr)
            {
                xyz = xyz_attr;
            }
            if (const char* rpy_attr = origin_element->Attribute("rpy");
                rpy_attr)
            {
                rpy = rpy_attr;
            }

            origin_xyz = parseVector3(xyz);
            origin_rpy = parseVector3(rpy);
        }

        // Create the joint with origin parameters
        auto joint =
            std::make_unique<Joint>(name, type, axis, origin_xyz, origin_rpy);

        // Parse limits
        if (auto limit_element = joint_element->FirstChildElement("limit"))
        {
            double lower = 0, upper = 0;
            if (limit_element->QueryDoubleAttribute("lower", &lower) ==
                    tinyxml2::XML_SUCCESS &&
                limit_element->QueryDoubleAttribute("upper", &upper) ==
                    tinyxml2::XML_SUCCESS)
            {
                joint->setLimits(lower, upper);
            }
        }

        // Parse parent and child links
        auto parent_element = joint_element->FirstChildElement("parent");
        auto child_element = joint_element->FirstChildElement("child");
        if (parent_element && child_element)
        {
            const char* parent_name = parent_element->Attribute("link");
            const char* child_name = child_element->Attribute("link");

            if (parent_name && child_name)
            {
                auto parent_it = m_links.find(parent_name);
                auto child_it = m_links.find(child_name);

                if (parent_it != m_links.end() && child_it != m_links.end())
                {
                    parent_it->second->child_joint = joint.get();
                    child_it->second->parent_joint = joint.get();
                }
            }
        }

        m_joints[name] = std::move(joint);
    }
}

// ----------------------------------------------------------------------------
std::pair<Link*, Link const*> URDFParser::findRootAndEndEffector()
{
    std::pair<Link*, Link const*> result(nullptr, nullptr);
    for (auto const& [_, link] : m_links)
    {
        if (link->parent_joint == nullptr)
        {
            result.first = link.get();
        }
        if (link->child_joint == nullptr)
        {
            result.second = link.get();
        }
    }

    return result;
}

// ----------------------------------------------------------------------------
bool URDFParser::setRootAndEndEffector()
{
    auto [root_link, end_effector_link] = findRootAndEndEffector();
    if (!root_link || !end_effector_link)
    {
        m_error = (!root_link ? "No root link found in URDF"
                              : "No end effector link found in URDF");
        return false;
    }

    // Setup the robot: scene graph, set the root node and end effector node
    auto root_node = Node::create<Node>(root_link->name);
    buildSceneGraph(root_node.get(), root_link);

    auto end_effector_node = root_node->getNode(end_effector_link->name);
    if (!end_effector_node)
    {
        m_error = "No end effector node found in URDF";
        return false;
    }

    m_robot->setupRobot(std::move(root_node), *end_effector_node);
    return true;
}

// ----------------------------------------------------------------------------
void URDFParser::buildSceneGraph(Node* p_node, Link* p_link)
{
    Joint* child_joint = p_link->child_joint;
    if (!child_joint)
        return;

    // Find the joint in the joints map and transfer ownership
    auto joint_it = m_joints.find(child_joint->getName());
    if (joint_it == m_joints.end())
        return;

    p_node->addChild(std::move(joint_it->second));

    // Find the child link connected to this joint
    for (auto const& [link_name, link_ptr] : m_links)
    {
        if (link_ptr->parent_joint == child_joint)
        {
            auto child_node = std::make_unique<Node>(link_name);
            Node* child_node_ptr = child_node.get();

            child_joint->addChild(std::move(child_node));
            buildSceneGraph(child_node_ptr, link_ptr.get());
            break;
        }
    }
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
Transform URDFParser::parseOrigin(const std::string& p_xyz,
                                  const std::string& p_rpy) const
{
    Eigen::Vector3d translation = parseVector3(p_xyz);
    Eigen::Vector3d rotation = parseVector3(p_rpy);
    return utils::createTransform(
        translation, rotation.x(), rotation.y(), rotation.z());
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
Joint::Type URDFParser::parseJointType(const std::string_view& p_str_type) const
{
    if (p_str_type == "revolute")
        return Joint::Type::REVOLUTE;
    else if (p_str_type == "prismatic")
        return Joint::Type::PRISMATIC;
    else
        return Joint::Type::FIXED;
}

// ----------------------------------------------------------------------------
void URDFParser::setRobotLinks()
{
    for (auto& [name, link] : m_links)
    {
        m_robot->addLink(name, std::move(link));
    }
}

} // namespace robotik