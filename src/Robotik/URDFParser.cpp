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
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
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
    explicit URDFParserLink(std::string const& p_name) : name(p_name) {}

    //! \brief Name of the link
    std::string name;
    //! \brief Geometry of the link. Shall not be null.
    Geometry::Ptr geometry;
    //! \brief Color of the link
    Eigen::Vector3f color = Eigen::Vector3f(0.5, 0.5, 0.5);
    //! \brief Inertial properties of the link
    Inertial inertial;
    //! \brief Parent joint of the link
    Joint* parent_joint = nullptr;
    //! \brief Child joints of the link
    std::vector<Joint*> child_joints;
    //! \brief Collision data (computed from visual geometry or overridden by
    //! URDF collision)
    std::vector<double> urdf_collision_params;
    //! \brief Center of the collision volume in local frame
    Eigen::Vector3d urdf_collision_center = Eigen::Vector3d::Zero();
    //! \brief Orientation of the collision volume in local frame
    Eigen::Matrix3d urdf_collision_orientation = Eigen::Matrix3d::Identity();
    //! \brief Mesh path for OBB computation (collision mesh if exists, else
    //! visual mesh)
    std::string mesh_path_for_obb;
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
static Joint::Type jointType(std::string const& p_str_type)
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
// FIXME mettre template pour Robot afin de faire load<DerivedRobot>
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

    size_t index = 0;

    // Parse the joints.
    forEachChildElement( //
        p_robot_element,
        "joint",
        [this, &index](auto* xml)
        {
            std::string name = xml->Attribute("name");
            Joint::Type type = jointType(xml->Attribute("type"));

            Eigen::Vector3d axis = parseAxis(xml);
            auto [origin_xyz, origin_rpy] = parseOriginTransform(xml);
            auto joint = std::make_unique<Joint>(name, type, axis, index++);
            joint->localTransform(utils::createTransform(
                origin_xyz, origin_rpy.x(), origin_rpy.y(), origin_rpy.z()));

            parseLimits(xml, *joint);
            parseDynamics(xml, *joint);
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
    auto root_node = createLinkFromURDFData(*root_link, m_error);
    if (!root_node)
    {
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
    auto robot = std::make_unique<Robot>(p_robot_name, std::move(root_node));

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
    auto child_link_node = createLinkFromURDFData(*child_link, m_error);
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
        // Create geometry with "_visual" suffix.
        std::string visual_name = p_link.name + "_geometry";
        p_link.geometry = parseGeometry(geometry_element, visual_name);

        // Apply visual transform and color directly.
        if (p_link.geometry)
        {
            Transform visual_origin = parseOriginFromElement(visual_element);
            p_link.geometry->localTransform(visual_origin);

            // Parse and apply material color
            parseMaterial(visual_element, p_link);

            // Store collision data from visual geometry.
            // Will be overridden if URDF provides explicit collision geometry.
            if (auto* geom = dynamic_cast<Geometry*>(p_link.geometry.get()))
            {
                geom->color = p_link.color;

                if (geom->type() == Geometry::Type::MESH)
                {
                    // Store mesh path for later OBB computation
                    // (may be overridden by collision mesh)
                    if (p_link.mesh_path_for_obb.empty())
                    {
                        p_link.mesh_path_for_obb = geom->meshPath();
                    }
                }
                else
                {
                    // For non-mesh geometry, use parameters directly
                    p_link.urdf_collision_center = Eigen::Vector3d::Zero();
                    p_link.urdf_collision_orientation =
                        Eigen::Matrix3d::Identity();
                    p_link.urdf_collision_params = geom->parameters();
                }
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

    // Parse collision origin (center and orientation)
    Eigen::Vector3d collision_center = Eigen::Vector3d::Zero();
    Eigen::Matrix3d collision_orientation = Eigen::Matrix3d::Identity();

    auto origin_element = collision_element->FirstChildElement("origin");
    if (origin_element)
    {
        collision_center = parseOptionalVector3(origin_element, "xyz");
        Eigen::Vector3d rpy = parseOptionalVector3(origin_element, "rpy");
        collision_orientation =
            utils::createTransform(
                Eigen::Vector3d::Zero(), rpy.x(), rpy.y(), rpy.z())
                .block<3, 3>(0, 0);
    }

    // Parse collision geometry type and parameters (overrides default from
    // visual)
    if (auto geometry_element =
            collision_element->FirstChildElement("geometry"))
    {
        if (auto boxElement = geometry_element->FirstChildElement("box"))
        {
            auto size = parseOptionalVector3(boxElement, "size");
            p_link.urdf_collision_center = collision_center;
            p_link.urdf_collision_orientation = collision_orientation;
            p_link.urdf_collision_params = { size.x() / 2.0,
                                             size.y() / 2.0,
                                             size.z() / 2.0 };
        }
        else if (auto cylinderElement =
                     geometry_element->FirstChildElement("cylinder"))
        {
            double radius = 0.1, length = 0.2;
            cylinderElement->QueryDoubleAttribute("radius", &radius);
            cylinderElement->QueryDoubleAttribute("length", &length);
            p_link.urdf_collision_center = collision_center;
            p_link.urdf_collision_orientation = collision_orientation;
            p_link.urdf_collision_params = { radius, length };
        }
        else if (auto sphereElement =
                     geometry_element->FirstChildElement("sphere"))
        {
            double radius = 0.1;
            sphereElement->QueryDoubleAttribute("radius", &radius);
            p_link.urdf_collision_center = collision_center;
            p_link.urdf_collision_orientation = collision_orientation;
            p_link.urdf_collision_params = { radius };
        }
        else if (auto meshElement = geometry_element->FirstChildElement("mesh"))
        {
            // Store collision mesh path (overrides visual mesh for OBB)
            std::string mesh_path =
                getAttributeOrDefault(meshElement, "filename", "");
            if (!mesh_path.empty())
            {
                p_link.mesh_path_for_obb = mesh_path;
            }
        }
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

    // Parse velocity limit
    double velocity = 0;
    if (limit_element->QueryDoubleAttribute("velocity", &velocity) ==
        tinyxml2::XML_SUCCESS)
    {
        p_joint.maxVelocity(velocity);
    }

    // Parse effort limit
    double effort = 0;
    if (limit_element->QueryDoubleAttribute("effort", &effort) ==
        tinyxml2::XML_SUCCESS)
    {
        p_joint.effort_max(effort);
    }
}

// ----------------------------------------------------------------------------
void URDFParser::parseDynamics(tinyxml2::XMLElement* p_joint_element,
                               Joint& p_joint) const
{
    auto dynamics_element = p_joint_element->FirstChildElement("dynamics");
    if (!dynamics_element)
        return;

    // Parse damping
    double damping = 0.0;
    if (dynamics_element->QueryDoubleAttribute("damping", &damping) ==
        tinyxml2::XML_SUCCESS)
    {
        p_joint.damping(damping);
    }

    // Parse friction
    double friction = 0.0;
    if (dynamics_element->QueryDoubleAttribute("friction", &friction) ==
        tinyxml2::XML_SUCCESS)
    {
        p_joint.friction(friction);
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
URDFParser::createLinkFromURDFData(URDFParserLink& p_urdf_link,
                                   std::string& p_error) const
{
    // We need at least a visual geometry to create a link
    if (!p_urdf_link.geometry)
    {
        p_error = "We need at least a visual geometry to create a link for " +
                  p_urdf_link.name;
        return nullptr;
    }

#if 0
    // Compute OBB from mesh
    if (!p_urdf_link.mesh_path_for_obb.empty())
    {
        if (!computeOBBFromMesh(p_urdf_link.mesh_path_for_obb,
                                p_urdf_link.urdf_collision_center,
                                p_urdf_link.urdf_collision_orientation,
                                p_urdf_link.urdf_collision_params))
        {
            p_error = "Failed to compute OBB from mesh for " + p_urdf_link.name;
            return nullptr;
        }
    }
#endif

    // Create link with visual geometry and collision data
    auto link = Node::create<robotik::Link>(p_urdf_link.name,
                                            std::move(p_urdf_link.geometry));
    link->setCollisionData(p_urdf_link.urdf_collision_center,
                           p_urdf_link.urdf_collision_orientation,
                           p_urdf_link.urdf_collision_params);

    // Transfer inertial properties
    link->inertia(p_urdf_link.inertial);

    return link;
}

// ----------------------------------------------------------------------------
bool URDFParser::computeOBBFromMesh(const std::string& p_mesh_path,
                                    Eigen::Vector3d& p_center,
                                    Eigen::Matrix3d& p_orientation,
                                    std::vector<double>& p_params) const
{
    // Parse STL file directly to extract vertices
    std::vector<Eigen::Vector3d> positions;
    if (!parseSTLVertices(p_mesh_path, positions))
    {
        // Failed to load mesh, use default bounding box
        p_center = Eigen::Vector3d::Zero();
        p_orientation = Eigen::Matrix3d::Identity();
        p_params = { 1.0, 1.0, 1.0 };
        return false;
    }

    computeOBBFromVertices(positions, p_center, p_orientation, p_params);
    return true;
}

// ----------------------------------------------------------------------------
void URDFParser::computeOBBFromVertices(
    const std::vector<Eigen::Vector3d>& p_positions,
    Eigen::Vector3d& p_center,
    Eigen::Matrix3d& p_orientation,
    std::vector<double>& p_params) const
{
    if (p_positions.empty())
    {
        p_center = Eigen::Vector3d::Zero();
        p_orientation = Eigen::Matrix3d::Identity();
        p_params = { 1.0, 1.0, 1.0 };
        return;
    }

    // Compute centroid
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& pos : p_positions)
    {
        centroid += pos;
    }
    centroid /= static_cast<double>(p_positions.size());

    // Compute covariance matrix for PCA
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto& pos : p_positions)
    {
        Eigen::Vector3d centered = pos - centroid;
        covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(p_positions.size());

    // Eigen decomposition to get principal axes
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance);
    Eigen::Matrix3d eigenvectors = eigen_solver.eigenvectors();

    // Ensure right-handed coordinate system
    if (eigenvectors.determinant() < 0)
    {
        eigenvectors.col(0) = -eigenvectors.col(0);
    }

    // Project vertices onto principal axes and find extents
    Eigen::Vector3d min_extents =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
    Eigen::Vector3d max_extents =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

    for (const auto& pos : p_positions)
    {
        Eigen::Vector3d local_pos = eigenvectors.transpose() * (pos - centroid);
        min_extents = min_extents.cwiseMin(local_pos);
        max_extents = max_extents.cwiseMax(local_pos);
    }

    // Compute half-extents and adjusted center
    Eigen::Vector3d half_extents = (max_extents - min_extents) * 0.5;
    Eigen::Vector3d local_center = (min_extents + max_extents) * 0.5;
    Eigen::Vector3d world_center = centroid + eigenvectors * local_center;

    // Store OBB data
    p_center = world_center;
    p_orientation = eigenvectors;
    p_params = { half_extents.x(), half_extents.y(), half_extents.z() };
}

// ----------------------------------------------------------------------------
bool URDFParser::parseSTLVertices(
    const std::string& p_mesh_path,
    std::vector<Eigen::Vector3d>& p_positions) const
{
    std::ifstream file(p_mesh_path, std::ios::binary);
    if (!file.is_open())
    {
        return false;
    }

    // Check if binary STL (read first 5 bytes)
    char header[5];
    file.read(header, 5);
    file.seekg(0); // Reset to beginning

    bool is_binary = (strncmp(header, "solid", 5) != 0);

    if (is_binary)
    {
        // Binary STL format
        // Skip 80-byte header
        file.seekg(80);

        // Read triangle count
        uint32_t triangle_count;
        file.read(reinterpret_cast<char*>(&triangle_count), sizeof(uint32_t));

        if (triangle_count == 0)
        {
            return false;
        }

        p_positions.reserve(triangle_count * 3);

        for (uint32_t i = 0; i < triangle_count; ++i)
        {
            // Skip normal vector (3 floats)
            file.seekg(3 * sizeof(float), std::ios::cur);

            // Read 3 vertices (9 floats)
            float vertices[9];
            file.read(reinterpret_cast<char*>(vertices), 9 * sizeof(float));

            // Add vertices
            for (int j = 0; j < 3; ++j)
            {
                p_positions.emplace_back(
                    vertices[j * 3], vertices[j * 3 + 1], vertices[j * 3 + 2]);
            }

            // Skip attribute byte count (2 bytes)
            file.seekg(2, std::ios::cur);
        }
    }
    else
    {
        // ASCII STL format
        file.close();
        file.open(p_mesh_path, std::ios::in);
        if (!file.is_open())
        {
            return false;
        }

        std::string line;
        while (std::getline(file, line))
        {
            // Convert to lowercase for easier parsing
            std::string line_lower = line;
            std::transform(line_lower.begin(),
                           line_lower.end(),
                           line_lower.begin(),
                           ::tolower);

            std::istringstream iss(line_lower);
            std::string token;
            iss >> token;

            if (token == "vertex")
            {
                // Read vertex coordinates
                float x, y, z;
                iss >> x >> y >> z;
                p_positions.emplace_back(x, y, z);
            }
        }
    }

    return !p_positions.empty();
}

} // namespace robotik