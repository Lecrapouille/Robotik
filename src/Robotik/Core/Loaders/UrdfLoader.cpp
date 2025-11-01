/**
 * @file Parser.cpp
 * @brief Parser for URDF files to create a robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Loaders/UrdfLoader.hpp"

#include "Robotik/Core/Common/Conversions.hpp"
#include "Robotik/Core/Robot/Robot.hpp"

#include <pugixml.hpp>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

namespace robotik
{

// ------------------------------------------------------------------------
//! \brief Private Link data extracted from the URDF file. Used temporarily
//! to be converted into the definitive robotik::Link object in the method
//! buildSceneGraph().
// ------------------------------------------------------------------------
struct URDFLoaderLink
{
    explicit URDFLoaderLink(std::string const& p_name) : name(p_name) {}

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

namespace
{

// ----------------------------------------------------------------------------
//! \brief Parse vector from string "x y z" or "r p y" etc.
//! \param p_str String to parse.
//! \return Vector.
// ----------------------------------------------------------------------------
template <int N>
Eigen::Matrix<double, N, 1> parseVector(const std::string& p_str)
{
    std::istringstream iss(p_str);
    Eigen::Matrix<double, N, 1> vec;
    for (int i = 0; i < N; ++i)
    {
        iss >> vec[i] || (vec[i] = 0.0, false);
    }
    return vec;
}

// ----------------------------------------------------------------------------
//! \brief Parse 3D vector from XML attribute with default.
//! \param node XML node to parse.
//! \param attr Attribute name to parse.
//! \param def Default value if attribute is not found.
//! \return 3D vector.
// ----------------------------------------------------------------------------
Eigen::Vector3d
parseVec3(pugi::xml_node node, const char* attr, const char* def = "0 0 0")
{
    return parseVector<3>(node.attribute(attr).as_string(def));
}

// ----------------------------------------------------------------------------
//! \brief Parse joint type from string.
//! \param type String representing the joint type.
//! \return Joint type enum value.
// ----------------------------------------------------------------------------
Joint::Type parseJointType(const std::string& type)
{
    if (type == "revolute")
        return Joint::Type::REVOLUTE;
    if (type == "continuous")
        return Joint::Type::CONTINUOUS;
    if (type == "prismatic")
        return Joint::Type::PRISMATIC;
    if (type == "fixed")
        return Joint::Type::FIXED;

    std::cerr << "Unsupported joint type: " << type << std::endl;
    return Joint::Type::FIXED;
}

} // anonymous namespace

// ----------------------------------------------------------------------------
URDFLoader::URDFLoader() = default;

// ----------------------------------------------------------------------------
URDFLoader::~URDFLoader() = default;

// ----------------------------------------------------------------------------
std::unique_ptr<Robot> URDFLoader::load(const std::string& p_filename)
{
    // Open the file
    if (std::ifstream file(p_filename); !file)
    {
        m_error = "Failed opening '" + p_filename + "'. Reason was '" +
                  strerror(errno) + "'";
        return nullptr;
    }

    // Load the file
    pugi::xml_document xml;
    if (pugi::xml_parse_result result = xml.load_file(p_filename.c_str());
        !result)
    {
        m_error =
            "Failed to parse URDF XML: " + std::string(result.description());
        return nullptr;
    }

    // Find the <robot> element
    auto robot_element = xml.child("robot");
    if (!robot_element)
    {
        m_error = "No <robot> element found in URDF";
        return nullptr;
    }

    // Get the robot name
    std::string robot_name = robot_element.attribute("name").as_string();

    // Parse the links
    if (!parseLinks(robot_element))
    {
        m_error = "Failed to parse links in URDF";
        return nullptr;
    }

    // Parse the joints
    if (!parseJoints(robot_element))
    {
        m_error = "Failed to parse joints in URDF";
        return nullptr;
    }

    // Build the flat array robot model (new architecture)
    return buildFlatArrays(robot_name);
}

// ----------------------------------------------------------------------------
bool URDFLoader::parseLinks(pugi::xml_node p_robot_element)
{
    for (auto link_node : p_robot_element.children("link"))
    {
        // Get the link name
        std::string name = link_node.attribute("name").as_string();
        auto link = std::make_unique<URDFLoaderLink>(name);

        // Parse the visual and collision geometries
        if (!parseLinkGeometries(link_node, *link))
        {
            m_error = "Failed to parse link geometries for link '" + name + "'";
            return false;
        }

        // Parse the inertial properties
        if (auto inertial = link_node.child("inertial"))
        {
            // Get the mass
            link->inertial.mass =
                inertial.child("mass").attribute("value").as_double(0.0);

            // Get the center of mass
            link->inertial.center_of_mass =
                parseVec3(inertial.child("origin"), "xyz");

            // Get the inertia matrix
            if (auto I = inertial.child("inertia"))
            {
                double ixx = I.attribute("ixx").as_double(0.0);
                double ixy = I.attribute("ixy").as_double(0.0);
                double ixz = I.attribute("ixz").as_double(0.0);
                double iyy = I.attribute("iyy").as_double(0.0);
                double iyz = I.attribute("iyz").as_double(0.0);
                double izz = I.attribute("izz").as_double(0.0);
                link->inertial.inertia_matrix << ixx, ixy, ixz, ixy, iyy, iyz,
                    ixz, iyz, izz;
            }
        }

        m_links[name] = std::move(link);
    }

    return !m_links.empty();
}

// ----------------------------------------------------------------------------
bool URDFLoader::parseJoints(pugi::xml_node p_robot_element)
{
    // Give a unique index to each joint
    size_t id = 0;

    for (auto joint_node : p_robot_element.children("joint"))
    {
        // Get the joint name
        std::string name = joint_node.attribute("name").as_string();

        // Get the joint type
        Joint::Type type =
            parseJointType(joint_node.attribute("type").as_string());

        // Parse axis (default to Z)
        Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
        if (auto axis_node = joint_node.child("axis"))
            axis = parseVec3(axis_node, "xyz", "0 0 1");

        // Parse origin transform
        auto origin = joint_node.child("origin");
        Eigen::Vector3d xyz = parseVec3(origin, "xyz");
        Eigen::Vector3d rpy = parseVec3(origin, "rpy");

        // Create the joint
        auto joint = std::make_unique<Joint>(name, type, axis, id++);

        // Set the local transform
        joint->localTransform(createTransform(xyz, rpy.x(), rpy.y(), rpy.z()));

        // Parse the joint limits (position, velocity, effort)
        if (auto limit = joint_node.child("limit"))
        {
            if (auto lower = limit.attribute("lower");
                auto upper = limit.attribute("upper"))
            {
                joint->limits(lower.as_double(0.0), upper.as_double(0.0));
            }
            if (auto vel = limit.attribute("velocity"))
                joint->maxVelocity(vel.as_double(0.0));
            if (auto eff = limit.attribute("effort"))
                joint->effort_max(eff.as_double(0.0));
        }

        // Parse the joint dynamics (damping, friction)
        if (auto dyn = joint_node.child("dynamics"))
        {
            if (auto damp = dyn.attribute("damping"))
                joint->damping(damp.as_double(0.0));
            if (auto fric = dyn.attribute("friction"))
                joint->friction(fric.as_double(0.0));
        }

        // Get the parent and child link names
        auto parent_name =
            joint_node.child("parent").attribute("link").as_string();
        auto child_name =
            joint_node.child("child").attribute("link").as_string();

        // Find the parent and child links
        if (auto parent = m_links.find(parent_name); parent != m_links.end())
        {
            if (auto child = m_links.find(child_name); child != m_links.end())
            {
                parent->second->child_joints.push_back(joint.get());
                child->second->parent_joint = joint.get();
            }
        }

        m_joints[name] = std::move(joint);
    }

    return !m_joints.empty();
}

// ----------------------------------------------------------------------------
std::unique_ptr<Robot>
URDFLoader::buildFlatArrays(std::string const& p_robot_name)
{
    // Find root link (link with no parent joint)
    URDFLoaderLink* root_link = nullptr;
    for (const auto& [name, link] : m_links)
    {
        if (!link->parent_joint)
        {
            root_link = link.get();
            break;
        }
    }

    if (!root_link)
    {
        m_error = "Could not find root link (link with no parent joint)";
        return nullptr;
    }

    // Prepare flat arrays
    std::vector<JointData> joint_data;
    std::vector<LinkData> link_data;
    std::vector<GeometryData> geometry_data;
    std::unordered_map<std::string, size_t> link_name_to_index;

    // Build flat arrays recursively starting from root
    buildFlatArraysRecursive(root_link,
                             SIZE_MAX, // Root link has no parent joint
                             joint_data,
                             link_data,
                             geometry_data,
                             link_name_to_index);

    // Create Blueprint from flat arrays
    Blueprint blueprint(
        std::move(joint_data), std::move(link_data), std::move(geometry_data));

    // Create and return Robot
    return std::make_unique<Robot>(p_robot_name, std::move(blueprint));
}

// ----------------------------------------------------------------------------
void URDFLoader::buildFlatArraysRecursive(
    URDFLoaderLink const* p_link,
    size_t p_parent_joint_index,
    std::vector<JointData>& p_joint_data,
    std::vector<LinkData>& p_link_data,
    std::vector<GeometryData>& p_geometry_data,
    std::unordered_map<std::string, size_t>& p_link_name_to_index)
{
    // Add this link to link_data
    LinkData link_data;
    link_data.name = p_link->name;
    link_data.parent_joint_index = p_parent_joint_index;
    link_data.placement = Transform::Identity(); // Links have identity
                                                 // placement relative to joint
    link_data.inertial = p_link->inertial;

    size_t link_index = p_link_data.size();
    p_link_name_to_index[p_link->name] = link_index;
    p_link_data.push_back(link_data);

    // Add geometry for this link
    if (p_link->geometry)
    {
        // Cast to Geometry to access geometry-specific methods
        Geometry* geom = dynamic_cast<Geometry*>(p_link->geometry.get());
        if (geom)
        {
            GeometryData geom_data;
            geom_data.name = p_link->name + "_visual";
            geom_data.parent_link_index = link_index;
            geom_data.local_transform =
                geom->localTransform(); // Use geometry's local transform from
                                        // URDF <origin>
            geom_data.type = geom->type();
            geom_data.parameters = geom->parameters();
            geom_data.mesh_path = geom->meshPath();
            geom_data.color = p_link->color;

            p_geometry_data.push_back(geom_data);
        }
    }

    // Process child joints
    for (Joint const* child_joint : p_link->child_joints)
    {
        // Skip FIXED joints (they're not actuable)
        if (child_joint->type() == Joint::Type::FIXED)
            continue;

        // Add joint to joint_data
        JointData joint_data;
        joint_data.name = child_joint->name();
        joint_data.type = child_joint->type();
        joint_data.parent_link_index =
            link_index; // Joint placement is relative to parent link
        joint_data.placement =
            child_joint->localTransform(); // Joint placement from URDF <origin>
        joint_data.axis = child_joint->axis();
        auto limits = child_joint->limits();
        joint_data.position_min = limits.first;
        joint_data.position_max = limits.second;
        joint_data.velocity_max = child_joint->maxVelocity();
        joint_data.effort_max = child_joint->effort_max();
        joint_data.damping = child_joint->damping();
        joint_data.friction = child_joint->friction();

        size_t joint_index = p_joint_data.size();
        p_joint_data.push_back(joint_data);

        // Find child link of this joint
        URDFLoaderLink* child_link = nullptr;
        for (const auto& [name, link] : m_links)
        {
            if (link->parent_joint == child_joint)
            {
                child_link = link.get();
                break;
            }
        }

        // Recursively process child link
        if (child_link)
        {
            buildFlatArraysRecursive(
                child_link,
                joint_index, // This joint is now the parent
                p_joint_data,
                p_link_data,
                p_geometry_data,
                p_link_name_to_index);
        }
    }
}

// ----------------------------------------------------------------------------
// LEGACY METHODS (kept for reference, not used anymore)
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
bool URDFLoader::parseLinkGeometries(pugi::xml_node p_link_element,
                                     URDFLoaderLink& p_link) const
{
    // Parse visual geometry
    if (auto visual = p_link_element.child("visual"))
    {
        if (auto geom = visual.child("geometry"))
        {
            // Parse the visual geometry
            p_link.geometry = parseGeometry(geom, p_link.name + "_geometry");
            if (!p_link.geometry)
            {
                return false;
            }

            // Parse origin and apply transform
            if (auto origin = visual.child("origin"))
            {
                auto xyz = parseVec3(origin, "xyz");
                auto rpy = parseVec3(origin, "rpy");
                p_link.geometry->localTransform(
                    createTransform(xyz, rpy.x(), rpy.y(), rpy.z()));
            }

            // Parse color from material
            if (auto mat = visual.child("material"))
            {
                if (auto color = mat.child("color"))
                {
                    auto rgba = color.attribute("rgba").as_string();
                    if (rgba && rgba[0] != '\0')
                        p_link.color =
                            parseVector<4>(rgba).head<3>().cast<float>();
                }
            }

            // Apply color and set default collision from visual
            if (auto* g = dynamic_cast<Geometry*>(p_link.geometry.get()))
            {
                g->color = p_link.color;

                if (g->type() == Geometry::Type::MESH)
                {
                    p_link.mesh_path_for_obb = g->meshPath();
                }
                else
                {
                    p_link.urdf_collision_center = Eigen::Vector3d::Zero();
                    p_link.urdf_collision_orientation =
                        Eigen::Matrix3d::Identity();
                    // Convert to half-extents for collision (box needs /2)
                    auto params = g->parameters();
                    if (g->type() == Geometry::Type::BOX && params.size() >= 3)
                    {
                        p_link.urdf_collision_params = { params[0] / 2.0,
                                                         params[1] / 2.0,
                                                         params[2] / 2.0 };
                    }
                    else
                    {
                        p_link.urdf_collision_params = params;
                    }
                }
            }
        }
    }

    // Parse collision (overrides visual defaults)
    if (auto collision = p_link_element.child("collision"))
    {
        // Parse collision origin
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();

        if (auto origin = collision.child("origin"))
        {
            center = parseVec3(origin, "xyz");
            auto rpy = parseVec3(origin, "rpy");
            orientation =
                createTransform(
                    Eigen::Vector3d::Zero(), rpy.x(), rpy.y(), rpy.z())
                    .block<3, 3>(0, 0);
        }

        // Parse collision geometry
        if (auto geom = collision.child("geometry"))
        {
            if (auto box = geom.child("box"))
            {
                auto size = parseVec3(box, "size");
                p_link.urdf_collision_center = center;
                p_link.urdf_collision_orientation = orientation;
                p_link.urdf_collision_params = { size.x() / 2.0,
                                                 size.y() / 2.0,
                                                 size.z() / 2.0 };
            }
            else if (auto cyl = geom.child("cylinder"))
            {
                p_link.urdf_collision_center = center;
                p_link.urdf_collision_orientation = orientation;
                p_link.urdf_collision_params = {
                    cyl.attribute("radius").as_double(0.1),
                    cyl.attribute("length").as_double(0.2)
                };
            }
            else if (auto sph = geom.child("sphere"))
            {
                p_link.urdf_collision_center = center;
                p_link.urdf_collision_orientation = orientation;
                p_link.urdf_collision_params = {
                    sph.attribute("radius").as_double(0.1)
                };
            }
            else if (auto mesh = geom.child("mesh"))
            {
                std::string path = mesh.attribute("filename").as_string("");
                if (!path.empty())
                {
                    p_link.mesh_path_for_obb = path;
                }
            }
        }
    }

    return true;
}

// ----------------------------------------------------------------------------
std::unique_ptr<Geometry>
URDFLoader::parseGeometry(pugi::xml_node p_geometry_element,
                          const std::string& p_name) const
{
    if (!p_geometry_element)
        return nullptr;

    if (auto box = p_geometry_element.child("box"))
    {
        auto size = parseVec3(box, "size");
        return std::make_unique<Box>(p_name, size.x(), size.y(), size.z());
    }
    if (auto cyl = p_geometry_element.child("cylinder"))
    {
        return std::make_unique<Cylinder>(
            p_name,
            cyl.attribute("radius").as_double(0.1),
            cyl.attribute("length").as_double(0.2));
    }
    if (auto sph = p_geometry_element.child("sphere"))
    {
        return std::make_unique<Sphere>(p_name,
                                        sph.attribute("radius").as_double(0.1));
    }
    if (auto mesh = p_geometry_element.child("mesh"))
    {
        return std::make_unique<Mesh>(p_name,
                                      mesh.attribute("filename").as_string(""));
    }

    return nullptr;
}

} // namespace robotik