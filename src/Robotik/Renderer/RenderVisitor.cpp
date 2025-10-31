/**
 * @file RenderVisitor.cpp
 * @brief Implementation of RenderVisitor for rendering robot nodes.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "Robotik/Renderer/RenderVisitor.hpp"

#include "Robotik/Core/Robot/Blueprint/Geometry.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/Blueprint/Link.hpp"
#include "Robotik/Core/Robot/Blueprint/Node.hpp"
#include "Robotik/Renderer/Loaders/StlLoader.hpp"
#include "Robotik/Renderer/Managers/MeshManager.hpp"
#include "Robotik/Renderer/Managers/ShaderManager.hpp"
#include "Robotik/Renderer/Renderer.hpp"

#include <iostream>
#include <sstream>

namespace robotik::renderer
{

using namespace robotik;

// ----------------------------------------------------------------------------
RenderVisitor::RenderVisitor(MeshManager& mesh_mgr,
                             Renderer const& renderer,
                             ShaderManager const& shader_mgr)
    : m_mesh_manager(mesh_mgr),
      m_renderer(renderer),
      m_shader_manager(shader_mgr)
{
}

// ----------------------------------------------------------------------------
void RenderVisitor::visit(const robotik::Joint& joint)
{
    // Optionally render joint axes for debugging
    if (m_show_joint_axes)
    {
        renderJointAxis(joint);
    }
}

// ----------------------------------------------------------------------------
void RenderVisitor::visit(const robotik::Link& /*link*/)
{
    // Links themselves have no geometry to render
    // Their geometry is in separate Geometry child nodes
}

// ----------------------------------------------------------------------------
void RenderVisitor::visit(const robotik::Geometry& geometry)
{
    // Get world transform and render the geometry
    Eigen::Matrix4f transform = geometry.worldTransform().cast<float>();
    renderGeometry(geometry, transform);
}

// ----------------------------------------------------------------------------
void RenderVisitor::visit(const robotik::Sensor& /*sensor*/)
{
    // Not yet implemented
}

// ----------------------------------------------------------------------------
void RenderVisitor::visit(const robotik::Actuator& /*actuator*/)
{
    // Not yet implemented
}

// ----------------------------------------------------------------------------
void RenderVisitor::visit(const robotik::Node& /*node*/)
{
    // Fallback for generic nodes - nothing to render
}

// ----------------------------------------------------------------------------
void RenderVisitor::renderGeometry(robotik::Geometry const& geom,
                                   Eigen::Matrix4f const& transform)
{
    std::string mesh_name = getMeshName(geom);
    const MeshManager::GPUMesh* gpu_mesh = m_mesh_manager.getMesh(mesh_name);

    // Create the mesh if it doesn't exist yet
    if (!gpu_mesh)
    {
        if (auto* box = dynamic_cast<const robotik::Box*>(&geom))
        {
            const auto& params = box->parameters();
            if (params.size() == 3)
            {
                m_mesh_manager.createBox(
                    mesh_name,
                    static_cast<float>(params[0]),  // width
                    static_cast<float>(params[1]),  // height
                    static_cast<float>(params[2])); // depth
            }
        }
        else if (auto* cyl = dynamic_cast<const robotik::Cylinder*>(&geom))
        {
            const auto& params = cyl->parameters();
            if (params.size() == 2)
            {
                m_mesh_manager.createCylinder(
                    mesh_name,
                    static_cast<float>(params[0]), // radius
                    static_cast<float>(params[1]), // length
                    32);                           // segments
            }
        }
        else if (auto* sphere = dynamic_cast<const robotik::Sphere*>(&geom))
        {
            const auto& params = sphere->parameters();
            if (params.size() == 1)
            {
                m_mesh_manager.createSphere(
                    mesh_name,
                    static_cast<float>(params[0]), // radius
                    16,                            // latitude segments
                    16);                           // longitude segments
            }
        }
        else if (auto* mesh = dynamic_cast<const robotik::Mesh*>(&geom))
        {
            STLLoader loader;
            if (!m_mesh_manager.loadFromFile(
                    mesh_name, mesh->meshPath(), loader, false))
            {
                std::cerr << "Failed to load mesh: " << mesh->meshPath()
                          << std::endl;
            }
        }

        gpu_mesh = m_mesh_manager.getMesh(mesh_name);
    }

    // Render if mesh was created/found successfully
    if (gpu_mesh)
    {
        Eigen::Vector3f color = geom.color.cast<float>();
        m_renderer.render(gpu_mesh, transform, color);
    }
}

// ----------------------------------------------------------------------------
std::string RenderVisitor::getMeshName(robotik::Geometry const& geom) const
{
    std::stringstream ss;
    ss << geom.name();

    // Add parameters to make name unique
    for (const auto& param : geom.parameters())
    {
        ss << "_" << param;
    }

    return ss.str();
}

// ----------------------------------------------------------------------------
void RenderVisitor::renderJointAxis(robotik::Joint const& p_joint)
{
    constexpr float axis_length = 0.15f;  // 15 cm
    constexpr float axis_radius = 0.015f; // 1.5 cm

    // Get joint type and check if we should render this type
    robotik::Joint::Type joint_type = p_joint.type();

    if (joint_type == robotik::Joint::Type::REVOLUTE ||
        joint_type == robotik::Joint::Type::CONTINUOUS)
    {
        if (!m_show_revolute_axes)
            return;
    }
    else if (joint_type == robotik::Joint::Type::PRISMATIC)
    {
        if (!m_show_prismatic_axes)
            return;
    }
    else
    {
        // FIXED joints don't have axes to render
        return;
    }

    // Get joint world transform and axis in world space
    Eigen::Matrix4d joint_world_transform = p_joint.worldTransform();
    Eigen::Vector3d joint_axis_local = p_joint.axis();
    Eigen::Vector3d joint_axis_world =
        joint_world_transform.block<3, 3>(0, 0) * joint_axis_local;

    // Get joint position in world space
    Eigen::Vector3d joint_pos = joint_world_transform.block<3, 1>(0, 3);

    // Create mesh name based on joint type
    std::string mesh_name;
    const MeshManager::GPUMesh* gpu_mesh = nullptr;

    if (joint_type == robotik::Joint::Type::REVOLUTE ||
        joint_type == robotik::Joint::Type::CONTINUOUS)
    {
        // Use cylinder for revolute joints
        mesh_name = "joint_axis_cylinder";
        if (!m_mesh_manager.hasMesh(mesh_name))
        {
            m_mesh_manager.createCylinder(
                mesh_name, axis_radius, axis_length, 16);
        }
        gpu_mesh = m_mesh_manager.getMesh(mesh_name);
    }
    else if (joint_type == robotik::Joint::Type::PRISMATIC)
    {
        // Use box for prismatic joints
        mesh_name = "joint_axis_box";
        if (!m_mesh_manager.hasMesh(mesh_name))
        {
            m_mesh_manager.createBox(
                mesh_name, axis_radius * 2.0f, axis_radius * 2.0f, axis_length);
        }
        gpu_mesh = m_mesh_manager.getMesh(mesh_name);
    }

    if (!gpu_mesh || !gpu_mesh->is_loaded)
        return;

    // Create rotation to align Z-axis (default mesh axis) with joint axis
    Eigen::Vector3f z_axis = Eigen::Vector3f::UnitZ();
    Eigen::Vector3f target_axis = joint_axis_world.normalized().cast<float>();

    // Handle case where axis is already aligned with Z
    Eigen::Matrix3f rotation;
    if (target_axis.isApprox(z_axis) || target_axis.isApprox(-z_axis))
    {
        if (target_axis.isApprox(-z_axis))
        {
            // Rotate 180 degrees around X or Y
            rotation = Eigen::AngleAxisf(M_PIf, Eigen::Vector3f::UnitX())
                           .toRotationMatrix();
        }
        else
        {
            rotation = Eigen::Matrix3f::Identity();
        }
    }
    else
    {
        // Create rotation from Z-axis to target axis
        Eigen::Quaternionf quat =
            Eigen::Quaternionf::FromTwoVectors(z_axis, target_axis);
        rotation = quat.toRotationMatrix();
    }

    // Create transformation matrix
    // The mesh is centered at origin with Z from -height/2 to +height/2
    // We position it so it starts at the joint and extends along the axis
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // Set rotation
    transform.block<3, 3>(0, 0) = rotation;

    // Set position: mesh center should be at joint position + half length along
    // axis Since mesh extends from -axis_length/2 to +axis_length/2 along Z,
    // placing it at joint_pos + target_axis * (axis_length * 0.5f) centers it
    // correctly
    Eigen::Vector3f position =
        joint_pos.cast<float>() + target_axis * (axis_length * 0.5f);
    transform.block<3, 1>(0, 3) = position;

    // Render with a distinct color
    Eigen::Vector3f axis_color;
    if (joint_type == robotik::Joint::Type::PRISMATIC)
    {
        // Yellow for prismatic joints
        axis_color = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
    }
    else
    {
        // Cyan for revolute joints
        axis_color = Eigen::Vector3f(0.0f, 1.0f, 1.0f);
    }

    m_renderer.render(gpu_mesh, transform, axis_color);
}

} // namespace robotik::renderer
