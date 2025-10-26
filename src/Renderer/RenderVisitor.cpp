/**
 * @file RenderVisitor.cpp
 * @brief Implementation of RenderVisitor for rendering robot nodes.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "Robotik/Renderer/RenderVisitor.hpp"

#include "Robotik/Core/Geometry.hpp"
#include "Robotik/Core/Joint.hpp"
#include "Robotik/Core/Link.hpp"
#include "Robotik/Core/Node.hpp"
#include "Robotik/Renderer/Loaders/STLLoader.hpp"
#include "Robotik/Renderer/Managers/MeshManager.hpp"
#include "Robotik/Renderer/Managers/ShaderManager.hpp"
#include "Robotik/Renderer/Renderer.hpp"

#include <iostream>
#include <sstream>

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
RenderVisitor::RenderVisitor(MeshManager& mesh_mgr,
                             Renderer& renderer,
                             ShaderManager& shader_mgr)
    : m_mesh_manager(mesh_mgr),
      m_renderer(renderer),
      m_shader_manager(shader_mgr)
{
}

// ----------------------------------------------------------------------------
void RenderVisitor::visit(const robotik::Joint& /*joint*/)
{
    // Optionally render joint axes for debugging
    if (m_show_joint_axes)
    {
        // TODO: Implement joint axes rendering
        // Could use m_renderer.renderAxes() here
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
            // Box parameters are half-extents, MeshManager needs full
            // dimensions
            const auto& params = box->parameters();
            if (params.size() >= 3)
            {
                m_mesh_manager.createBox(mesh_name,
                                         static_cast<float>(params[0] * 2.0),
                                         static_cast<float>(params[1] * 2.0),
                                         static_cast<float>(params[2] * 2.0));
            }
        }
        else if (auto* cyl = dynamic_cast<const robotik::Cylinder*>(&geom))
        {
            const auto& params = cyl->parameters();
            if (params.size() >= 2)
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
            // TODO: Add createSphere() to MeshManager
            // For now, create a small box as placeholder
            const auto& params = sphere->parameters();
            if (!params.empty())
            {
                float r = static_cast<float>(params[0]);
                m_mesh_manager.createBox(
                    mesh_name, r * 2.0f, r * 2.0f, r * 2.0f);
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

} // namespace robotik::renderer
