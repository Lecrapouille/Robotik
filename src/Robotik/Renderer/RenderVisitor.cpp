/**
 * @file RenderVisitor.cpp
 * @brief Implementation of RenderVisitor for rendering robot nodes.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "Robotik/Renderer/RenderVisitor.hpp"

#include "Robotik/Core/Robot/Blueprint/Blueprint.hpp"
#include "Robotik/Core/Robot/Blueprint/Geometry.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/Blueprint/Link.hpp"
#include "Robotik/Core/Robot/Blueprint/Node.hpp"
#include "Robotik/Renderer/Loaders/StlLoader.hpp"
#include "Robotik/Renderer/Managers/MeshManager.hpp"
#include "Robotik/Renderer/Managers/ShaderManager.hpp"

#include <GL/glew.h>
#include <iostream>

namespace robotik::renderer
{

using namespace robotik;

// ----------------------------------------------------------------------------
static const Eigen::Vector3f s_red_color(1.0f, 0.0f, 0.0f);
static const Eigen::Vector3f s_green_color(0.0f, 1.0f, 0.0f);
static const Eigen::Vector3f s_blue_color(0.0f, 0.0f, 1.0f);

// ----------------------------------------------------------------------------
RenderVisitor::RenderVisitor(MeshManager& mesh_mgr,
                             ShaderManager const& shader_mgr)
    : m_mesh_manager(mesh_mgr), m_shader_manager(shader_mgr)
{
}

// ----------------------------------------------------------------------------
void RenderVisitor::renderBlueprint(robotik::Blueprint const& p_blueprint,
                                    robotik::State const& p_state)
{
    // Iterate over all geometries and render them using transforms from State
    p_blueprint.forEachGeometryData([&](robotik::GeometryData const& geom, size_t i) {
        // Get the geometry's mesh
        std::string const& mesh_name = geom.name;
        const MeshManager::GPUMesh* gpu_mesh = m_mesh_manager.getMesh(mesh_name);

        if (!gpu_mesh)
        {
            std::cerr << "Warning: Mesh not preloaded: " << mesh_name << std::endl;
            return; // Skip this geometry
        }

        // Compute world transform for this geometry:
        // T_world = T_parent_link * T_local
        Transform world_transform;
        if (geom.parent_link_index < p_state.link_transforms.size())
        {
            world_transform = p_state.link_transforms[geom.parent_link_index] * 
                            geom.local_transform;
        }
        else
        {
            // Fallback to local transform if parent link index is invalid
            world_transform = geom.local_transform;
        }

        // Render the mesh
        Eigen::Vector3f color = geom.color.cast<float>();
        renderMesh(gpu_mesh, world_transform.cast<float>(), color);
    });
}

// ----------------------------------------------------------------------------
void RenderVisitor::preloadGeometries(robotik::Blueprint const& p_blueprint)
{
    // Iterate through all geometry data and create meshes
    p_blueprint.forEachGeometryData([&](robotik::GeometryData const& geom, size_t i) {
        std::string const& mesh_name = geom.name;

        // Skip if already loaded
        if (m_mesh_manager.hasMesh(mesh_name))
            return;

        // Create mesh based on geometry type
        if (geom.type == robotik::Geometry::Type::BOX)
        {
            const auto& params = geom.parameters;
            if (params.size() == 3)
            {
                m_mesh_manager.createBox(
                    mesh_name,
                    static_cast<float>(params[0]),  // width
                    static_cast<float>(params[1]),  // height
                    static_cast<float>(params[2])); // depth
            }
        }
        else if (geom.type == robotik::Geometry::Type::CYLINDER)
        {
            const auto& params = geom.parameters;
            if (params.size() == 2)
            {
                m_mesh_manager.createCylinder(
                    mesh_name,
                    static_cast<float>(params[0]), // radius
                    static_cast<float>(params[1]), // length
                    32);                           // segments
            }
        }
        else if (geom.type == robotik::Geometry::Type::SPHERE)
        {
            const auto& params = geom.parameters;
            if (params.size() == 1)
            {
                m_mesh_manager.createSphere(
                    mesh_name,
                    static_cast<float>(params[0]), // radius
                    16,                            // latitude segments
                    16);                           // longitude segments
            }
        }
        else if (geom.type == robotik::Geometry::Type::MESH)
        {
            STLLoader loader;
            if (!m_mesh_manager.loadFromFile(
                    mesh_name, geom.mesh_path, loader, false))
            {
                std::cerr << "Failed to load mesh: " << geom.mesh_path
                          << std::endl;
            }
        }
    });
}

// ----------------------------------------------------------------------------
void RenderVisitor::renderMesh(const MeshManager::GPUMesh* p_mesh,
                               const Eigen::Matrix4f& p_transform,
                               const Eigen::Vector3f& p_color) const
{
    if (!p_mesh || !p_mesh->is_loaded)
    {
        return;
    }

    m_shader_manager.setMatrix4f(m_shader_manager.getUniformLocation("model"),
                                 p_transform.data());
    m_shader_manager.setVector3f(m_shader_manager.getUniformLocation("color"),
                                 p_color.data());

    glBindVertexArray(p_mesh->vao);
    glDrawElements(GL_TRIANGLES,
                   static_cast<GLsizei>(p_mesh->index_count),
                   GL_UNSIGNED_INT,
                   nullptr);
}

// ----------------------------------------------------------------------------
void RenderVisitor::renderGrid(const MeshManager::GPUMesh* p_grid_mesh,
                               const Eigen::Vector3f& p_color) const
{
    if (!p_grid_mesh || !p_grid_mesh->is_loaded)
    {
        return;
    }

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    m_shader_manager.setMatrix4f(m_shader_manager.getUniformLocation("model"),
                                 model.data());
    m_shader_manager.setVector3f(m_shader_manager.getUniformLocation("color"),
                                 p_color.data());

    glBindVertexArray(p_grid_mesh->vao);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(p_grid_mesh->index_count));
}

// ----------------------------------------------------------------------------
void RenderVisitor::renderAxes(const MeshManager::GPUMesh* p_axes_mesh,
                               const Eigen::Matrix4f& p_transform,
                               float p_scale) const
{
    if (!p_axes_mesh || !p_axes_mesh->is_loaded)
    {
        return;
    }

    const float radius = 0.02f * p_scale; // Thin cylinder radius
    const float height = p_scale;         // Axis length

    int model_uniform = m_shader_manager.getUniformLocation("model");
    int color_uniform = m_shader_manager.getUniformLocation("color");

    // X axis (red) - cylinder along X direction
    {
        // Create scaling transformation (cylinder default: radius in XY, height
        // in Z)
        Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
        scale(0, 0) = radius;
        scale(1, 1) = radius;
        scale(2, 2) = height / 2.0f;

        // Create rotation to align cylinder with X axis (rotate 90° around Y)
        Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
        rotation.block<3, 3>(0, 0) =
            Eigen::AngleAxisf(M_PIf / 2.0f, Eigen::Vector3f::UnitY())
                .toRotationMatrix();

        // Create translation to position cylinder at half its length along X
        Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
        translation(0, 3) = height * 0.5f;

        // Set model matrix to OpenGL shader
        Eigen::Matrix4f model = p_transform * translation * rotation * scale;
        m_shader_manager.setMatrix4f(model_uniform, model.data());

        // Set color to OpenGL shader
        m_shader_manager.setVector3f(color_uniform, s_red_color.data());

        // Draw the cylinder
        glBindVertexArray(p_axes_mesh->vao);
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(p_axes_mesh->index_count),
                       GL_UNSIGNED_INT,
                       nullptr);
    }

    // Y axis (green) - cylinder along Y direction
    {
        // Create scaling transformation
        Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
        scale(0, 0) = radius;
        scale(1, 1) = radius;
        scale(2, 2) = height / 2.0f;

        // Create rotation to align cylinder with Y axis (rotate 90° around X)
        Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
        rotation.block<3, 3>(0, 0) =
            Eigen::AngleAxisf(-M_PIf / 2.0f, Eigen::Vector3f::UnitX())
                .toRotationMatrix();

        // Create translation to position cylinder at half its length along Y
        Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
        translation(1, 3) = height * 0.5f;

        // Set model matrix to OpenGL shader
        Eigen::Matrix4f model = p_transform * translation * rotation * scale;
        m_shader_manager.setMatrix4f(model_uniform, model.data());

        // Set color to OpenGL shader
        m_shader_manager.setVector3f(color_uniform, s_green_color.data());

        // Draw the cylinder
        glBindVertexArray(p_axes_mesh->vao);
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(p_axes_mesh->index_count),
                       GL_UNSIGNED_INT,
                       nullptr);
    }

    // Z axis (blue) - cylinder along Z direction
    {
        // Create scaling transformation
        Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
        scale(0, 0) = radius;
        scale(1, 1) = radius;
        scale(2, 2) = height / 2.0f; // Base cylinder has height 2

        // No rotation needed, cylinder is already aligned with Z axis

        // Create translation to position cylinder at half its length along Z
        Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
        translation(2, 3) = height * 0.5f;

        // Set model matrix to OpenGL shader
        Eigen::Matrix4f model = p_transform * translation * scale;
        m_shader_manager.setMatrix4f(model_uniform, model.data());

        // Set color to OpenGL shader
        m_shader_manager.setVector3f(color_uniform, s_blue_color.data());

        // Draw the cylinder
        glBindVertexArray(p_axes_mesh->vao);
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(p_axes_mesh->index_count),
                       GL_UNSIGNED_INT,
                       nullptr);
    }
}

} // namespace robotik::renderer
