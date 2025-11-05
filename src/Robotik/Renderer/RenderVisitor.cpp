/**
 * @file RenderVisitor.cpp
 * @brief Implementation of RenderVisitor for rendering robot nodes.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "Robotik/Renderer/RenderVisitor.hpp"

#include "Robotik/Core/Robot/Blueprint/Blueprint.hpp"
#include "Robotik/Core/Robot/Blueprint/Frame.hpp"
#include "Robotik/Core/Robot/Blueprint/Geometry.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/Blueprint/Link.hpp"
#include "Robotik/Core/Robot/Blueprint/Node.hpp"
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
RenderVisitor::RenderVisitor(GeometryManager& geometry_mgr,
                             ShaderManager const& shader_mgr)
    : m_geometry_manager(geometry_mgr), m_shader_manager(shader_mgr)
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
void RenderVisitor::visit(const robotik::Frame& frame)
{
    // Always render axes for frames (coordinate frames)
    const GeometryManager::GPUMesh* axis_mesh =
        m_geometry_manager.getMesh("revolute");
    if (!axis_mesh || !axis_mesh->is_loaded)
    {
        std::cerr << "Warning: axis mesh not found for frames axes rendering"
                  << std::endl;
        return;
    }

    // Get frames world transform
    Eigen::Matrix4f frame_transform = frame.worldTransform().cast<float>();

    // Render the standard RGB axes at the frames position
    constexpr float axis_scale = 1.0f;
    renderAxes(axis_mesh, frame_transform, axis_scale);
}

// ----------------------------------------------------------------------------
void RenderVisitor::visit(const robotik::Node& /*node*/)
{
    // Fallback for generic nodes - nothing to render
}

// ----------------------------------------------------------------------------
void RenderVisitor::renderMesh(const GeometryManager::GPUMesh* p_mesh,
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
void RenderVisitor::renderGrid(const GeometryManager::GPUMesh* p_grid_mesh,
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
void RenderVisitor::renderAxes(const GeometryManager::GPUMesh* p_axes_mesh,
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

// ----------------------------------------------------------------------------
void RenderVisitor::renderGeometry(robotik::Geometry const& geom,
                                   Eigen::Matrix4f const& transform) const
{
    std::string const& mesh_name = geom.name();
    const GeometryManager::GPUMesh* gpu_mesh =
        m_geometry_manager.getMesh(mesh_name);

    // Render if mesh was preloaded successfully
    if (gpu_mesh)
    {
        Eigen::Vector3f color = geom.color.cast<float>();
        renderMesh(gpu_mesh, transform, color);
    }
    else
    {
        std::cerr << "Warning: Mesh not preloaded: " << mesh_name << std::endl;
    }
}

// ----------------------------------------------------------------------------
void RenderVisitor::renderJointAxis(robotik::Joint const& p_joint) const
{
    // Get joint type and check if we should render this type
    switch (p_joint.type())
    {
        case robotik::Joint::Type::REVOLUTE:
        case robotik::Joint::Type::CONTINUOUS:
            if (!m_show_revolute_axes)
                return;
            break;
        case robotik::Joint::Type::PRISMATIC:
            if (!m_show_prismatic_axes)
                return;
            break;
        case robotik::Joint::Type::FIXED:
        default:
            return;
    }

    // Get the axis mesh (should be preloaded by Application)
    const GeometryManager::GPUMesh* axis_mesh =
        m_geometry_manager.getMesh("revolute");
    if (!axis_mesh || !axis_mesh->is_loaded)
    {
        std::cerr << "Warning: axis mesh not found for joint axes rendering"
                  << std::endl;
        return;
    }

    // Get joint world transform
    Eigen::Matrix4f joint_transform = p_joint.worldTransform().cast<float>();

    // Render the standard RGB axes at the joint position
    constexpr float axis_scale = 1.0f;
    renderAxes(axis_mesh, joint_transform, axis_scale);
}

} // namespace robotik::renderer
