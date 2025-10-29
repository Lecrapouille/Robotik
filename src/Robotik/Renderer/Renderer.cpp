/**
 * @file Renderer.cpp
 * @brief 3D mesh rendering class implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Renderer/Renderer.hpp"
#include "Robotik/Renderer/Managers/ShaderManager.hpp"

#include <GL/glew.h>
#include <cmath>

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
Renderer::Renderer(ShaderManager& p_shader_manager)
    : m_shader_manager(p_shader_manager)
{
}

// ----------------------------------------------------------------------------
bool Renderer::initialize()
{
    // Get uniform locations
    m_model_uniform = m_shader_manager.getUniformLocation("model");
    m_color_uniform = m_shader_manager.getUniformLocation("color");

    if (m_model_uniform < 0 || m_color_uniform < 0)
    {
        m_error = "Failed to get shader uniform locations";
        return false;
    }

    // Initialize grid
    if (!initializeGrid())
    {
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
void Renderer::render(const MeshManager::GPUMesh* p_mesh,
                      const Eigen::Matrix4f& p_transform,
                      const Eigen::Vector3f& p_color) const
{
    if (!p_mesh || !p_mesh->is_loaded)
    {
        return;
    }

    m_shader_manager.setMatrix4f(m_model_uniform, p_transform.data());
    m_shader_manager.setVector3f(m_color_uniform, p_color.data());

    glBindVertexArray(p_mesh->vao);
    glDrawElements(GL_TRIANGLES,
                   static_cast<GLsizei>(p_mesh->index_count),
                   GL_UNSIGNED_INT,
                   nullptr);
}

// ----------------------------------------------------------------------------
void Renderer::renderGrid(const Eigen::Vector3f& p_color,
                          int /*p_size*/,
                          float /*p_spacing*/) const
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    m_shader_manager.setMatrix4f(m_model_uniform, model.data());
    m_shader_manager.setVector3f(m_color_uniform, p_color.data());

    glBindVertexArray(m_grid.vao);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_grid_vertex_count));
}

// ----------------------------------------------------------------------------
void Renderer::renderAxes(const Eigen::Matrix4f& p_transform,
                          float p_scale,
                          const MeshManager::GPUMesh* p_axes_mesh) const
{
    if (!p_axes_mesh || !p_axes_mesh->is_loaded)
    {
        return;
    }

    const float radius = 0.02f * p_scale;
    const float height = p_scale;

    // X axis (red)
    {
        Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
        scale(0, 0) = radius;
        scale(1, 1) = radius;
        scale(2, 2) = height / 2.0f;

        Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
        rotation.block<3, 3>(0, 0) =
            Eigen::AngleAxisf(M_PIf / 2.0f, Eigen::Vector3f::UnitY())
                .toRotationMatrix();

        Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
        translation(0, 3) = height * 0.5f;

        Eigen::Matrix4f model = p_transform * translation * rotation * scale;
        m_shader_manager.setMatrix4f(m_model_uniform, model.data());

        Eigen::Vector3f red_color(1.0f, 0.0f, 0.0f);
        m_shader_manager.setVector3f(m_color_uniform, red_color.data());

        glBindVertexArray(p_axes_mesh->vao);
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(p_axes_mesh->index_count),
                       GL_UNSIGNED_INT,
                       nullptr);
    }

    // Y axis (green)
    {
        Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
        scale(0, 0) = radius;
        scale(1, 1) = radius;
        scale(2, 2) = height / 2.0f;

        Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
        rotation.block<3, 3>(0, 0) =
            Eigen::AngleAxisf(-M_PIf / 2.0f, Eigen::Vector3f::UnitX())
                .toRotationMatrix();

        Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
        translation(1, 3) = height * 0.5f;

        Eigen::Matrix4f model = p_transform * translation * rotation * scale;
        m_shader_manager.setMatrix4f(m_model_uniform, model.data());

        Eigen::Vector3f green_color(0.0f, 1.0f, 0.0f);
        m_shader_manager.setVector3f(m_color_uniform, green_color.data());

        glBindVertexArray(p_axes_mesh->vao);
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(p_axes_mesh->index_count),
                       GL_UNSIGNED_INT,
                       nullptr);
    }

    // Z axis (blue)
    {
        Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
        scale(0, 0) = radius;
        scale(1, 1) = radius;
        scale(2, 2) = height / 2.0f;

        Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
        translation(2, 3) = height * 0.5f;

        Eigen::Matrix4f model = p_transform * translation * scale;
        m_shader_manager.setMatrix4f(m_model_uniform, model.data());

        Eigen::Vector3f blue_color(0.0f, 0.0f, 1.0f);
        m_shader_manager.setVector3f(m_color_uniform, blue_color.data());

        glBindVertexArray(p_axes_mesh->vao);
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(p_axes_mesh->index_count),
                       GL_UNSIGNED_INT,
                       nullptr);
    }
}

// ----------------------------------------------------------------------------
bool Renderer::initializeGrid()
{
    std::vector<float> grid_vertices;
    generateGrid(grid_vertices, 20, 1.0f);

    glGenVertexArrays(1, &m_grid.vao);
    glGenBuffers(1, &m_grid.vbo);

    glBindVertexArray(m_grid.vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_grid.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 grid_vertices.size() * sizeof(float),
                 grid_vertices.data(),
                 GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    m_grid_vertex_count =
        grid_vertices.size() / 6; // 6 floats per vertex (pos + normal)
    m_grid.is_loaded = true;

    return true;
}

// ----------------------------------------------------------------------------
void Renderer::generateGrid(std::vector<float>& p_vertices,
                            int p_size,
                            float p_spacing) const
{
    p_vertices.clear();

    // Grid in XY plane (Z=0) for Z-up coordinate system to match URDF standard
    for (int i = -p_size; i <= p_size; ++i)
    {
        // Lines parallel to X axis (varying Y)
        p_vertices.push_back(-float(p_size) * p_spacing);
        p_vertices.push_back(float(i) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(1.0f);

        p_vertices.push_back(float(p_size) * p_spacing);
        p_vertices.push_back(float(i) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(1.0f);

        // Lines parallel to Y axis (varying X)
        p_vertices.push_back(float(i) * p_spacing);
        p_vertices.push_back(-float(p_size) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(1.0f);

        p_vertices.push_back(float(i) * p_spacing);
        p_vertices.push_back(float(p_size) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(1.0f);
    }
}

} // namespace robotik::renderer
