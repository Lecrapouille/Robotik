/**
 * @file GeometryRenderer.cpp
 * @brief OpenGL geometry rendering class implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Viewer/GeometryRenderer.hpp"
#include "Viewer/ShaderManager.hpp"
#include <GL/glew.h>
#include <cmath>

namespace robotik::viewer
{

// ----------------------------------------------------------------------------
GeometryRenderer::GeometryRenderer(ShaderManager& p_shader_manager)
    : m_shader_manager(p_shader_manager)
{
}

// ----------------------------------------------------------------------------
GeometryRenderer::~GeometryRenderer()
{
    // Clean up OpenGL resources
    if (m_box_vao != 0)
        glDeleteVertexArrays(1, &m_box_vao);
    if (m_box_vbo != 0)
        glDeleteBuffers(1, &m_box_vbo);
    if (m_box_ebo != 0)
        glDeleteBuffers(1, &m_box_ebo);

    if (m_cylinder_vao != 0)
        glDeleteVertexArrays(1, &m_cylinder_vao);
    if (m_cylinder_vbo != 0)
        glDeleteBuffers(1, &m_cylinder_vbo);
    if (m_cylinder_ebo != 0)
        glDeleteBuffers(1, &m_cylinder_ebo);

    if (m_sphere_vao != 0)
        glDeleteVertexArrays(1, &m_sphere_vao);
    if (m_sphere_vbo != 0)
        glDeleteBuffers(1, &m_sphere_vbo);
    if (m_sphere_ebo != 0)
        glDeleteBuffers(1, &m_sphere_ebo);

    if (m_grid_vao != 0)
        glDeleteVertexArrays(1, &m_grid_vao);
    if (m_grid_vbo != 0)
        glDeleteBuffers(1, &m_grid_vbo);
}

// ----------------------------------------------------------------------------
bool GeometryRenderer::initialize()
{
    // Get uniform locations
    m_model_uniform = m_shader_manager.getUniformLocation("model");
    m_color_uniform = m_shader_manager.getUniformLocation("color");

    if (m_model_uniform < 0 || m_color_uniform < 0)
    {
        m_error = "Failed to get shader uniform locations";
        return false;
    }

    // Initialize all geometry
    if (!initializeBox() || !initializeCylinder() || !initializeSphere() ||
        !initializeGrid())
    {
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
void GeometryRenderer::renderBox(const Eigen::Matrix4f& p_transform,
                                 const Eigen::Vector3f& p_color,
                                 float p_width,
                                 float p_height,
                                 float p_depth) const
{
    // Create scaling matrix
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    scale(0, 0) = p_width;
    scale(1, 1) = p_height;
    scale(2, 2) = p_depth;

    Eigen::Matrix4f model = p_transform * scale;

    m_shader_manager.setMatrix4f(m_model_uniform, model.data());
    m_shader_manager.setVector3f(m_color_uniform, p_color.data());

    glBindVertexArray(m_box_vao);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, nullptr);
}

// ----------------------------------------------------------------------------
void GeometryRenderer::renderCylinder(const Eigen::Matrix4f& p_transform,
                                      const Eigen::Vector3f& p_color,
                                      float p_radius,
                                      float p_height) const
{
    // Create scaling matrix
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    scale(0, 0) = p_radius;
    scale(1, 1) = p_radius;
    scale(2, 2) = p_height / 2.0f; // Base cylinder has height 2

    Eigen::Matrix4f model = p_transform * scale;

    m_shader_manager.setMatrix4f(m_model_uniform, model.data());
    m_shader_manager.setVector3f(m_color_uniform, p_color.data());

    glBindVertexArray(m_cylinder_vao);
    glDrawElements(GL_TRIANGLES,
                   static_cast<GLsizei>(m_cylinder_index_count),
                   GL_UNSIGNED_INT,
                   nullptr);
}

// ----------------------------------------------------------------------------
void GeometryRenderer::renderSphere(const Eigen::Matrix4f& p_transform,
                                    const Eigen::Vector3f& p_color,
                                    float p_radius) const
{
    // Create scaling matrix
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    scale(0, 0) = p_radius;
    scale(1, 1) = p_radius;
    scale(2, 2) = p_radius;

    Eigen::Matrix4f model = p_transform * scale;

    m_shader_manager.setMatrix4f(m_model_uniform, model.data());
    m_shader_manager.setVector3f(m_color_uniform, p_color.data());

    glBindVertexArray(m_sphere_vao);
    glDrawElements(GL_TRIANGLES,
                   static_cast<GLsizei>(m_sphere_index_count),
                   GL_UNSIGNED_INT,
                   nullptr);
}

// ----------------------------------------------------------------------------
void GeometryRenderer::renderGrid(const Eigen::Vector3f& p_color,
                                  int p_size,
                                  float p_spacing) const
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    m_shader_manager.setMatrix4f(m_model_uniform, model.data());
    m_shader_manager.setVector3f(m_color_uniform, p_color.data());

    glBindVertexArray(m_grid_vao);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_grid_vertex_count));
}

// ----------------------------------------------------------------------------
void GeometryRenderer::renderAxes(const Eigen::Matrix4f& p_transform,
                                  float p_scale) const
{
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

        glBindVertexArray(m_cylinder_vao);
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(m_cylinder_index_count),
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

        glBindVertexArray(m_cylinder_vao);
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(m_cylinder_index_count),
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

        glBindVertexArray(m_cylinder_vao);
        glDrawElements(GL_TRIANGLES,
                       static_cast<GLsizei>(m_cylinder_index_count),
                       GL_UNSIGNED_INT,
                       nullptr);
    }
}

// ----------------------------------------------------------------------------
bool GeometryRenderer::initializeBox()
{
    std::vector<float> box_vertices;
    std::vector<unsigned int> box_indices;
    generateBox(box_vertices, box_indices);

    glGenVertexArrays(1, &m_box_vao);
    glGenBuffers(1, &m_box_vbo);
    glGenBuffers(1, &m_box_ebo);

    glBindVertexArray(m_box_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_box_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 box_vertices.size() * sizeof(float),
                 box_vertices.data(),
                 GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_box_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 box_indices.size() * sizeof(unsigned int),
                 box_indices.data(),
                 GL_STATIC_DRAW);

    glVertexAttribPointer(
        0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    return true;
}

// ----------------------------------------------------------------------------
bool GeometryRenderer::initializeCylinder()
{
    std::vector<float> cylinder_vertices;
    std::vector<unsigned int> cylinder_indices;
    generateCylinder(cylinder_vertices, cylinder_indices, 1.0f, 2.0f, 16);

    glGenVertexArrays(1, &m_cylinder_vao);
    glGenBuffers(1, &m_cylinder_vbo);
    glGenBuffers(1, &m_cylinder_ebo);

    glBindVertexArray(m_cylinder_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_cylinder_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 cylinder_vertices.size() * sizeof(float),
                 cylinder_vertices.data(),
                 GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_cylinder_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 cylinder_indices.size() * sizeof(unsigned int),
                 cylinder_indices.data(),
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

    m_cylinder_index_count = cylinder_indices.size();
    return true;
}

// ----------------------------------------------------------------------------
bool GeometryRenderer::initializeSphere()
{
    std::vector<float> sphere_vertices;
    std::vector<unsigned int> sphere_indices;
    generateSphere(sphere_vertices, sphere_indices, 1.0f, 16, 16);

    glGenVertexArrays(1, &m_sphere_vao);
    glGenBuffers(1, &m_sphere_vbo);
    glGenBuffers(1, &m_sphere_ebo);

    glBindVertexArray(m_sphere_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_sphere_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 sphere_vertices.size() * sizeof(float),
                 sphere_vertices.data(),
                 GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_sphere_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 sphere_indices.size() * sizeof(unsigned int),
                 sphere_indices.data(),
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

    m_sphere_index_count = sphere_indices.size();
    return true;
}

// ----------------------------------------------------------------------------
bool GeometryRenderer::initializeGrid()
{
    std::vector<float> grid_vertices;
    generateGrid(grid_vertices, 20, 1.0f);

    glGenVertexArrays(1, &m_grid_vao);
    glGenBuffers(1, &m_grid_vbo);

    glBindVertexArray(m_grid_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_grid_vbo);
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
    return true;
}

// ----------------------------------------------------------------------------
void GeometryRenderer::generateBox(std::vector<float>& p_vertices,
                                   std::vector<unsigned int>& p_indices) const
{
    p_vertices.clear();
    p_indices.clear();

    // Box vertices with positions and normals
    const float box_vertices[] = {
        // positions          // normals
        -0.5f, -0.5f, -0.5f, 0.0f,  0.0f,  -1.0f, 0.5f,  -0.5f, -0.5f, 0.0f,
        0.0f,  -1.0f, 0.5f,  0.5f,  -0.5f, 0.0f,  0.0f,  -1.0f, -0.5f, 0.5f,
        -0.5f, 0.0f,  0.0f,  -1.0f, -0.5f, -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,
        0.5f,  -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,  0.5f,  0.5f,  0.5f,  0.0f,
        0.0f,  1.0f,  -0.5f, 0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  -0.5f, -0.5f,
        -0.5f, -1.0f, 0.0f,  0.0f,  -0.5f, 0.5f,  -0.5f, -1.0f, 0.0f,  0.0f,
        -0.5f, 0.5f,  0.5f,  -1.0f, 0.0f,  0.0f,  -0.5f, -0.5f, 0.5f,  -1.0f,
        0.0f,  0.0f,  0.5f,  -0.5f, -0.5f, 1.0f,  0.0f,  0.0f,  0.5f,  0.5f,
        -0.5f, 1.0f,  0.0f,  0.0f,  0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
        0.5f,  -0.5f, 0.5f,  1.0f,  0.0f,  0.0f,  -0.5f, -0.5f, -0.5f, 0.0f,
        -1.0f, 0.0f,  0.5f,  -0.5f, -0.5f, 0.0f,  -1.0f, 0.0f,  0.5f,  -0.5f,
        0.5f,  0.0f,  -1.0f, 0.0f,  -0.5f, -0.5f, 0.5f,  0.0f,  -1.0f, 0.0f,
        -0.5f, 0.5f,  -0.5f, 0.0f,  1.0f,  0.0f,  0.5f,  0.5f,  -0.5f, 0.0f,
        1.0f,  0.0f,  0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  -0.5f, 0.5f,
        0.5f,  0.0f,  1.0f,  0.0f
    };

    // Copy vertices data
    p_vertices.assign(box_vertices, box_vertices + std::size(box_vertices));

    // Box indices
    const unsigned int box_indices[] = {
        0,  1,  2,  2,  3,  0,  // front
        4,  5,  6,  6,  7,  4,  // back
        8,  9,  10, 10, 11, 8,  // left
        12, 13, 14, 14, 15, 12, // right
        16, 17, 18, 18, 19, 16, // bottom
        20, 21, 22, 22, 23, 20  // top
    };

    // Copy indices data
    p_indices.assign(box_indices, box_indices + std::size(box_indices));
}

// ----------------------------------------------------------------------------
void GeometryRenderer::generateCylinder(std::vector<float>& p_vertices,
                                        std::vector<unsigned int>& p_indices,
                                        float p_radius,
                                        float p_height,
                                        size_t p_segments) const
{
    p_vertices.clear();
    p_indices.clear();

    // Generate vertices
    // Bottom center (at z = -height/2)
    p_vertices.insert(p_vertices.end(),
                      { 0.0f, 0.0f, -p_height / 2, 0.0f, 0.0f, -1.0f });
    // Top center (at z = height/2)
    p_vertices.insert(p_vertices.end(),
                      { 0.0f, 0.0f, p_height / 2, 0.0f, 0.0f, 1.0f });

    // Bottom and top circles
    for (size_t i = 0; i <= p_segments; ++i)
    {
        float angle = 2.0f * M_PIf * float(i) / float(p_segments);
        float x = p_radius * std::cos(angle);
        float y = p_radius * std::sin(angle);

        // Bottom circle vertex
        p_vertices.insert(p_vertices.end(),
                          { x, y, -p_height / 2, 0.0f, 0.0f, -1.0f });
        // Top circle vertex
        p_vertices.insert(p_vertices.end(),
                          { x, y, p_height / 2, 0.0f, 0.0f, 1.0f });
    }

    // Side vertices with side normals
    for (size_t i = 0; i <= p_segments; ++i)
    {
        float angle = 2.0f * M_PIf * float(i) / float(p_segments);
        float x = p_radius * std::cos(angle);
        float y = p_radius * std::sin(angle);

        // Side vertices with side normals
        float nx = std::cos(angle);
        float ny = std::sin(angle);
        p_vertices.insert(p_vertices.end(),
                          { x, y, -p_height / 2, nx, ny, 0.0f });
        p_vertices.insert(p_vertices.end(),
                          { x, y, p_height / 2, nx, ny, 0.0f });
    }

    // Generate indices
    // Bottom cap
    for (size_t i = 0; i < p_segments; ++i)
    {
        p_indices.insert(p_indices.end(),
                         { 0u,
                           static_cast<unsigned int>(2 + i * 2),
                           static_cast<unsigned int>(
                               2 + ((i + 1) % (p_segments + 1)) * 2) });
    }

    // Top cap
    for (size_t i = 0; i < p_segments; ++i)
    {
        p_indices.insert(
            p_indices.end(),
            { 1u,
              static_cast<unsigned int>(3 + ((i + 1) % (p_segments + 1)) * 2),
              static_cast<unsigned int>(3 + i * 2) });
    }

    // Side faces
    size_t side_offset = 2 + (p_segments + 1) * 2;
    for (size_t i = 0; i < p_segments; ++i)
    {
        size_t curr = side_offset + i * 2;
        size_t next = side_offset + ((i + 1) % (p_segments + 1)) * 2;

        // Two triangles per side face
        p_indices.insert(p_indices.end(),
                         { static_cast<unsigned int>(curr),
                           static_cast<unsigned int>(next),
                           static_cast<unsigned int>(curr + 1) });
        p_indices.insert(p_indices.end(),
                         { static_cast<unsigned int>(next),
                           static_cast<unsigned int>(next + 1),
                           static_cast<unsigned int>(curr + 1) });
    }
}

// ----------------------------------------------------------------------------
void GeometryRenderer::generateSphere(std::vector<float>& p_vertices,
                                      std::vector<unsigned int>& p_indices,
                                      float p_radius,
                                      size_t p_latitude_segments,
                                      size_t p_longitude_segments) const
{
    p_vertices.clear();
    p_indices.clear();

    // Generate vertices
    for (size_t lat = 0; lat <= p_latitude_segments; ++lat)
    {
        float theta = M_PIf * float(lat) / float(p_latitude_segments);
        float sin_theta = std::sin(theta);
        float cos_theta = std::cos(theta);

        for (size_t lon = 0; lon <= p_longitude_segments; ++lon)
        {
            float phi = 2.0f * M_PIf * float(lon) / float(p_longitude_segments);
            float sin_phi = std::sin(phi);
            float cos_phi = std::cos(phi);

            float x = sin_theta * cos_phi;
            float y = cos_theta;
            float z = sin_theta * sin_phi;

            // Position
            p_vertices.push_back(p_radius * x);
            p_vertices.push_back(p_radius * y);
            p_vertices.push_back(p_radius * z);

            // Normal (same as normalized position for sphere)
            p_vertices.push_back(x);
            p_vertices.push_back(y);
            p_vertices.push_back(z);
        }
    }

    // Generate indices
    for (size_t lat = 0; lat < p_latitude_segments; ++lat)
    {
        for (size_t lon = 0; lon < p_longitude_segments; ++lon)
        {
            size_t first = lat * (p_longitude_segments + 1) + lon;
            size_t second = first + p_longitude_segments + 1;

            // Two triangles per quad
            p_indices.insert(p_indices.end(),
                             { static_cast<unsigned int>(first),
                               static_cast<unsigned int>(second),
                               static_cast<unsigned int>(first + 1) });
            p_indices.insert(p_indices.end(),
                             { static_cast<unsigned int>(second),
                               static_cast<unsigned int>(second + 1),
                               static_cast<unsigned int>(first + 1) });
        }
    }
}

// ----------------------------------------------------------------------------
void GeometryRenderer::generateGrid(std::vector<float>& p_vertices,
                                    int p_size,
                                    float p_spacing) const
{
    p_vertices.clear();

    for (int i = -p_size; i <= p_size; ++i)
    {
        // Lines parallel to X axis
        p_vertices.push_back(-float(p_size) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(float(i) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(1.0f);
        p_vertices.push_back(0.0f);

        p_vertices.push_back(float(p_size) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(float(i) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(1.0f);
        p_vertices.push_back(0.0f);

        // Lines parallel to Z axis
        p_vertices.push_back(float(i) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(-float(p_size) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(1.0f);
        p_vertices.push_back(0.0f);

        p_vertices.push_back(float(i) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(float(p_size) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(1.0f);
        p_vertices.push_back(0.0f);
    }
}

} // namespace robotik::viewer
