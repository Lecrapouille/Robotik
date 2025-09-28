/**
 * @file RobotViewer.cpp
 * @brief Robot viewer class implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Viewer/RobotViewer.hpp"
#include <GL/glew.h>
#include <cstddef>

namespace robotik::viewer
{

// Shader sources
const std::string s_vertex_shader_source = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 color;

out vec3 FragColor;
out vec3 Normal;
out vec3 FragPos;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);

    // Transform normal to world space
    Normal = mat3(transpose(inverse(model))) * aNormal;

    // Fragment position in world space
    FragPos = vec3(model * vec4(aPos, 1.0));

    FragColor = color;
}
)";

const std::string s_fragment_shader_source = R"(
#version 330 core
in vec3 FragColor;
in vec3 Normal;
in vec3 FragPos;

out vec4 outColor;

void main()
{
    // Simple directional light
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.3));
    vec3 lightColor = vec3(1.0, 1.0, 1.0);

    // Normalize the normal
    vec3 norm = normalize(Normal);

    // Ambient lighting
    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * lightColor;

    // Diffuse lighting
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // Combine lighting
    vec3 result = (ambient + diffuse) * FragColor;
    outColor = vec4(result, 1.0);
}
)";

// ----------------------------------------------------------------------------
RobotViewer::RobotViewer(size_t p_width, size_t p_height)
    : m_camera(p_width, p_height),
      m_geometry_renderer(m_shader_manager),
      m_width(p_width),
      m_height(p_height)
{
}

// ----------------------------------------------------------------------------
RobotViewer::~RobotViewer() = default;

// ----------------------------------------------------------------------------
bool RobotViewer::initialize()
{
    // Setup shaders
    if (!setupShaders())
    {
        return false;
    }

    // Initialize geometry renderer
    if (!m_geometry_renderer.initialize())
    {
        m_last_error = "Failed to initialize geometry renderer: " +
                       m_geometry_renderer.getLastError();
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
void RobotViewer::render(const Eigen::Vector3f& p_clear_color)
{
    // Clear screen
    glClearColor(p_clear_color.x(), p_clear_color.y(), p_clear_color.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use shader program
    m_shader_manager.useProgram("default");

    // Set matrices
    int projection_uniform = m_shader_manager.getUniformLocation("projection");
    int view_uniform = m_shader_manager.getUniformLocation("view");

    if (projection_uniform >= 0)
    {
        m_shader_manager.setMatrix4f(projection_uniform,
                                     m_camera.getProjectionMatrix().data());
    }
    if (view_uniform >= 0)
    {
        m_shader_manager.setMatrix4f(view_uniform,
                                     m_camera.getViewMatrix().data());
    }

    // Render grid
    m_geometry_renderer.renderGrid();
}

// ----------------------------------------------------------------------------
const std::string& RobotViewer::getLastError() const
{
    return m_last_error;
}

// ----------------------------------------------------------------------------
bool RobotViewer::setupShaders()
{
    if (!m_shader_manager.createProgram(
            "default", s_vertex_shader_source, s_fragment_shader_source))
    {
        m_last_error = "Failed to create shader program: " +
                       m_shader_manager.getLastError();
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
bool RobotViewer::loadMesh(const std::string& p_mesh_path, bool p_force_reload)
{
    return m_mesh_manager.loadMesh(p_mesh_path, p_force_reload);
}

// ----------------------------------------------------------------------------
bool RobotViewer::renderMesh(const std::string& p_mesh_path,
                             const Eigen::Matrix4f& p_transform,
                             const Eigen::Vector3f& p_color) const
{
    // Set model matrix
    int model_uniform = m_shader_manager.getUniformLocation("model");
    if (model_uniform >= 0)
    {
        m_shader_manager.setMatrix4f(model_uniform, p_transform.data());
    }

    // Set color
    int color_uniform = m_shader_manager.getUniformLocation("color");
    if (color_uniform >= 0)
    {
        m_shader_manager.setVector3f(color_uniform, p_color.data());
    }

    // Render the mesh
    return m_mesh_manager.renderMesh(p_mesh_path);
}

// ----------------------------------------------------------------------------
void RobotViewer::setMeshBasePath(const std::string& p_base_path)
{
    m_mesh_manager.setBasePath(p_base_path);
}

} // namespace robotik::viewer