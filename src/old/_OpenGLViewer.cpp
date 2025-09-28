#include "DAELoader.hpp"
#include "OpenGLWindow.hpp"
#include "STLLoader.hpp"

#include "Robotik/private/Path.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace robotik
{

static const Eigen::Vector3f red_color(1.0f, 0.0f, 0.0f);
static const Eigen::Vector3f green_color(0.0f, 1.0f, 0.0f);

// ============================================================================
// GeometryRenderer Implementation (Stub - to be implemented)
// ============================================================================

GeometryRenderer::GeometryRenderer() = default;
GeometryRenderer::~GeometryRenderer() = default;

bool GeometryRenderer::initialize()
{
    // TODO: Initialize geometry buffers
    return true;
}

void GeometryRenderer::renderGrid() const
{
    // TODO: Implement grid rendering
}

void GeometryRenderer::renderBox(const Eigen::Matrix4f& p_transform,
                                 const Eigen::Vector3f& p_size) const
{
    // TODO: Implement box rendering
}

void GeometryRenderer::renderCylinder(const Eigen::Matrix4f& p_transform,
                                      float p_radius,
                                      float p_length) const
{
    // TODO: Implement cylinder rendering
}

void GeometryRenderer::renderSphere(const Eigen::Matrix4f& p_transform,
                                    float p_radius) const
{
    // TODO: Implement sphere rendering
}

void GeometryRenderer::renderAxes(const Eigen::Matrix4f& p_transform,
                                  double p_scale) const
{
    // TODO: Implement axes rendering
}

// ============================================================================
// OpenGLWindow Implementation
// ============================================================================

// ----------------------------------------------------------------------------
OpenGLWindow::OpenGLWindow(Path& p_path,
                           size_t p_width,
                           size_t p_height,
                           const std::string& p_title)
    : m_path(p_path),
      m_camera_manager(p_width, p_height),
      m_mesh_manager(p_path),
      m_width(p_width),
      m_height(p_height),
      m_title(p_title)
{
    // Initialize URDF to OpenGL coordinate system conversion matrix
    m_urdf_to_opengl_matrix = Eigen::Matrix4f::Zero();
    m_urdf_to_opengl_matrix(0, 1) = -1.0f; // OpenGL_X = -URDF_Y
    m_urdf_to_opengl_matrix(1, 2) = 1.0f;  // OpenGL_Y = URDF_Z
    m_urdf_to_opengl_matrix(2, 0) = -1.0f; // OpenGL_Z = -URDF_X
    m_urdf_to_opengl_matrix(3, 3) = 1.0f;  // Homogeneous coordinate
}

// ----------------------------------------------------------------------------
OpenGLWindow::~OpenGLWindow()
{
    // Cleanup is now handled by the specialized managers
    // Only cleanup window resources here
    if (m_window)
    {
        glfwDestroyWindow(m_window);
    }
    glfwTerminate();
}

// ----------------------------------------------------------------------------
bool OpenGLWindow::initialize()
{
    // Initialize GLFW
    if (!glfwInit())
    {
        m_error_message = "Failed to initialize GLFW";
        return false;
    }

    // Set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window
    m_window = glfwCreateWindow(
        int(m_width), int(m_height), m_title.c_str(), nullptr, nullptr);
    if (!m_window)
    {
        m_error_message = "Failed to create GLFW window";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);

    // Initialize GLEW
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
    {
        m_error_message = "Failed to initialize GLEW";
        glfwTerminate();
        return false;
    }

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    // Initialize managers
    if (!m_shader_manager.initialize())
    {
        m_error_message =
            "Failed to initialize shaders: " + m_shader_manager.error();
        return false;
    }

    if (!m_geometry_renderer.initialize())
    {
        m_error_message = "Failed to initialize geometry renderer";
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
bool OpenGLWindow::isHalting() const
{
    return glfwWindowShouldClose(m_window);
}

// ----------------------------------------------------------------------------
void OpenGLWindow::processInput(KeyCallback const& p_key_callback)
{
    glfwPollEvents();

    // Update key states
    for (int i = 0; i < 256; ++i)
    {
        bool pressed = glfwGetKey(m_window, i) == GLFW_PRESS;
        if (pressed && !m_keys[i])
        {
            // Key just pressed
            p_key_callback(i, GLFW_PRESS);
        }
        m_keys[i] = pressed;
    }
}

// ----------------------------------------------------------------------------
bool OpenGLWindow::isKeyPressed(int p_key) const
{
    if (p_key < 0 || p_key >= 256)
        return false;
    return m_keys[p_key];
}

// ----------------------------------------------------------------------------
void OpenGLWindow::cameraView(CameraViewType p_view_type,
                              const Eigen::Vector3f& p_camera_target)
{
    m_camera_manager.setView(p_view_type, p_camera_target);
}

// ----------------------------------------------------------------------------
void OpenGLWindow::cameraView(CameraViewType p_view_type,
                              const Eigen::Vector3d& p_camera_target)
{
    m_camera_manager.setView(p_view_type, p_camera_target.cast<float>());
}

// ----------------------------------------------------------------------------
void OpenGLWindow::render(Robot const& p_robot)
{
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use shader program
    m_shader_manager.use();

    // Set projection matrix
    glUniformMatrix4fv(
        glGetUniformLocation(m_shader_manager.getProgramId(), "projection"),
        1,
        GL_FALSE,
        m_camera_manager.getProjectionMatrix().data());

    // Set view matrix
    glUniformMatrix4fv(
        glGetUniformLocation(m_shader_manager.getProgramId(), "view"),
        1,
        GL_FALSE,
        m_camera_manager.getViewMatrix().data());

    // Render grid
    renderGrid();

    // Render robot
    renderRobot(p_robot);

    glfwSwapBuffers(m_window);
}

// ============================================================================
// Private rendering methods (simplified stubs)
// ============================================================================

void OpenGLWindow::renderGrid() const
{
    m_geometry_renderer.renderGrid();
}

void OpenGLWindow::renderBox(Transform const& p_transform,
                             const Eigen::Vector3f& p_color) const
{
    Eigen::Matrix4f model = m_urdf_to_opengl_matrix * p_transform.cast<float>();
    glUniformMatrix4fv(
        glGetUniformLocation(m_shader_manager.getProgramId(), "model"),
        1,
        GL_FALSE,
        model.data());
    glUniform3fv(glGetUniformLocation(m_shader_manager.getProgramId(), "color"),
                 1,
                 p_color.data());

    Eigen::Vector3f size(1.0f, 1.0f, 1.0f);
    m_geometry_renderer.renderBox(model, size);
}

void OpenGLWindow::renderCylinder(Transform const& p_transform,
                                  const Eigen::Vector3f& p_color) const
{
    Eigen::Matrix4f model = m_urdf_to_opengl_matrix * p_transform.cast<float>();
    glUniformMatrix4fv(
        glGetUniformLocation(m_shader_manager.getProgramId(), "model"),
        1,
        GL_FALSE,
        model.data());
    glUniform3fv(glGetUniformLocation(m_shader_manager.getProgramId(), "color"),
                 1,
                 p_color.data());

    m_geometry_renderer.renderCylinder(model, 1.0f, 1.0f);
}

void OpenGLWindow::renderSphere(Transform const& p_transform,
                                const Eigen::Vector3f& p_color) const
{
    Eigen::Matrix4f model = m_urdf_to_opengl_matrix * p_transform.cast<float>();
    glUniformMatrix4fv(
        glGetUniformLocation(m_shader_manager.getProgramId(), "model"),
        1,
        GL_FALSE,
        model.data());
    glUniform3fv(glGetUniformLocation(m_shader_manager.getProgramId(), "color"),
                 1,
                 p_color.data());

    m_geometry_renderer.renderSphere(model, 1.0f);
}

void OpenGLWindow::renderRobot(Robot const& p_robot) const
{
    // TODO: Implement robot rendering
    // This would traverse the robot's kinematic tree and render each component
}

void OpenGLWindow::renderJoint(Joint const& p_joint,
                               Transform const& p_world_transform) const
{
    // TODO: Implement joint rendering
}

void OpenGLWindow::renderLink(Link const& p_link,
                              Transform const& p_world_transform) const
{
    // TODO: Implement link rendering
}

void OpenGLWindow::renderGeometry(Geometry const& p_geometry,
                                  Transform const& p_world_transform) const
{
    // TODO: Implement geometry rendering
}

void OpenGLWindow::renderAxes(Transform const& p_transform,
                              double p_scale) const
{
    Eigen::Matrix4f model = m_urdf_to_opengl_matrix * p_transform.cast<float>();
    m_geometry_renderer.renderAxes(model, p_scale);
}

void OpenGLWindow::renderMesh(Transform const& p_transform,
                              const std::string& p_mesh_path,
                              const Eigen::Vector3f& p_color) const
{
    // Load mesh if not already loaded
    if (!m_mesh_manager.isMeshLoaded(p_mesh_path))
    {
        const_cast<MeshManager&>(m_mesh_manager).loadMesh(p_mesh_path);
    }

    // Set uniforms
    Eigen::Matrix4f model = m_urdf_to_opengl_matrix * p_transform.cast<float>();
    glUniformMatrix4fv(
        glGetUniformLocation(m_shader_manager.getProgramId(), "model"),
        1,
        GL_FALSE,
        model.data());
    glUniform3fv(glGetUniformLocation(m_shader_manager.getProgramId(), "color"),
                 1,
                 p_color.data());

    // Render mesh
    m_mesh_manager.renderMesh(p_mesh_path);
}

// Stub implementations for compatibility
void OpenGLWindow::initializeMeshLoaders() {}
bool OpenGLWindow::initializeShaders()
{
    return m_shader_manager.initialize();
}

} // namespace robotik
