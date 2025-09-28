/**
 * @file RobotViewerApplication.cpp
 * @brief Robot viewer application class for the 3D viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Viewer/RobotViewerApplication.hpp"
#include "Robotik/private/Conversions.hpp"

#include <GLFW/glfw3.h>
#include <array>
#include <iostream>

namespace robotik::viewer
{

// Simple vertex shader
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

// Simple fragment shader
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

// Background color
const Eigen::Vector3f s_clear_color(0.2f, 0.3f, 0.3f);

} // namespace robotik::viewer

namespace robotik::viewer
{

// ----------------------------------------------------------------------------
RobotViewerApplication::RobotViewerApplication(Configuration const& p_config)
    : m_config(p_config),
      m_window(p_config),
      m_camera(p_config.window_width, p_config.window_height),
      m_geometry_renderer(m_shader_manager),
      m_title(p_config.window_title)
{
    m_window.setTitle(m_title + " - FPS: " + std::to_string(m_fps));
}

// ----------------------------------------------------------------------------
RobotViewerApplication::~RobotViewerApplication() = default;

// ----------------------------------------------------------------------------
bool RobotViewerApplication::run()
{
    return Application::run(m_config.target_fps, m_config.target_physics_hz);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::setTitle(std::string const& p_title)
{
    m_title = p_title;
    m_window.setTitle(m_title + " - FPS: " + std::to_string(m_fps));
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::isHalting() const
{
    return m_window.isHalting();
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::onSetup()
{
    if (!m_window.initialize())
    {
        m_error = m_window.error();
        return false;
    }

    m_window.setCallbacks(std::bind(&RobotViewerApplication::onKeyInput,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2,
                                    std::placeholders::_3,
                                    std::placeholders::_4),
                          std::bind(&RobotViewerApplication::onMouseButton,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2,
                                    std::placeholders::_3),
                          std::bind(&RobotViewerApplication::onCursorPos,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2),
                          std::bind(&RobotViewerApplication::onScroll,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2),
                          std::bind(&RobotViewerApplication::onWindowResize,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));

    // Setup shaders
    if (!m_shader_manager.createProgram(
            "default", s_vertex_shader_source, s_fragment_shader_source))
    {
        m_error =
            "Failed to create shader program: " + m_shader_manager.error();
        return false;
    }

    useShaderProgram("default");

    // Initialize geometry renderer
    if (!m_geometry_renderer.initialize())
    {
        m_error = "Failed to initialize geometry renderer: " +
                  m_geometry_renderer.error();
        return false;
    }

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    // Set mesh base path if specified
    if (!m_config.search_paths.empty())
    {
        m_mesh_manager.setBasePath(m_config.search_paths);
    }

    if (!loadRobot())
    {
        return false;
    }
    return true;
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::useShaderProgram(const std::string& p_program_name)
{
    // Use shader program
    m_shader_manager.useProgram(p_program_name);

    // Initialize shader uniform locations
    m_model_uniform = m_shader_manager.getUniformLocation("model");
    m_color_uniform = m_shader_manager.getUniformLocation("color");
    m_projection_uniform = m_shader_manager.getUniformLocation("projection");
    m_view_uniform = m_shader_manager.getUniformLocation("view");
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::loadRobot()
{
    // Load robot from URDF file if specified
    if (!m_robot_manager.loadRobot(m_config.urdf_file))
    {
        m_error = "Failed to load robot from URDF: " + m_robot_manager.error();
        return false;
    }

    // Get the loaded robot name
    if (auto* controlled_robot = m_robot_manager.getCurrentRobot();
        controlled_robot)
    {
        controlled_robot->control_joint = scene::Node::find(
            controlled_robot->robot->root(), m_config.control_joint);
        controlled_robot->camera_target = scene::Node::find(
            controlled_robot->robot->root(), m_config.camera_target);

        // Set initial joint values to a neutral position
        std::vector<double> neutral_joints = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        m_robot_manager.setRobotJointValues(controlled_robot->robot->name(),
                                            neutral_joints);

        std::cout << "Successfully loaded robot: "
                  << controlled_robot->robot->name() << std::endl;

        // Set camera view type
        if (controlled_robot->camera_target)
        {
            Eigen::Vector3d target_pos_d = utils::getTranslation(
                controlled_robot->camera_target->worldTransform());
            Eigen::Vector3f target_pos = target_pos_d.cast<float>();
            m_camera.setView(m_config.camera_view, target_pos);
        }
        else
        {
            m_camera.setView(m_config.camera_view, Eigen::Vector3f::Zero());
        }
    }

    return true;
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onCleanup()
{
    m_robot_manager.reset();
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onDraw()
{
    // Clear screen
    glClearColor(s_clear_color.x(), s_clear_color.y(), s_clear_color.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Camera tracks the robot
    auto const* controlled_robot = m_robot_manager.getCurrentRobot();
    Eigen::Vector3f target_pos;
    if (controlled_robot && controlled_robot->camera_target)
    {
        Eigen::Vector3d target_pos_d = utils::getTranslation(
            controlled_robot->camera_target->worldTransform());
        target_pos = target_pos_d.cast<float>();
    }
    else
    {
        target_pos = Eigen::Vector3f::Zero();
    }
    m_camera.setView(m_camera.getViewType(), target_pos);

    // Set camera matrices
    m_shader_manager.setMatrix4f(m_projection_uniform,
                                 m_camera.getProjectionMatrix().data());
    m_shader_manager.setMatrix4f(m_view_uniform,
                                 m_camera.getViewMatrix().data());

    // Render the world ground
    m_geometry_renderer.renderGrid();

    // Render all loaded robots
    for (const auto& [_, it] : m_robot_manager.robots())
    {
        if (it.is_visible)
        {
            renderRobot(*it.robot);
        }
    }

    m_window.swapBuffers();
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::renderRobot(Robot const& p_robot)
{
    if (!p_robot.hasRoot())
        return;

    p_robot.root().traverse(
        [this](scene::Node const& node, size_t /*p_depth*/)
        {
            if (auto geometry = dynamic_cast<Geometry const*>(&node))
            {
                Eigen::Matrix4f transform = node.worldTransform().cast<float>();
                renderGeometry(*geometry, transform);
            }
        });
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::renderGeometry(Geometry const& p_geometry,
                                            Eigen::Matrix4f const& p_transform)
{
    // Set model matrix
    if (m_model_uniform >= 0)
    {
        m_shader_manager.setMatrix4f(m_model_uniform, p_transform.data());
    }

    // Set color
    if (m_color_uniform >= 0)
    {
        m_shader_manager.setVector3f(m_color_uniform, p_geometry.color.data());
    }

    // Render based on geometry type
    switch (p_geometry.type())
    {
        case Geometry::Type::BOX:
        {
            if (auto const& params = p_geometry.parameters();
                params.size() >= 3)
            {
                m_geometry_renderer.renderBox(p_transform,
                                              p_geometry.color,
                                              static_cast<float>(params[0]),
                                              static_cast<float>(params[1]),
                                              static_cast<float>(params[2]));
            }
            break;
        }
        case Geometry::Type::CYLINDER:
        {
            if (auto const& params = p_geometry.parameters();
                params.size() >= 2)
            {
                m_geometry_renderer.renderCylinder(
                    p_transform,
                    p_geometry.color,
                    static_cast<float>(params[0]),
                    static_cast<float>(params[1]));
            }
            break;
        }
        case Geometry::Type::SPHERE:
        {
            if (auto const& params = p_geometry.parameters(); !params.empty())
            {
                m_geometry_renderer.renderSphere(p_transform,
                                                 p_geometry.color,
                                                 static_cast<float>(params[0]));
            }
            break;
        }
        case Geometry::Type::MESH:
        {
            // Load and render mesh using MeshManager
            if (std::string const& mesh_path = p_geometry.meshPath();
                !mesh_path.empty())
            {
                // Load mesh if not already loaded
                if (!m_mesh_manager.isMeshLoaded(mesh_path))
                {
                    m_mesh_manager.loadMesh(mesh_path);
                }

                // Render the mesh
                m_mesh_manager.renderMesh(mesh_path);
            }
            break;
        }
        default:
            break;
    }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onUpdate(float const /* dt */)
{
    // No update logic needed for now
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onPhysicUpdate(float const /* dt */)
{
    // No physics simulation needed for now
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onFPSUpdated(size_t const p_fps)
{
    m_fps = p_fps;
    m_window.setTitle(m_title + " - FPS: " + std::to_string(p_fps));
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onWindowResize(int width, int height)
{
    glViewport(0, 0, width, height);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onKeyInput(int key,
                                        int /* scancode */,
                                        int action,
                                        int /* mods */)
{
    if (action != GLFW_PRESS)
    {
        return;
    }

    switch (key)
    {
        case GLFW_KEY_ESCAPE:
            m_window.halt();
            break;
        // Camera view controls
        case GLFW_KEY_1:
            m_camera.setView(Camera::ViewType::PERSPECTIVE,
                             Eigen::Vector3f(0.0f, 0.0f, 0.0f));
            break;
        case GLFW_KEY_2:
            m_camera.setView(Camera::ViewType::TOP,
                             Eigen::Vector3f(0.0f, 0.0f, 0.0f));
            break;
        case GLFW_KEY_3:
            m_camera.setView(Camera::ViewType::FRONT,
                             Eigen::Vector3f(0.0f, 0.0f, 0.0f));
            break;
        case GLFW_KEY_4:
            m_camera.setView(Camera::ViewType::SIDE,
                             Eigen::Vector3f(0.0f, 0.0f, 0.0f));
            break;
        case GLFW_KEY_5:
            m_camera.setView(Camera::ViewType::ISOMETRIC,
                             Eigen::Vector3f(0.0f, 0.0f, 0.0f));
            break;
        // Reset robot to neutral position
        case GLFW_KEY_SPACE:
            if (auto* controlled_robot = m_robot_manager.getCurrentRobot();
                controlled_robot)
            {
                controlled_robot->robot->setNeutralPosition();
            }
            break;
        // Toggle robot visibility
        case GLFW_KEY_V:
            if (auto* robot = m_robot_manager.getCurrentRobot(); robot)
            {
                robot->is_visible = !robot->is_visible;
            }
            break;
        default:
            break;
    }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onMouseButton(int /* button */,
                                           int /* action */,
                                           int /* mods */)
{
    // Handle mouse button event
    // This could be used for camera controls, object selection, etc.
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onCursorPos(double /* xpos */, double /* ypos */)
{
    // Handle cursor position event
    // This could be used for camera controls, mouse tracking, etc.
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onScroll(double /* xoffset */,
                                      double /* yoffset */)
{
    // Handle scroll event
    // This could be used for zoom controls, etc.
}

} // namespace robotik::viewer