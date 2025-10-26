#include "RobotViewerApplication.hpp"
#include "Robotik/Core/Loaders/UrdfLoader.hpp"
#include "Robotik/Core/Robot.hpp"
#include "Robotik/Renderer/Camera/OrbitController.hpp"
#include "Robotik/Renderer/Camera/PerspectiveCamera.hpp"
#include "Robotik/Renderer/Loaders/STLLoader.hpp"
#include "Robotik/Renderer/Managers/MeshManager.hpp"
#include "Robotik/Renderer/Managers/ShaderManager.hpp"
#include "Robotik/Renderer/RenderVisitor.hpp"
#include "Robotik/Renderer/Renderer.hpp"

#include <Eigen/Dense>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <iostream>

namespace robotik::renderer::application
{

// Vertex shader source
static const char* vertex_shader_source = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 FragPos;
out vec3 Normal;

void main()
{
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";

// Fragment shader source
static const char* fragment_shader_source = R"(
#version 330 core
in vec3 FragPos;
in vec3 Normal;

uniform vec3 color;

out vec4 FragColor;

void main()
{
    vec3 lightDir = normalize(vec3(1.0, 1.0, 1.0));
    vec3 normal = normalize(Normal);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * color;
    vec3 ambient = 0.3 * color;
    FragColor = vec4(ambient + diffuse, 1.0);
}
)";

// ----------------------------------------------------------------------------
RobotViewerApplication::RobotViewerApplication(Configuration const& p_config)
    : OpenGLApplication(p_config.window_width, p_config.window_height, true),
      m_config(p_config)
{
    setTitle(p_config.window_title + " - FPS: " + std::to_string(m_fps));
}

// ----------------------------------------------------------------------------
RobotViewerApplication::~RobotViewerApplication() = default;

// ----------------------------------------------------------------------------
bool RobotViewerApplication::run()
{
    return Application::run(m_config.target_fps, m_config.target_physics_hz);
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::onSetup()
{
    // Create perspective camera
    float aspect_ratio = static_cast<float>(m_config.window_width) /
                         static_cast<float>(m_config.window_height);
    m_camera =
        std::make_unique<PerspectiveCamera>(45.0f, aspect_ratio, 0.1f, 100.0f);
    m_camera->setPosition(Eigen::Vector3f(3.0f, 3.0f, 3.0f));
    m_camera->lookAt(Eigen::Vector3f(0.0f, 0.0f, 0.0f));

    // Create orbit controller for intuitive robot inspection
    m_camera_controller = std::make_unique<OrbitController>(
        *m_camera, Eigen::Vector3f(0.0f, 0.0f, 0.5f), 5.0f);

    // Create shader manager
    m_shader_manager = std::make_unique<ShaderManager>();
    if (!m_shader_manager->createProgram(
            "basic", vertex_shader_source, fragment_shader_source))
    {
        std::cerr << "Failed to create shader program: "
                  << m_shader_manager->error() << std::endl;
        return false;
    }

    // Use the shader
    if (!m_shader_manager->useProgram("basic"))
    {
        std::cerr << "Failed to use shader program" << std::endl;
        return false;
    }

    // Set up camera matrices
    int view_loc = m_shader_manager->getUniformLocation("view");
    int proj_loc = m_shader_manager->getUniformLocation("projection");
    m_shader_manager->setMatrix4f(view_loc, m_camera->viewMatrix().data());
    m_shader_manager->setMatrix4f(proj_loc,
                                  m_camera->projectionMatrix().data());

    // Create mesh manager
    m_mesh_manager = std::make_unique<MeshManager>();

    // Create example box mesh
    if (!m_mesh_manager->createBox("example_box", 1.0f, 1.0f, 1.0f))
    {
        std::cerr << "Failed to create box mesh: " << m_mesh_manager->error()
                  << std::endl;
        return false;
    }

    // Create cylinder for axes rendering
    if (!m_mesh_manager->createCylinder("axis_cylinder", 1.0f, 2.0f, 16))
    {
        std::cerr << "Failed to create cylinder mesh: "
                  << m_mesh_manager->error() << std::endl;
        return false;
    }

    // Try to load an STL file (example - this will fail if file doesn't exist)
    // Update this path to a real STL file path on your system
    STLLoader stl_loader;
    if (!m_mesh_manager->loadFromFile(
            "stl_model", "/tmp/example.stl", stl_loader, false))
    {
        std::cerr << "Note: Failed to load STL file (this is expected if file "
                     "doesn't exist): "
                  << m_mesh_manager->error() << std::endl;
        // Continue anyway - we'll just render the box
    }

    // Create renderer
    m_renderer = std::make_unique<Renderer>(*m_shader_manager);
    if (!m_renderer->initialize())
    {
        std::cerr << "Failed to initialize renderer: " << m_renderer->error()
                  << std::endl;
        return false;
    }

    // Try to load a robot URDF
    robotik::loader::URDFLoader parser;
    m_robot = parser.load("/home/qq/MyGitHub/Robotik/data/robot_6axis_urdf");
    if (!m_robot)
    {
        std::cerr << "Note: Failed to load URDF: " << parser.error()
                  << std::endl;
        std::cerr << "Continuing without robot (update path in "
                     "RobotViewerApplication.cpp)"
                  << std::endl;
    }
    else
    {
        std::cout << "Robot loaded successfully: " << m_robot->name()
                  << std::endl;
    }

    std::cout << "Rendering system initialized successfully!" << std::endl;
    return true;
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onTeardown() {}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onUpdate(float const dt)
{
    // Update camera controller
    m_camera_controller->update(dt);

    // Animate box rotation
    m_rotation_angle += dt * 0.5f; // Rotate 0.5 radians per second
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onPhysicUpdate(float const /* dt */) {}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onDrawScene()
{
    // Clear screen
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    // Use shader
    m_shader_manager->useProgram("basic");

    // Update camera matrices
    int view_loc = m_shader_manager->getUniformLocation("view");
    int proj_loc = m_shader_manager->getUniformLocation("projection");
    m_shader_manager->setMatrix4f(view_loc, m_camera->viewMatrix().data());
    m_shader_manager->setMatrix4f(proj_loc,
                                  m_camera->projectionMatrix().data());

    // Render grid
    m_renderer->renderGrid(Eigen::Vector3f(0.5f, 0.5f, 0.5f));

    // Render coordinate axes at origin
    const MeshManager::GPUMesh* axis_mesh =
        m_mesh_manager->getMesh("axis_cylinder");
    if (axis_mesh)
    {
        m_renderer->renderAxes(Eigen::Matrix4f::Identity(), 1.0f, axis_mesh);
    }

    // Render the rotating box
    const MeshManager::GPUMesh* box_mesh =
        m_mesh_manager->getMesh("example_box");
    if (box_mesh)
    {
        Eigen::Matrix4f box_transform = Eigen::Matrix4f::Identity();

        // Apply rotation around Z axis
        Eigen::Matrix3f rotation =
            Eigen::AngleAxisf(m_rotation_angle, Eigen::Vector3f::UnitZ())
                .toRotationMatrix();
        box_transform.block<3, 3>(0, 0) = rotation;

        // Position the box at (0, 0, 0.5) to sit on the grid
        box_transform(2, 3) = 0.5f;

        // Render with orange color
        m_renderer->render(
            box_mesh, box_transform, Eigen::Vector3f(1.0f, 0.5f, 0.0f));
    }

    // If STL model was loaded, render it next to the box
    const MeshManager::GPUMesh* stl_mesh = m_mesh_manager->getMesh("stl_model");
    if (stl_mesh)
    {
        Eigen::Matrix4f stl_transform = Eigen::Matrix4f::Identity();
        stl_transform(0, 3) = 2.0f; // Position at x = 2.0
        stl_transform(2, 3) = 0.5f; // Position at z = 0.5

        // Render with cyan color
        m_renderer->render(
            stl_mesh, stl_transform, Eigen::Vector3f(0.0f, 1.0f, 1.0f));
    }

    // Render robot if loaded
    if (m_robot && m_robot->hierarchy().hasRoot())
    {
        RenderVisitor visitor(*m_mesh_manager, *m_renderer, *m_shader_manager);
        m_robot->hierarchy().root().traverse(visitor);
    }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onDrawMenuBar()
{
    if (ImGui::BeginMenu("File"))
    {
        if (ImGui::MenuItem("Quit"))
        {
            halt();
        }
        ImGui::EndMenu();
    }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onDrawMainPanel()
{
    ImGui::Begin("Robot Control");
    ImGui::Text("Robot control panel");
    // TODO: Add robot control UI here
    ImGui::End();
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onFPSUpdated(size_t const p_fps)
{
    m_fps = p_fps;
    setTitle(m_config.window_title + " - FPS: " + std::to_string(p_fps));
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onKeyInput(int p_key,
                                        int p_scancode,
                                        int p_action,
                                        int p_mods)
{
    // Forward to camera controller
    m_camera_controller->handleKeyboard(p_key, p_action, p_mods);

    if (p_action == GLFW_PRESS)
    {
        switch (p_key)
        {
            case GLFW_KEY_ESCAPE:
                halt();
                break;
            default:
                break;
        }
    }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onMouseButton(int p_button,
                                           int p_action,
                                           int p_mods)
{
    m_camera_controller->handleMouseButton(p_button, p_action, p_mods);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onCursorPos(double p_xpos, double p_ypos)
{
    m_camera_controller->handleMouseMove(p_xpos, p_ypos);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onScroll(double p_xoffset, double p_yoffset)
{
    m_camera_controller->handleScroll(p_xoffset, p_yoffset);
}

} // namespace robotik::renderer::application