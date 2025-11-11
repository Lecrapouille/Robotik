/**
 * @file MainApplication.cpp
 * @brief Main application implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "MainApplication.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace robotik::application
{

// ----------------------------------------------------------------------------
MainApplication::MainApplication(Configuration const& p_config)
    : OpenGLApplication(p_config.window_width, p_config.window_height),
      m_config(p_config),
      m_path(p_config.search_paths)
{
    setTitle(p_config.window_title);
}

// ----------------------------------------------------------------------------
bool MainApplication::run()
{
    return OpenGLApplication::run(m_config.target_fps,
                                  m_config.target_physics_hz);
}

// ----------------------------------------------------------------------------
void MainApplication::setTitle(std::string const& p_title)
{
    m_title = p_title;
    OpenGLApplication::setTitle(m_title + " - FPS: " + std::to_string(m_fps));
}
// ----------------------------------------------------------------------------
bool MainApplication::onSetup()
{
    glEnable(GL_DEPTH_TEST);

    // Setup OpenGL shader programs
    if (!setupMainShader())
        return false;

    // Setup mesh manager (meshes from built-in and from URDF files)
    if (!setupMeshes())
        return false;

    // Setup render visitor (render the robot, grid, axes, joints ...)
    setupRender();

    // Create robot manager (manage the list of robots)
    if (!setupRobots())
        return false;

    // Setup physics simulator (simulate the physics of the robots)
    setupPhysicsSimulator();

    // Setup the ImGui view and its Model-View-Controller pattern
    if (!setupImGuiView())
        return false;

    return true;
}

// ----------------------------------------------------------------------------
bool MainApplication::setupMainShader()
{
    m_shader_manager = std::make_unique<renderer::ShaderManager>();

    constexpr char const* const main_shader_name = "main";
    static char const* const vertex_shader_source = R"(
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

    static char const* const fragment_shader_source = R"(
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

    // Create the main OpenGL shader program
    if (!m_shader_manager->createProgram(
            main_shader_name, vertex_shader_source, fragment_shader_source))
    {
        m_error =
            "Failed to create shader program: " + m_shader_manager->error();
        return false;
    }

    // Use the main OpenGL shader program
    if (!m_shader_manager->useProgram(main_shader_name))
    {
        m_error = "Failed to use shader program";
        return false;
    }

    // Get the uniform locations for the view and projection matrices
    m_view_uniform = m_shader_manager->getUniformLocation("view");
    m_projection_uniform = m_shader_manager->getUniformLocation("projection");
    return true;
}

// ----------------------------------------------------------------------------
bool MainApplication::setupMeshes()
{
    m_geometry_manager = std::make_unique<renderer::GeometryManager>(m_path);

    // Create grid mesh for ground plane
    if (!m_geometry_manager->createGrid("grid", 20, 1.0f))
    {
        m_error = "Failed to create grid mesh: " + m_geometry_manager->error();
        return false;
    }

    // Create axis cylinder mesh for coordinate axes visualization (used by
    // RenderVisitor::renderAxes() method)
    if (!m_geometry_manager->createCylinder("axis", 1.0f, 1.0f, 16))
    {
        m_error = "Failed to create axis mesh: " + m_geometry_manager->error();
        return false;
    }

    // Create sphere mesh for waypoint markers
    if (!m_geometry_manager->createSphere("waypoint", 0.05f, 16, 16))
    {
        m_error =
            "Failed to create waypoint mesh: " + m_geometry_manager->error();
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
void MainApplication::setupRender()
{
    assert(m_geometry_manager != nullptr && "Geometry manager not setup");
    assert(m_shader_manager != nullptr && "Shader manager not setup");

    m_render = std::make_unique<renderer::RenderVisitor>(*m_geometry_manager,
                                                         *m_shader_manager);

    // Configuration for the display of the joint axes
    m_render->setJointAxesOptions(m_config.show_joint_axes,
                                  m_config.show_revolute_joint_axes,
                                  m_config.show_prismatic_joint_axes);
}

// ----------------------------------------------------------------------------
void MainApplication::setupPhysicsSimulator()
{
    m_physics_simulator = std::make_unique<PhysicsSimulator>(
        1.0 / double(m_config.target_physics_hz), m_config.physics_gravity);
}

// ----------------------------------------------------------------------------
bool MainApplication::setupRobots()
{
    assert(m_geometry_manager != nullptr && "Geometry manager not setup");

    // Create the application controller
    m_application_controller =
        std::make_unique<ApplicationController>(m_config);

    auto& robot_manager = m_application_controller->getRobotManager();

    // Connect waypoint manager to robot manager signals
    robot_manager.onRobotAdded.connect(
        [this](Robot* robot)
        {
            m_application_controller->getWaypointManager().onRobotAdded(robot);
        });

    // Connect waypoint manager to robot manager signals
    robot_manager.onRobotRemoved.connect(
        [this](std::string const& name)
        {
            m_application_controller->getWaypointManager().onRobotRemoved(name);
        });

    // Create and initialize all robots
    std::string robot_name;
    for (auto const& urdf_file : m_config.urdf_files)
    {
        // Create from URDF file
        auto* robot = m_application_controller->loadRobot(urdf_file);
        if (robot == nullptr)
        {
            m_error = "Failed to load robot from URDF: " +
                      m_application_controller->error();
            return false;
        }

        robot_name = robot->name();

        // Create meshes for the display of the robot geometries
        for (auto const& geom_ref : robot->blueprint().geometries())
        {
            robotik::Geometry const& geom = geom_ref.get();
            m_geometry_manager->createMeshFromGeometry(geom, robot_name);
        }
    }

    robot_manager.selectRobot(robot_name);

    return true;
}

// ----------------------------------------------------------------------------
bool MainApplication::setupImGuiView()
{
    assert(m_application_controller != nullptr &&
           "Application controller not setup");

    m_imgui_view = std::make_unique<ImGuiView>(*m_application_controller,
                                               [this]() { halt(); });

    return true;
}

// ----------------------------------------------------------------------------
void MainApplication::onDrawMenuBar()
{
    m_imgui_view->onDrawMenuBar();
}

// ----------------------------------------------------------------------------
void MainApplication::onDrawMainPanel()
{
    assert(m_imgui_view != nullptr && "ImGui view not setup");
    m_imgui_view->onDrawRobotManagementWindow();
    m_imgui_view->onDrawCameraTargetWindow();
    m_imgui_view->onDrawTrajectoryWindow();
    m_imgui_view->onDrawTeachPendantWindow();
}

// ----------------------------------------------------------------------------
void MainApplication::onDrawScene()
{
    // OpenGL
    static const Eigen::Vector3f s_clear_color(0.1f, 0.1f, 0.1f);
    glClearColor(s_clear_color.x(), s_clear_color.y(), s_clear_color.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Set camera matrices first
    m_shader_manager->setMatrix4f(
        m_view_uniform,
        m_application_controller->getCameraController()
            .camera()
            .viewMatrix()
            .data());
    m_shader_manager->setMatrix4f(
        m_projection_uniform,
        m_application_controller->getCameraController()
            .camera()
            .projectionMatrix()
            .data());

    // Render the ground grid
    if (auto* grid_mesh = m_geometry_manager->getMesh("grid"))
    {
        m_render->renderGrid(grid_mesh, Eigen::Vector3f(0.5f, 0.5f, 0.5f));
    }

    // Render all robots
    auto const* selected_robot = m_application_controller->getCurrentRobot();
    for (auto const& [robot_name, robot_ptr] :
         m_application_controller->getRobotManager().robots())
    {
        // Render only enabled robots
        if (robot_ptr->blueprint().enabled())
        {
            const bool is_selected = selected_robot == robot_ptr.get();
            renderRobot(*static_cast<ControlledRobot*>(robot_ptr.get()),
                        is_selected);
        }
    }
}

// ----------------------------------------------------------------------------
void MainApplication::renderRobot(ControlledRobot const& p_robot,
                                  bool p_is_selected) const
{
    // Render the robot by traversing its blueprint.
    // Set the namespace of the robot for the render visitor: this avoids
    // conflicts with other robots.
    m_render->setMeshNamespace(p_robot.name());
    p_robot.traverse(*m_render);
    m_render->setMeshNamespace("");

    if (p_is_selected)
    {
        // Render end effector axes if available
        if (p_robot.end_effector)
        {
            Eigen::Matrix4f target_transform_f =
                p_robot.end_effector->worldTransform().cast<float>();
            if (auto* axis_mesh = m_geometry_manager->getMesh("axis"))
            {
                m_render->renderAxes(axis_mesh, target_transform_f, 0.1f);
            }
        }

        // Render waypoints
        renderWaypoints(p_robot);
    }
}

// ----------------------------------------------------------------------------
void MainApplication::renderWaypoints(ControlledRobot const& p_robot) const
{
    // Waypoints are linked to an end effector. The user shall select the end
    // effector first.
    if (!p_robot.end_effector)
        return;

    // Get the waypoint OpenGL mesh
    auto* waypoint_mesh = m_geometry_manager->getMesh("waypoint");
    if (!waypoint_mesh || !waypoint_mesh->is_loaded)
        return;

    // Get the waypoint manager responsible for robot's end effector
    auto const& wm = m_application_controller->getWaypointManager();
    if (!wm.hasWaypoints(*p_robot.end_effector))
        return;

    // Get cached transforms from waypoint manager
    auto const& cached_transforms = wm.getRenderCache(*p_robot.end_effector);
    if (cached_transforms.empty())
        return; // No cache yet

    auto const& tc = m_application_controller->getTrajectoryController();

    // Render each waypoint using cached transforms
    for (size_t i = 0; i < cached_transforms.size(); ++i)
    {
        // Extract position from cached transform
        Eigen::Vector3f waypoint_pos = cached_transforms[i].block<3, 1>(0, 3);

        // Create transform matrix for the waypoint sphere
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 1>(0, 3) = waypoint_pos;

        // Choose color: yellow for waypoints, green if it's the target
        Eigen::Vector3f color(1.0f, 1.0f, 0.0f); // Yellow
        if (tc.targetWaypointIndex() == static_cast<int>(i))
        {
            color = Eigen::Vector3f(0.0f, 1.0f, 0.0f); // Green for target
        }
        else if (tc.currentWaypointIndex() == static_cast<int>(i))
        {
            color = Eigen::Vector3f(0.0f, 0.8f, 1.0f); // Cyan for current
        }

        // Render waypoint sphere
        m_render->renderMesh(waypoint_mesh, transform, color);
    }
}

// ----------------------------------------------------------------------------
void MainApplication::onUpdate(float const p_dt)
{
    m_application_controller->update(p_dt);
}

// ----------------------------------------------------------------------------
void MainApplication::onPhysicUpdate(float const /* dt */)
{
    // auto* robot = m_robot_manager->currentRobot();
    // if (robot && robot->robot)
    //{
    //     m_physics_simulator.step(*robot->robot);
    //}
}

// ----------------------------------------------------------------------------
void MainApplication::onFPSUpdated(size_t const p_fps)
{
    m_fps = p_fps;
    OpenGLApplication::setTitle(m_title + " - FPS: " + std::to_string(p_fps));
}

// ----------------------------------------------------------------------------
void MainApplication::onWindowResize(int p_width, int p_height)
{
    m_application_controller->getCameraController().onWindowResize(p_width,
                                                                   p_height);

    m_shader_manager->setMatrix4f(
        m_view_uniform,
        m_application_controller->getCameraController()
            .camera()
            .viewMatrix()
            .data());
    m_shader_manager->setMatrix4f(
        m_projection_uniform,
        m_application_controller->getCameraController()
            .camera()
            .projectionMatrix()
            .data());
}

// ----------------------------------------------------------------------------
void MainApplication::onKeyInput(int key,
                                 int /* scancode */,
                                 int action,
                                 int /* mods */)
{
    if (action == GLFW_PRESS)
    {
        switch (key)
        {
            case GLFW_KEY_ESCAPE:
                halt();
                break;
            case GLFW_KEY_SPACE:
                switchNeutralPosition();
                break;
            case GLFW_KEY_V:
                switchVisibility();
                break;
            default:
                break;
        }
    }
}

// ----------------------------------------------------------------------------
void MainApplication::switchNeutralPosition() const
{
    auto* robot = m_application_controller->getCurrentRobot();
    if (robot == nullptr)
        return;

    robot->setNeutralPosition();
}

// ----------------------------------------------------------------------------
void MainApplication::switchVisibility() const
{
    auto* robot = m_application_controller->getCurrentRobot();
    if (robot == nullptr)
        return;

    robot->blueprint().enable(!robot->blueprint().enabled());
}

// ----------------------------------------------------------------------------
void MainApplication::onMouseButton(int p_button, int p_action, int p_mods)
{
    if (!isViewportHovered())
        return;

    m_application_controller->getCameraController().handleMouseButton(
        p_button, p_action, p_mods);
}

// ----------------------------------------------------------------------------
void MainApplication::onCursorPos(double p_xpos, double p_ypos)
{
    m_application_controller->getCameraController().handleMouseMove(p_xpos,
                                                                    p_ypos);
}

// ----------------------------------------------------------------------------
void MainApplication::onScroll(double xoffset, double yoffset)
{
    if (!isViewportHovered())
        return;

    m_application_controller->getCameraController().handleScroll(xoffset,
                                                                 yoffset);
}

} // namespace robotik::application
