/**
 * @file MainApplication.cpp
 * @brief Main application implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "MainApplication.hpp"

#include "Robotik/Core/Common/Conversions.hpp"
#include "Robotik/Core/Robot/Debug.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>

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

    // Create the robot controller
    m_robot_controller = std::make_unique<RobotController>(m_config);

    // Connect waypoint manager to robot manager signals
    m_robot_controller->getRobotManager().onRobotAdded.connect(
        [this](Robot* robot)
        { m_robot_controller->getWaypointManager().onRobotAdded(robot); });
    m_robot_controller->getRobotManager().onRobotRemoved.connect(
        [this](std::string const& name)
        { m_robot_controller->getWaypointManager().onRobotRemoved(name); });

    // Create and initialize all robots
    for (auto const& urdf_file : m_config.urdf_files)
    {
        // Load robot from URDF file
        auto* robot =
            m_robot_controller->getRobotManager().loadRobot<ControlledRobot>(
                urdf_file);
        if (robot == nullptr)
        {
            m_error = "Failed to load robot from URDF: " +
                      m_robot_controller->getRobotManager().error();
            return false;
        }
        std::cout << "Loaded robot from: " << urdf_file << std::endl;
        std::cout << robotik::debug::printRobot(*robot, true) << std::endl;

        // Set the robot configuration (home position, end effector selection,
        // camera target selection)
        if (!m_robot_controller->initializeRobot(*robot,
                                                 m_config.control_link,
                                                 m_config.home_position,
                                                 m_config.camera_target))
        {
            m_error = "Failed to initialize robot configuration";
            return false;
        }

        // Get controlled robot to access its blueprint for mesh creation
        auto* controlled_robot = getControlledRobot(robot->name());
        if (controlled_robot != nullptr)
        {
            // Create meshes for all geometries of the robot
            for (auto const& geom_ref :
                 controlled_robot->blueprint().geometries())
            {
                robotik::Geometry const& geom = geom_ref.get();
                m_geometry_manager->createMeshFromGeometry(geom);
            }
        }
    }

    return true;
}

// ----------------------------------------------------------------------------
bool MainApplication::setupImGuiView()
{
    assert(m_robot_controller != nullptr && "Application controller not setup");

    m_imgui_view =
        std::make_unique<ImGuiView>(*m_robot_controller, [this]() { halt(); });

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
    m_imgui_view->onDrawMainPanel();
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
        m_robot_controller->getCameraController().camera().viewMatrix().data());
    m_shader_manager->setMatrix4f(m_projection_uniform,
                                  m_robot_controller->getCameraController()
                                      .camera()
                                      .projectionMatrix()
                                      .data());

    // Render the ground grid
    if (auto* grid_mesh = m_geometry_manager->getMesh("grid"))
    {
        m_render->renderGrid(grid_mesh, Eigen::Vector3f(0.5f, 0.5f, 0.5f));
    }

    // Render all robots
    for (auto const& [robot_name, robot_ptr] :
         m_robot_controller->getRobotManager().robots())
    {
        if (auto const* robot = getControlledRobot(robot_name);
            robot != nullptr && robot->blueprint().enabled())
        {
            bool const is_selected =
                (robot_name == m_imgui_view->selectedRobot());
            renderRobot(*robot, is_selected);
        }
    }
}

// ----------------------------------------------------------------------------
void MainApplication::renderRobot(ControlledRobot const& p_robot,
                                  bool p_is_selected) const
{
    // Render the robot by traversing its blueprint.
    p_robot.traverse(*m_render);

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
    auto& wm = m_robot_controller->getWaypointManager();
    if (!wm.hasWaypoints(*p_robot.end_effector))
        return;

    // Get cached transforms from waypoint manager
    auto const& cached_transforms = wm.getRenderCache(*p_robot.end_effector);
    if (cached_transforms.empty())
        return; // No cache yet

    auto& tc = m_robot_controller->getTrajectoryController();

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
void MainApplication::onUpdate(float const dt)
{
    // Camera tracking - use active robot
    auto* robot = m_robot_controller->getCurrentRobot();
    if (robot && robot->camera_tracking_enabled && robot->camera_target)
    {
        Eigen::Vector3f target_pos = robot->camera_target->worldTransform()
                                         .block<3, 1>(0, 3)
                                         .cast<float>();

        m_robot_controller->getCameraController().setTrackingEnabled(true);
        m_robot_controller->getCameraController().update(dt, &target_pos);
    }

    m_robot_controller->getCameraController().setTrackingEnabled(false);
    m_robot_controller->getCameraController().update(dt, nullptr);

    // Update trajectory controller (single controller for current robot)
    auto* trajectory_controller =
        &m_robot_controller->getTrajectoryController();
    if (trajectory_controller->isPlaying())
    {
        trajectory_controller->update(static_cast<double>(dt));
        // Check if still playing after update (might have stopped)
        if (trajectory_controller->isPlaying())
        {
            auto target = trajectory_controller->getCurrentTarget();
            if (!target.empty() && robot)
            {
                robot->setJointPositions(target);
            }
        }
    }
    else
    {
        auto& robots = m_robot_controller->getRobotManager().robots();
        for (auto& [name, robot_ptr] : robots)
        {
            if (auto* controlled_robot =
                    dynamic_cast<ControlledRobot*>(robot_ptr.get());
                controlled_robot != nullptr &&
                controlled_robot->state == ControlledRobot::State::PLAYING)
            {
                controlled_robot->state = ControlledRobot::State::IDLE;
            }
        }
    }
}

// ----------------------------------------------------------------------------
void MainApplication::onPhysicUpdate(float const /* dt */)
{
    // auto* controlled_robot = m_robot_manager->currentRobot();
    // if (controlled_robot && controlled_robot->robot)
    //{
    //     m_physics_simulator.step(*controlled_robot->robot);
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
    m_robot_controller->getCameraController().onWindowResize(p_width, p_height);

    m_shader_manager->setMatrix4f(
        m_view_uniform,
        m_robot_controller->getCameraController().camera().viewMatrix().data());
    m_shader_manager->setMatrix4f(m_projection_uniform,
                                  m_robot_controller->getCameraController()
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
ControlledRobot*
MainApplication::getControlledRobot(std::string const& p_robot_name) const
{
    return m_robot_controller->getRobotManager().getRobot<ControlledRobot>(
        p_robot_name);
}

// ----------------------------------------------------------------------------
void MainApplication::switchNeutralPosition() const
{
    auto* robot = m_robot_controller->getCurrentRobot();
    if (robot == nullptr)
        return;
    robot->setNeutralPosition();
}

// ----------------------------------------------------------------------------
void MainApplication::switchVisibility() const
{
    auto* controlled_robot = getControlledRobot(m_imgui_view->selectedRobot());
    if (controlled_robot == nullptr)
        return;

    controlled_robot->blueprint().enable(
        !controlled_robot->blueprint().enabled());
}

// ----------------------------------------------------------------------------
void MainApplication::onMouseButton(int p_button, int p_action, int p_mods)
{
    if (!isViewportHovered())
        return;

    m_robot_controller->getCameraController().handleMouseButton(
        p_button, p_action, p_mods);
}

// ----------------------------------------------------------------------------
void MainApplication::onCursorPos(double p_xpos, double p_ypos)
{
    m_robot_controller->getCameraController().handleMouseMove(p_xpos, p_ypos);
}

// ----------------------------------------------------------------------------
void MainApplication::onScroll(double xoffset, double yoffset)
{
    if (!isViewportHovered())
        return;

    m_robot_controller->getCameraController().handleScroll(xoffset, yoffset);
}

} // namespace robotik::application
