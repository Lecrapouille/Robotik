#include "Application.hpp"

#include "Robotik/Core/Common/Conversions.hpp"
#include "Robotik/Core/Robot/Debug.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>

namespace robotik::application
{

// ----------------------------------------------------------------------------
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

// ----------------------------------------------------------------------------
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

// ----------------------------------------------------------------------------
static const Eigen::Vector3f s_clear_color(0.1f, 0.1f, 0.1f);

// ----------------------------------------------------------------------------
Application::Application(Configuration const& p_config)
    : OpenGLApplication(p_config.window_width, p_config.window_height),
      m_config(p_config),
      m_path(p_config.search_paths)
{
    setTitle(p_config.window_title);
}

// ----------------------------------------------------------------------------
bool Application::run()
{
    return OpenGLApplication::run(m_config.target_fps,
                                  m_config.target_physics_hz);
}

// ----------------------------------------------------------------------------
void Application::setTitle(std::string const& p_title)
{
    m_title = p_title;
    OpenGLApplication::setTitle(m_title + " - FPS: " + std::to_string(m_fps));
}
// ----------------------------------------------------------------------------
bool Application::onSetup()
{
    glEnable(GL_DEPTH_TEST);

    // Setup the OpenGL camera and its controller
    setupCamera();

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

    // Setup the Human-Machine Interface and its Model-View-Controller pattern
    if (!setupHMI())
        return false;

    return true;
}

// ----------------------------------------------------------------------------
void Application::setupCamera()
{
    float aspect_ratio = static_cast<float>(m_config.window_width) /
                         static_cast<float>(m_config.window_height);

    m_perspective_camera = std::make_unique<renderer::PerspectiveCamera>(
        45.0f, aspect_ratio, 0.1f, 100.0f);
    m_perspective_camera->setPosition(Eigen::Vector3f(3.0f, 3.0f, 3.0f));
    m_perspective_camera->lookAt(Eigen::Vector3f(0.0f, 0.0f, 0.0f));

    // Create both controllers
    Eigen::Vector3f initial_target(0.0f, 0.0f, 0.5f);
    float initial_distance = 5.0f;

    m_orbit_controller = std::make_unique<renderer::OrbitController>(
        *m_perspective_camera, initial_target, initial_distance);

    m_drag_controller = std::make_unique<renderer::DragController>(
        *m_perspective_camera, initial_target, initial_distance);

    // Start with OrbitController
    m_camera_controller = m_orbit_controller.get();
}

// ----------------------------------------------------------------------------
bool Application::setupMainShader()
{
    m_shader_manager = std::make_unique<renderer::ShaderManager>();

    constexpr char const* const main_shader_name = "main";

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
bool Application::setupMeshes()
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
    if (!m_geometry_manager->createCylinder("axis", 0.01f, 0.1f, 16))
    {
        m_error = "Failed to create axis mesh: " + m_geometry_manager->error();
        return false;
    }

    // Create joint axis cylinder mesh for joint visualization (used by
    // Application::renderJointAxes() method)
    if (!m_geometry_manager->createCylinder("revolute", 0.015f, 0.3f, 16))
    {
        m_error =
            "Failed to create joint axis mesh: " + m_geometry_manager->error();
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
void Application::setupRender()
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
void Application::setupPhysicsSimulator()
{
    m_physics_simulator = std::make_unique<PhysicsSimulator>(
        1.0 / double(m_config.target_physics_hz), m_config.physics_gravity);
}

// ----------------------------------------------------------------------------
bool Application::setupRobots()
{
    assert(m_geometry_manager != nullptr && "Geometry manager not setup");

    // Create the Model-View-Controller's controller
    m_robot_manager = std::make_unique<renderer::RobotManager>();
    m_controller = std::make_unique<Controller>(*m_robot_manager);

    // Create and initialize all robots
    for (auto const& urdf_file : m_config.urdf_files)
    {
        // Load robot from URDF file
        auto* robot = m_robot_manager->loadRobot<ControlledRobot>(urdf_file);
        if (robot == nullptr)
        {
            m_error =
                "Failed to load robot from URDF: " + m_robot_manager->error();
            return false;
        }
        std::cout << "Loaded robot from: " << urdf_file << std::endl;
        std::cout << robotik::debug::printRobot(*robot, true) << std::endl;

        if (!m_controller->initializeRobot(robot,
                                           m_config.control_link,
                                           m_config.joint_positions,
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
bool Application::setupHMI()
{
    assert(m_controller != nullptr && "Controller not setup");
    assert(m_camera_controller != nullptr && "Camera controller not setup");
    assert(m_robot_manager != nullptr && "Robot manager not setup");

    // Create the HMI view (view of the Model-View-Controller pattern)
    // Note: m_controller was already created in setupRobots()
    m_hmi = std::make_unique<HMI>(
        *m_robot_manager, *m_controller, *this, [this]() { halt(); });

    return true;
}

// ----------------------------------------------------------------------------
void Application::onDrawMenuBar()
{
    if (!m_hmi)
        return;
    m_hmi->onDrawMenuBar();
}

// ----------------------------------------------------------------------------
void Application::onDrawMainPanel()
{
    if (!m_hmi)
        return;

    // Draw dockable windows
    m_hmi->onDrawRobotManagementWindow();
    m_hmi->onDrawCameraTargetWindow();

    // Draw main teach pendant window
    m_hmi->onDrawMainPanel();
}

// ----------------------------------------------------------------------------
void Application::onDrawScene()
{
    // Set camera matrices before rendering
    m_shader_manager->setMatrix4f(m_view_uniform,
                                  m_perspective_camera->viewMatrix().data());
    m_shader_manager->setMatrix4f(
        m_projection_uniform, m_perspective_camera->projectionMatrix().data());

    // OpenGL
    glClearColor(s_clear_color.x(), s_clear_color.y(), s_clear_color.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render the ground grid
    if (auto* grid_mesh = m_geometry_manager->getMesh("grid"))
    {
        m_render->renderGrid(grid_mesh, Eigen::Vector3f(0.5f, 0.5f, 0.5f));
    }

    // Render the robots
    // Note: Robots are now stored in controller as ControlledRobot
    // We still need to get them from somewhere for rendering
    // For now, let's iterate through the robot manager's basic robots
    for (auto const& [robot_name, robot_ptr] : m_robot_manager->robots())
    {
        bool const selected = (robot_name == m_hmi->selectedRobot());
        // Get the controlled robot from controller
        auto* controlled_robot = getControlledRobot(robot_name);
        if (controlled_robot)
        {
            renderRobot(*controlled_robot, selected);
        }
    }
}

// ----------------------------------------------------------------------------
void Application::renderRobot(ControlledRobot const& p_robot,
                              bool p_is_selected)
{
    // Render nodes of the robot (always visible in new architecture)
    auto& blueprint = p_robot.blueprint();
    if (!blueprint.enabled())
        return;
    if (blueprint.hasRoot())
    {
        blueprint.root().traverse(*m_render);
    }

    // Only render teach pendant visualization for selected robot
    if (!p_is_selected)
        return;

    // Render end effector axes if available
    if (p_robot.end_effector)
    {
        Eigen::Matrix4f target_transform_f =
            p_robot.end_effector->worldTransform().cast<float>();
        if (auto* axis_mesh = m_geometry_manager->getMesh("axis"))
        {
            m_render->renderAxes(axis_mesh, target_transform_f, 1.0f);
        }
    }
}

// ----------------------------------------------------------------------------
void Application::onUpdate(float const dt)
{
    // Camera update
    if (m_camera_controller)
    {
        m_camera_controller->update(dt);
    }

    // Camera tracking update - get controlled robots from controller
    // Only apply tracking if user is not currently interacting with the camera
    bool user_interacting = false;
    if (m_orbit_controller && m_camera_controller == m_orbit_controller.get())
    {
        user_interacting = m_orbit_controller->isUserInteracting();
    }

    if (!user_interacting)
    {
        for (auto const& [robot_name, robot_ptr] : m_robot_manager->robots())
        {
            auto const* controlled_robot = getControlledRobot(robot_name);
            if (controlled_robot && controlled_robot->camera_tracking_enabled &&
                controlled_robot->camera_target)
            {
                Eigen::Vector3d target_pos =
                    controlled_robot->camera_target->worldTransform()
                        .block<3, 1>(0, 3);
                if (m_camera_controller)
                {
                    m_camera_controller->setTarget(target_pos.cast<float>());
                }
            }
        }
    }

    // Robot controller update
    if (m_controller)
    {
        m_controller->update(static_cast<double>(dt));
    }
}

// ----------------------------------------------------------------------------
void Application::onPhysicUpdate(float const /* dt */)
{
    // auto* controlled_robot = m_robot_manager->currentRobot();
    // if (controlled_robot && controlled_robot->robot)
    //{
    //     m_physics_simulator.step(*controlled_robot->robot);
    //}
}

// ----------------------------------------------------------------------------
void Application::onFPSUpdated(size_t const p_fps)
{
    m_fps = p_fps;
    OpenGLApplication::setTitle(m_title + " - FPS: " + std::to_string(p_fps));
}

// ----------------------------------------------------------------------------
void Application::onWindowResize(int p_width, int p_height)
{
    auto aspect_ratio =
        static_cast<float>(p_width) / static_cast<float>(p_height);
    m_perspective_camera->setAspectRatio(aspect_ratio);

    m_shader_manager->setMatrix4f(m_view_uniform,
                                  m_perspective_camera->viewMatrix().data());
    m_shader_manager->setMatrix4f(
        m_projection_uniform, m_perspective_camera->projectionMatrix().data());
}

// ----------------------------------------------------------------------------
void Application::onKeyInput(int key,
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
Application::getControlledRobot(std::string const& p_robot_name) const
{
    return m_robot_manager->getRobot<ControlledRobot>(p_robot_name);
}

// ----------------------------------------------------------------------------
void Application::switchNeutralPosition() const
{
    auto* robot = m_robot_manager->currentRobot();
    if (robot == nullptr)
        return;
    robot->setNeutralPosition();
}

// ----------------------------------------------------------------------------
void Application::switchVisibility() const
{
    auto* controlled_robot = getControlledRobot(m_hmi->selectedRobot());
    if (controlled_robot == nullptr)
        return;
    controlled_robot->blueprint().enable(
        !controlled_robot->blueprint().enabled());
}

// ----------------------------------------------------------------------------
void Application::onMouseButton(int p_button, int p_action, int p_mods)
{
    if (!isViewportHovered())
        return;
    if (m_camera_controller)
    {
        m_camera_controller->handleMouseButton(p_button, p_action, p_mods);
    }
}

// ----------------------------------------------------------------------------
void Application::onCursorPos(double p_xpos, double p_ypos)
{
    if (m_camera_controller)
    {
        m_camera_controller->handleMouseMove(p_xpos, p_ypos);
    }
}

// ----------------------------------------------------------------------------
void Application::onScroll(double xoffset, double yoffset)
{
    if (!isViewportHovered())
        return;
    if (m_camera_controller)
    {
        m_camera_controller->handleScroll(xoffset, yoffset);
    }
}

// ----------------------------------------------------------------------------
Eigen::Vector3f Application::getCameraTarget() const
{
    // Try to get target from selected robot
    if (m_hmi && !m_hmi->selectedRobot().empty())
    {
        auto const* controlled_robot =
            getControlledRobot(m_hmi->selectedRobot());
        if (controlled_robot && controlled_robot->camera_target)
        {
            Eigen::Vector3d target_pos =
                controlled_robot->camera_target->worldTransform().block<3, 1>(
                    0, 3);
            return target_pos.cast<float>();
        }
    }

    // Try to get from any robot
    for (auto const& [robot_name, robot_ptr] : m_robot_manager->robots())
    {
        auto const* controlled_robot = getControlledRobot(robot_name);
        if (controlled_robot && controlled_robot->camera_target)
        {
            Eigen::Vector3d target_pos =
                controlled_robot->camera_target->worldTransform().block<3, 1>(
                    0, 3);
            return target_pos.cast<float>();
        }
    }

    // Default target
    return Eigen::Vector3f(0.0f, 0.0f, 0.5f);
}

// ----------------------------------------------------------------------------
void Application::switchToOrbitController()
{
    if (!m_orbit_controller)
        return;

    Eigen::Vector3f target = getCameraTarget();
    float distance = 5.0f;

    // If we have a current camera controller, try to preserve distance
    if (m_camera_controller && m_orbit_controller.get() == m_camera_controller)
    {
        // Already using orbit controller, just update target
        m_orbit_controller->setTarget(target);
        m_camera_controller = m_orbit_controller.get();
        return;
    }

    // Recreate orbit controller with current target
    m_orbit_controller = std::make_unique<renderer::OrbitController>(
        *m_perspective_camera, target, distance);
    m_camera_controller = m_orbit_controller.get();
}

// ----------------------------------------------------------------------------
void Application::switchToDragController()
{
    if (!m_drag_controller)
        return;

    // Calculate target and distance from current camera position
    // This preserves the current view when switching to drag controller
    Eigen::Vector3f target = m_perspective_camera->target();
    Eigen::Vector3f position = m_perspective_camera->position();
    Eigen::Vector3f direction = target - position;
    float distance = direction.norm();

    // If distance is too small, use default values
    if (distance < 0.1f)
    {
        target = getCameraTarget();
        distance = 5.0f;
    }

    // Always recreate drag controller to capture the current camera direction
    // This is necessary when switching between orthographic views (Top, Bottom,
    // etc.) The constructor will automatically preserve the current camera
    // direction
    m_drag_controller = std::make_unique<renderer::DragController>(
        *m_perspective_camera, target, distance);
    m_camera_controller = m_drag_controller.get();
}

// ----------------------------------------------------------------------------
void Application::setTopView()
{
    Eigen::Vector3f target = getCameraTarget();
    float distance = 5.0f;

    // Looking down from Z+ (top view)
    Eigen::Vector3f position = target + Eigen::Vector3f(0.0f, 0.0f, distance);
    m_perspective_camera->lookAt(
        position, target, Eigen::Vector3f(0.0f, 1.0f, 0.0f));

    // Switch to drag controller to preserve this view
    switchToDragController();
}

// ----------------------------------------------------------------------------
void Application::setBottomView()
{
    Eigen::Vector3f target = getCameraTarget();
    float distance = 5.0f;

    // Looking up from Z- (bottom view)
    Eigen::Vector3f position = target + Eigen::Vector3f(0.0f, 0.0f, -distance);
    m_perspective_camera->lookAt(
        position, target, Eigen::Vector3f(0.0f, 1.0f, 0.0f));

    // Switch to drag controller to preserve this view
    switchToDragController();
}

// ----------------------------------------------------------------------------
void Application::setFrontView()
{
    Eigen::Vector3f target = getCameraTarget();
    float distance = 5.0f;

    // Looking from Y+ (front view)
    Eigen::Vector3f position = target + Eigen::Vector3f(0.0f, distance, 0.0f);
    m_perspective_camera->lookAt(
        position, target, Eigen::Vector3f(0.0f, 0.0f, 1.0f));

    // Switch to drag controller to preserve this view
    switchToDragController();
}

// ----------------------------------------------------------------------------
void Application::setBackView()
{
    Eigen::Vector3f target = getCameraTarget();
    float distance = 5.0f;

    // Looking from Y- (back view)
    Eigen::Vector3f position = target + Eigen::Vector3f(0.0f, -distance, 0.0f);
    m_perspective_camera->lookAt(
        position, target, Eigen::Vector3f(0.0f, 0.0f, 1.0f));

    // Switch to drag controller to preserve this view
    switchToDragController();
}

// ----------------------------------------------------------------------------
void Application::setRightView()
{
    Eigen::Vector3f target = getCameraTarget();
    float distance = 5.0f;

    // Looking from X+ (right view)
    Eigen::Vector3f position = target + Eigen::Vector3f(distance, 0.0f, 0.0f);
    m_perspective_camera->lookAt(
        position, target, Eigen::Vector3f(0.0f, 0.0f, 1.0f));

    // Switch to drag controller to preserve this view
    switchToDragController();
}

// ----------------------------------------------------------------------------
void Application::setLeftView()
{
    Eigen::Vector3f target = getCameraTarget();
    float distance = 5.0f;

    // Looking from X- (left view)
    Eigen::Vector3f position = target + Eigen::Vector3f(-distance, 0.0f, 0.0f);
    m_perspective_camera->lookAt(
        position, target, Eigen::Vector3f(0.0f, 0.0f, 1.0f));

    // Switch to drag controller to preserve this view
    switchToDragController();
}

} // namespace robotik::application
