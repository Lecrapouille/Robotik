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
const Eigen::Vector3f s_clear_color(0.1f, 0.1f, 0.1f);

// ----------------------------------------------------------------------------
Application::Application(Configuration const& p_config)
    : OpenGLApplication(p_config.window_width, p_config.window_height),
      m_config(p_config),
      m_path(p_config.search_paths),
      m_mesh_manager(m_path),
      m_physics_simulator(1.0 / double(p_config.target_physics_hz),
                          p_config.physics_gravity)
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
void Application::setupCamera()
{
    float aspect_ratio = static_cast<float>(m_config.window_width) /
                         static_cast<float>(m_config.window_height);

    m_perspective_camera = std::make_unique<renderer::PerspectiveCamera>(
        45.0f, aspect_ratio, 0.1f, 100.0f);
    m_perspective_camera->setPosition(Eigen::Vector3f(3.0f, 3.0f, 3.0f));
    m_perspective_camera->lookAt(Eigen::Vector3f(0.0f, 0.0f, 0.0f));

    m_orbit_controller = std::make_unique<renderer::OrbitController>(
        *m_perspective_camera, Eigen::Vector3f(0.0f, 0.0f, 0.5f), 5.0f);
}

// ----------------------------------------------------------------------------
bool Application::setupMainShader()
{
    constexpr char const* const main_shader_name = "main";

    if (!m_shader_manager.createProgram(
            main_shader_name, vertex_shader_source, fragment_shader_source))
    {
        m_error =
            "Failed to create shader program: " + m_shader_manager.error();
        return false;
    }

    if (!m_shader_manager.useProgram(main_shader_name))
    {
        m_error = "Failed to use shader program";
        return false;
    }

    m_view_uniform = m_shader_manager.getUniformLocation("view");
    m_projection_uniform = m_shader_manager.getUniformLocation("projection");
    return true;
}

// ----------------------------------------------------------------------------
bool Application::setupRenderer()
{
    m_renderer = std::make_unique<renderer::Renderer>(m_shader_manager);
    if (!m_renderer->initialize())
    {
        m_error = "Failed to initialize renderer: " + m_renderer->error();
        return false;
    }

    // Create axis meshes once for reuse
    if (!m_mesh_manager.hasMesh("axis_cylinder"))
    {
        m_mesh_manager.createCylinder("axis_cylinder", 0.01f, 0.1f);
    }
    if (!m_mesh_manager.hasMesh("axis_box"))
    {
        m_mesh_manager.createBox("axis_box", 0.01f, 0.01f, 0.05f);
    }

    return true;
}

// ----------------------------------------------------------------------------
bool Application::onSetup()
{
    glEnable(GL_DEPTH_TEST);

    setupCamera();

    if (!setupMainShader())
        return false;

    if (!setupRenderer())
        return false;

    // Load robots from URDF files
    for (auto const& urdf_file : m_config.urdf_files)
    {
        if (!setupRobot(urdf_file))
            return false;
    }

    // Initialize robot controller
    m_controller = std::make_unique<Controller>(m_robot_manager);
    if (!m_controller->initializeRobots(m_config.control_joint))
    {
        m_error = "Failed to initialize robot configurations";
        return false;
    }

    // Create HMI
    m_hmi = std::make_unique<HMI>(m_robot_manager,
                                  *m_controller,
                                  *m_orbit_controller,
                                  [this]() { halt(); });

    m_start_time = std::chrono::steady_clock::now();
    return true;
}

// ----------------------------------------------------------------------------
bool Application::setupRobot(std::string const& p_urdf_file)
{
    // Load robot from URDF file
    auto* robot = m_robot_manager.loadRobot(p_urdf_file);
    if (robot == nullptr)
    {
        m_error = "Failed to load robot from URDF: " + m_robot_manager.error();
        return false;
    }
    std::cout << "Loaded robot from: " << p_urdf_file << std::endl;
    std::cout << robotik::debug::printRobot(*robot, true) << std::endl;

    // Set initial joint values from configuration. If no configuration is
    // provided, set neutral position.
    if (auto const& joint_positions = m_config.joint_positions;
        !joint_positions.empty())
    {
        robot->setJointPositions(joint_positions);
        std::cout << "🤖 Set initial joint values from configuration"
                  << std::endl;
    }
    else
    {
        robot->setNeutralPosition();
        std::cout << "🤖 Set neutral position" << std::endl;
    }

    // Set camera target to the specified robot element.
    bool use_root_if_not_found = true;
    if (setCameraTarget(*robot, m_config.camera_target, use_root_if_not_found))
    {
        std::cout << "📷 Camera target: " << robot->camera_target->name()
                  << std::endl;
        robot->camera_tracking_enabled = true;
    }
    else
    {
        m_error = "📷 Failed to set camera target: " + m_config.camera_target;
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
bool Application::setCameraTarget(
    renderer::RobotManager::ControlledRobot& p_robot,
    std::string const& p_element_name,
    bool p_use_root_if_not_found) const
{
    p_robot.camera_target = nullptr;

    if (p_element_name.empty())
    {
        if (p_use_root_if_not_found)
            p_robot.camera_target = &p_robot.blueprint().root();
        else if (auto const& end_effectors = p_robot.blueprint().endEffectors();
                 !end_effectors.empty())
            p_robot.camera_target = &end_effectors[0].get();
    }
    else
    {
        p_robot.camera_target =
            Node::find(p_robot.blueprint().root(), p_element_name);
        if (!p_robot.camera_target && p_use_root_if_not_found)
            p_robot.camera_target = &p_robot.blueprint().root();
    }

    return p_robot.camera_target != nullptr;
}

// ----------------------------------------------------------------------------
void Application::onTeardown()
{
    m_robot_manager.clear();
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
    m_hmi->onDrawMainPanel();
}

// ----------------------------------------------------------------------------
void Application::onDrawScene()
{
    // Set camera matrices before rendering
    m_shader_manager.setMatrix4f(m_view_uniform,
                                 m_perspective_camera->viewMatrix().data());
    m_shader_manager.setMatrix4f(
        m_projection_uniform, m_perspective_camera->projectionMatrix().data());

    // OpenGL
    glClearColor(s_clear_color.x(), s_clear_color.y(), s_clear_color.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render the ground grid
    m_renderer->renderGrid(Eigen::Vector3f(0.5f, 0.5f, 0.5f));

    // Render the robots
    for (auto const& [robot_name, robot] : m_robot_manager.robots())
    {
        renderRobot(robot, robot_name == m_hmi->selectedRobot());
    }
}

// ----------------------------------------------------------------------------
void Application::renderRobot(
    renderer::RobotManager::ControlledRobot const& p_robot,
    bool p_is_selected)
{
    // Render nodes of the robot
    if (p_robot.is_visible && p_robot.blueprint().hasRoot())
    {
        // Const cast is safe here: rendering operations don't change logical
        // state
        renderer::RenderVisitor visitor(
            m_mesh_manager, *m_renderer, m_shader_manager);
        p_robot.blueprint().root().traverse(visitor);
    }

    // Only render IK target and trajectory for selected robot
    if (!p_is_selected)
        return;

    // Render IK target axes
    if (p_robot.control_mode ==
            renderer::RobotManager::ControlMode::INVERSE_KINEMATICS &&
        !p_robot.ik_target_poses.empty())
    {
        Transform target_transform =
            robotik::poseToTransform(p_robot.ik_target_poses[0]);
        if (auto* axis_mesh = m_mesh_manager.getMesh("axis_cylinder"))
        {
            m_renderer->renderAxes(
                target_transform.cast<float>(), 0.2f, axis_mesh);
        }
    }

    // Render trajectory path
    if (p_robot.control_mode ==
            renderer::RobotManager::ControlMode::TRAJECTORY &&
        p_robot.trajectory && p_robot.control_joint)
    {
        renderTrajectoryPath(p_robot);
    }
}

// ----------------------------------------------------------------------------
void Application::renderTrajectoryPath(
    renderer::RobotManager::ControlledRobot const& p_robot)
{
    if (!p_robot.trajectory || !p_robot.control_joint)
        return;

    constexpr size_t num_samples = 20;
    double duration = p_robot.trajectory->duration();

    std::vector<double> saved_positions;
    saved_positions.reserve(p_robot.blueprint().numJoints());
    p_robot.blueprint().forEachJoint(
        [&saved_positions](Joint const& joint, size_t /*index*/)
        { saved_positions.push_back(joint.position()); });

    auto* axis_mesh = m_mesh_manager.getMesh("axis_cylinder");
    if (!axis_mesh)
        return;

    for (size_t i = 0; i < num_samples; ++i)
    {
        double t = (double(i) / double(num_samples - 1)) * duration;
        auto states = p_robot.trajectory->evaluate(t);
        p_robot.blueprint().forEachJoint(
            [&states](Joint& joint, size_t index)
            { joint.position(states.position[index]); });
        m_renderer->renderAxes(
            p_robot.control_joint->worldTransform().cast<float>(),
            0.15f,
            axis_mesh);
    }

    p_robot.blueprint().forEachJoint(
        [&saved_positions](Joint& joint, size_t index)
        { joint.position(saved_positions[index]); });
}

// ----------------------------------------------------------------------------
void Application::onUpdate(float const dt)
{
    // Camera update
    m_orbit_controller->update(dt);

    // Camera tracking update
    for (auto const& [_, it] : m_robot_manager.robots())
    {
        if (it.camera_tracking_enabled && it.camera_target)
        {
            Eigen::Vector3d target_pos =
                it.camera_target->worldTransform().block<3, 1>(0, 3);
            m_orbit_controller->setTarget(target_pos.cast<float>());
        }
    }

    // Animation timing update
    auto current_time = std::chrono::steady_clock::now();
    double elapsed_seconds =
        std::chrono::duration<double>(current_time - m_start_time).count();

    // Robot controller update
    if (m_controller)
    {
        m_controller->update(elapsed_seconds, static_cast<double>(dt));
    }
}

// ----------------------------------------------------------------------------
void Application::onPhysicUpdate(float const /* dt */)
{
    // auto* controlled_robot = m_robot_manager.currentRobot();
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

    m_shader_manager.setMatrix4f(m_view_uniform,
                                 m_perspective_camera->viewMatrix().data());
    m_shader_manager.setMatrix4f(
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
void Application::switchNeutralPosition() const
{
    auto* robot = m_robot_manager.currentRobot();
    if (robot == nullptr)
        return;
    robot->setNeutralPosition();
}

// ----------------------------------------------------------------------------
void Application::switchVisibility() const
{
    auto* robot = m_robot_manager.currentRobot();
    if (robot == nullptr)
        return;
    robot->is_visible = !robot->is_visible;
}

// ----------------------------------------------------------------------------
void Application::onMouseButton(int p_button, int p_action, int p_mods)
{
    if (!isViewportHovered())
        return;
    m_orbit_controller->handleMouseButton(p_button, p_action, p_mods);
}

// ----------------------------------------------------------------------------
void Application::onCursorPos(double p_xpos, double p_ypos)
{
    m_orbit_controller->handleMouseMove(p_xpos, p_ypos);
}

// ----------------------------------------------------------------------------
void Application::onScroll(double xoffset, double yoffset)
{
    if (!isViewportHovered())
        return;
    m_orbit_controller->handleScroll(xoffset, yoffset);
}

} // namespace robotik::application
