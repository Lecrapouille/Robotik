/**
 * @file RobotViewerApplication.cpp
 * @brief Robot viewer application class for the 3D viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "RobotViewerApplication.hpp"

#include "Robotik/Core/Conversions.hpp"
#include "Robotik/Core/Exception.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>

namespace robotik::viewer
{

// Background color
const Eigen::Vector3f s_clear_color(0.2f, 0.3f, 0.3f);

} // namespace robotik::viewer

namespace robotik::viewer
{

// ----------------------------------------------------------------------------
RobotViewerApplication::RobotViewerApplication(Configuration const& p_config)
    : m_config(p_config),
      m_window(p_config.window_width, p_config.window_height),
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
    // Initialize OpenGL window
    if (!m_window.initialize(std::bind(&RobotViewerApplication::onKeyInput,
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
                                       std::placeholders::_2)))
    {
        m_error = m_window.error();
        return false;
    }

    // Set mesh base path if specified
    if (!m_config.search_paths.empty())
    {
        m_mesh_manager.setSearchPaths(m_config.search_paths);
    }

    // Setup shaders
    if (!setupShaders("default"))
    {
        return false;
    }

    // Initialize geometry renderer
    if (!m_geometry_renderer.initialize())
    {
        m_error = "Failed to initialize geometry renderer: " +
                  m_geometry_renderer.error();
        return false;
    }

    // Load robot from the specified URDF file
    auto* controlled_robot = m_robot_manager.loadRobot(m_config.urdf_file);
    if (controlled_robot == nullptr)
    {
        m_error = "Failed to load robot from URDF: " + m_robot_manager.error();
        return false;
    }
    else
    {
        std::cout << "Loaded robot from: " << m_config.urdf_file << std::endl;
        std::cout << debug::printRobot(*controlled_robot->robot, true)
                  << std::endl;
    }

    // Set initial joint values to the specified position
    if (!m_config.joint_positions.empty())
    {
        if (!controlled_robot->robot->setJointValues(m_config.joint_positions))
        {
            m_error = "Failed to set joint values for robot: " +
                      controlled_robot->robot->name();
            return false;
        }
    }

    // Set control joint (end effector) for inverse kinematics
    if (!setControlJoint(*controlled_robot, m_config.control_joint))
    {
        m_error = "🤖 Failed to set control joint: " + m_config.control_joint +
                  ": " + m_error;
        return false;
    }
    else
    {
        std::cout << "🤖 Controlled joint: "
                  << controlled_robot->control_joint->name() << std::endl;
    }

    // Set camera target (base link or tool center point) to track
    if (!initCameraView(*controlled_robot, m_config.camera_target))
    {
        m_error = "📷 Failed to set camera target: " + m_config.camera_target +
                  ": " + m_error;
        return false;
    }
    else
    {
        std::cout << "📷 Camera target: "
                  << controlled_robot->camera_target->name() << std::endl;
    }

    // Initialize OpenGL states
    glEnable(GL_DEPTH_TEST);

    return true;
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::setControlJoint(
    RobotManager::ControlledRobot& p_controlled_robot,
    std::string const& p_control_joint_name)
{
    try
    {
        if (!p_control_joint_name.empty())
        {
            p_controlled_robot.control_joint = scene::Node::find(
                p_controlled_robot.robot->root(), p_control_joint_name);
        }
        else
        {
            std::vector<std::reference_wrapper<Link const>> p_end_effectors;
            if (p_controlled_robot.robot->findEndEffectors(p_end_effectors))
            {
                p_controlled_robot.control_joint = &p_end_effectors[0].get();
            }
        }
        return p_controlled_robot.control_joint != nullptr;
    }
    catch (robotik::RobotikException const& e)
    {
        p_controlled_robot.control_joint = nullptr;
        m_error = e.what();
        return false;
    }
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::initCameraView(
    RobotManager::ControlledRobot& p_controlled_robot,
    std::string const& p_look_at_joint_name)
{
    try
    {
        if (!p_look_at_joint_name.empty())
        {
            p_controlled_robot.camera_target = scene::Node::find(
                p_controlled_robot.robot->root(), p_look_at_joint_name);
        }
        else
        {
            p_controlled_robot.camera_target =
                &p_controlled_robot.robot->root();
        }
    }
    catch (RobotikException const& e)
    {
        p_controlled_robot.camera_target = nullptr;
        m_error = e.what();
        return false;
    }

    // Set the camera view to the target joint.
    if (p_controlled_robot.camera_target != nullptr)
    {
        m_camera.setView(
            m_config.camera_view,
            utils::getTranslation(
                p_controlled_robot.camera_target->worldTransform()));
        return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::setupShaders(std::string const& p_program_name)
{
    // Simple vertex shader with lighting.
    const std::string vertex_shader = R"(
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

    // Simple fragment shader with lighting.
    const std::string fragment_shader = R"(
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

    // Setup shaders
    if (!m_shader_manager.createProgram(
            p_program_name, vertex_shader, fragment_shader))
    {
        m_error = "Failed to create shader program " + p_program_name + ": " +
                  m_shader_manager.error();
        return false;
    }

    // Use shader program and initialize shader uniform locations. Since we have
    // only one shader program, we can initialize the uniform locations once.
    m_shader_manager.useProgram(p_program_name);
    m_model_uniform = m_shader_manager.getUniformLocation("model");
    m_color_uniform = m_shader_manager.getUniformLocation("color");
    m_projection_uniform = m_shader_manager.getUniformLocation("projection");
    m_view_uniform = m_shader_manager.getUniformLocation("view");

    return true;
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onCleanup()
{
    m_robot_manager.clear();
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onDraw()
{
    // Clear screen
    glClearColor(s_clear_color.x(), s_clear_color.y(), s_clear_color.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Update camera target position, tracking one element of the robot
    auto const* controlled_robot = m_robot_manager.currentRobot();
    Eigen::Vector3d target_pos; // FIXME: fix Eigen conversion to float matrix
    if (controlled_robot && controlled_robot->camera_target)
    {
        target_pos = utils::getTranslation(
            controlled_robot->camera_target->worldTransform());
    }
    else
    {
        target_pos = Eigen::Vector3d::Zero();
    }
    m_camera.setView(m_camera.getViewType(), target_pos);

    // Set camera matrices to OpenGL shader
    m_shader_manager.setMatrix4f(m_projection_uniform,
                                 m_camera.getProjectionMatrix().data());
    m_shader_manager.setMatrix4f(m_view_uniform,
                                 m_camera.getViewMatrix().data());

    // Render the world ground
    m_geometry_renderer.renderGrid();

    // Render all robots
    for (const auto& [_, it] : m_robot_manager.robots())
    {
        if (it.is_visible && it.robot->hasRoot())
        {
            it.robot->root().traverse(
                [this](scene::Node const& node, size_t /*p_depth*/)
                {
                    if (auto geometry = dynamic_cast<Geometry const*>(&node))
                    {
                        Eigen::Matrix4f transform =
                            node.worldTransform().cast<float>();
                        renderGeometry(*geometry, transform);
                    }
                });
        }
    }

    m_window.swapBuffers();
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::renderGeometry(Geometry const& p_geometry,
                                            Eigen::Matrix4f const& p_transform)
{
    // Set model matrix to OpenGL shader
    m_shader_manager.setMatrix4f(m_model_uniform, p_transform.data());

    // Set color to OpenGL shader
    m_shader_manager.setVector3f(m_color_uniform, p_geometry.color.data());

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
                    if (!m_mesh_manager.loadMesh(mesh_path))
                    {
                        std::cout << "Failed to load mesh: " << mesh_path
                                  << ": " << m_mesh_manager.error()
                                  << std::endl;
                        return;
                    }
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
    auto current_time = std::chrono::steady_clock::now();
    double elapsed_seconds =
        std::chrono::duration<double>(current_time - m_start_time).count();

    for (const auto& [_, it] : m_robot_manager.robots())
    {
        if (it.control_mode == RobotManager::ControlMode::ANIMATION)
        {
            handleAnimation(*it.robot, elapsed_seconds);
        }
        else if (it.control_mode ==
                 RobotManager::ControlMode::INVERSE_KINEMATICS)
        {
            handleInverseKinematics();
        }
    }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onPhysicUpdate(float const /* dt */)
{
    // No physics simulation needed for now
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::handleAnimation(Robot& p_robot, double p_time)
{
    // Update animation time (slower for more visible animation)
    float animation_time = static_cast<float>(p_time) * 0.5f;

    // Get current joint values
    std::vector<double> joint_values = p_robot.jointPositions();

    // Animate each joint with different frequencies and amplitudes
    for (size_t i = 0; i < joint_values.size(); ++i)
    {
        // Different frequency for each joint to create varied motion
        float frequency = 0.8f + static_cast<float>(i) * 0.3f;
        float amplitude = 0.8f; // Increased amplitude for more visible motion

        // Sinusoidal motion with phase offset for each joint
        float phase = static_cast<float>(i) * 1.5f;
        joint_values[i] =
            double(amplitude * std::sin(frequency * animation_time + phase));
    }

    p_robot.setJointValues(joint_values);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::handleInverseKinematics()
{
    // TODO: Implement inverse kinematics
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
    if (action == GLFW_PRESS)
    {
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
                if (auto* controlled_robot = m_robot_manager.currentRobot();
                    controlled_robot)
                {
                    controlled_robot->robot->setNeutralPosition();
                }
                break;
            // Toggle robot visibility
            case GLFW_KEY_V:
                if (auto* robot = m_robot_manager.currentRobot(); robot)
                {
                    robot->is_visible = !robot->is_visible;
                }
                break;
            // Robot mode switching (animation or inverse kinematics)
            case GLFW_KEY_M:
                if (auto* robot = m_robot_manager.currentRobot(); robot)
                {
                    robot->control_mode =
                        robot->control_mode ==
                                RobotManager::ControlMode::ANIMATION
                            ? RobotManager::ControlMode::INVERSE_KINEMATICS
                            : RobotManager::ControlMode::ANIMATION;
                    std::cout
                        << "🤖 Mode: "
                        << (m_robot_manager.currentRobot()->control_mode ==
                                    RobotManager::ControlMode::ANIMATION
                                ? "Animation"
                                : "IK")
                        << std::endl;
                }
                break;
            default:
                break;
        }
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