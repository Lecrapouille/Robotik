/**
 * @file RobotViewerApplication.cpp
 * @brief Robot viewer application class for the 3D viewer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Application.hpp"

#include "Robotik/Common/Conversions.hpp"
#include "Robotik/Robot/Debug.hpp"
#include "Robotik/Solvers/IKSolver.hpp"
#include "Robotik/Solvers/Trajectory.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <iostream>

namespace robotik::renderer
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

// Background color
const Eigen::Vector3f s_clear_color(0.1f, 0.1f, 0.1f);

// ----------------------------------------------------------------------------
RobotViewerApplication::RobotViewerApplication(Configuration const& p_config)
    : OpenGLApplication(p_config.window_width, p_config.window_height, true),
      path(p_config.search_paths),
      m_config(p_config),
      m_physics_simulator(1.0 / double(p_config.target_physics_hz),
                          p_config.physics_gravity)
{
    setTitle(p_config.window_title);
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::run()
{
    return OpenGLApplication::run(m_config.target_fps,
                                  m_config.target_physics_hz);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::setTitle(std::string const& p_title)
{
    m_title = p_title;
    OpenGLApplication::setTitle(m_title + " - FPS: " + std::to_string(m_fps));
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::onSetup()
{
    // Initialize OpenGL states
    glEnable(GL_DEPTH_TEST);

    // Search for meshes in the specified paths.
    path.add(m_config.search_paths);

    // Create perspective camera for intuitive robot inspection
    float aspect_ratio = static_cast<float>(m_config.window_width) /
                         static_cast<float>(m_config.window_height);
    m_perspective_camera =
        std::make_unique<PerspectiveCamera>(45.0f, aspect_ratio, 0.1f, 100.0f);
    m_perspective_camera->setPosition(Eigen::Vector3f(3.0f, 3.0f, 3.0f));
    m_perspective_camera->lookAt(Eigen::Vector3f(0.0f, 0.0f, 0.0f));

    // Create orbit controller for intuitive robot inspection
    m_orbit_controller = std::make_unique<OrbitController>(
        *m_perspective_camera, Eigen::Vector3f(0.0f, 0.0f, 0.5f), 5.0f);

    // Create shader manager
    if (!m_shader_manager.createProgram(
            "basic", vertex_shader_source, fragment_shader_source))
    {
        m_error =
            "Failed to create shader program: " + m_shader_manager.error();
        return false;
    }

    // Use the shader
    if (!m_shader_manager.useProgram("basic"))
    {
        m_error = "Failed to use shader program";
        return false;
    }

    // Set up camera matrices
    m_view_uniform = m_shader_manager.getUniformLocation("view");
    m_projection_uniform = m_shader_manager.getUniformLocation("projection");
    m_shader_manager.setMatrix4f(m_view_uniform,
                                 m_perspective_camera->viewMatrix().data());
    m_shader_manager.setMatrix4f(
        m_projection_uniform, m_perspective_camera->projectionMatrix().data());

    // Create renderer
    m_renderer = std::make_unique<Renderer>(m_shader_manager);
    if (!m_renderer->initialize())
    {
        m_error = "Failed to initialize renderer: " + m_renderer->error();
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
        std::cout << robotik::debug::printRobot(*controlled_robot, true)
                  << std::endl;
    }

    // Set control joint (end effector) for inverse kinematics
    if (!setControlJoint(*controlled_robot, m_config.control_joint))
    {
        m_error = "🤖 Failed to set control joint: " + m_config.control_joint +
                  ": " + m_error;
        return false;
    }
    else if (controlled_robot->control_joint != nullptr)
    {
        std::cout << "🤖 Control joint: "
                  << controlled_robot->control_joint->name() << std::endl;
    }

    // Compute IK target poses if control joint is set
    computeIKTargetPoses(*controlled_robot);

    // Compute trajectory configurations
    computeTrajectoryConfigs(*controlled_robot);

    // Set initial joint values to the specified position
    if (auto const& joint_positions = m_config.joint_positions;
        !joint_positions.empty())
    {
        controlled_robot->blueprint().forEachJoint(
            [&joint_positions](Joint& joint, size_t index)
            { joint.position(joint_positions[index]); });
        std::cout << "🤖 Set initial joint values from configuration"
                  << std::endl;
    }
    else
    {
        controlled_robot->setNeutralPosition();
        std::cout << "🤖 Set neutral position" << std::endl;
    }

    // Set camera target (base link or tool center point) to track
    bool use_root_if_not_found = true;
    if (!setCameraTarget(
            *controlled_robot, m_config.camera_target, use_root_if_not_found))
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

    // Create HMI after all components are ready
    m_hmi = std::make_unique<DearRobotHMI>(
        m_robot_manager, *m_orbit_controller, [this]() { halt(); });

    return true;
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::computeIKTargetPoses(
    RobotManager::ControlledRobot& p_controlled_robot)
{
    // Compute IK target poses if control joint is set
    if (p_controlled_robot.control_joint == nullptr)
        return;

    std::cout << "🎯 Computing IK target poses..." << std::endl;

    // Reserve memory for 3 joint configurations
    size_t const num_poses = 3;
    std::vector<std::vector<double>> joint_configs(num_poses);
    for (auto& it : joint_configs)
    {
        it.resize(p_controlled_robot.blueprint().numJoints());
    }

    // Setup 3 joint configurations: 1/3 joint limits, 2/3 joint limits and
    // neutral position
    p_controlled_robot.blueprint().forEachJoint(
        [&joint_configs](Joint const& joint, size_t index)
        {
            auto [min, max] = joint.limits();

            // Pose 1: 1/3 from min
            joint_configs[0][index] = min + (max - min) * 1.0 / 3.0;
            // Pose 2: center (neutral)
            joint_configs[1][index] = (max + min) / 2.0;
            // Pose 3: 2/3 from min
            joint_configs[2][index] = min + (max - min) * 2.0 / 3.0;
        });

    // For each configuration, compute end-effector pose
    p_controlled_robot.ik_target_poses.clear();
    p_controlled_robot.ik_target_poses.reserve(num_poses);
    for (size_t pose_id = 0; pose_id < num_poses; ++pose_id)
    {
        // Apply joint configuration
        p_controlled_robot.blueprint().forEachJoint(
            [&joint_configs, pose_id](Joint& joint, size_t index)
            { joint.position(joint_configs[pose_id][index]); });

        // Get end-effector transform
        Transform end_effector_transform =
            p_controlled_robot.control_joint->worldTransform();

        // Convert to pose and store
        Pose target_pose = robotik::transformToPose(end_effector_transform);
        p_controlled_robot.ik_target_poses.push_back(target_pose);

        std::cout << "  Target " << (pose_id + 1) << ": ["
                  << target_pose.transpose() << "]" << std::endl;
    }

    // Initialize IK solver
    p_controlled_robot.ik_solver = std::make_unique<JacobianIKSolver>();
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::computeTrajectoryConfigs(
    RobotManager::ControlledRobot& p_controlled_robot)
{
    std::cout << "🎯 Computing trajectory configurations..." << std::endl;

    // Generate 3 joint configurations
    size_t const num_configs = 3;
    p_controlled_robot.trajectory_configs.clear();
    p_controlled_robot.trajectory_configs.resize(num_configs);

    for (auto& config : p_controlled_robot.trajectory_configs)
    {
        config.resize(p_controlled_robot.blueprint().numJoints());
    }

    // Setup 3 joint configurations: 1/4, 1/2, 3/4 of joint range
    p_controlled_robot.blueprint().forEachJoint(
        [&p_controlled_robot](Joint const& joint, size_t index)
        {
            auto [min, max] = joint.limits();

            // Config 0: 1/4 from min
            p_controlled_robot.trajectory_configs[0][index] =
                min + (max - min) * 0.25;
            // Config 1: center
            p_controlled_robot.trajectory_configs[1][index] = (max + min) / 2.0;
            // Config 2: 3/4 from min
            p_controlled_robot.trajectory_configs[2][index] =
                min + (max - min) * 0.75;
        });

    // Print configurations
    for (size_t i = 0; i < num_configs; ++i)
    {
        std::cout << "  Config " << (i + 1) << ": [";
        for (size_t j = 0; j < p_controlled_robot.trajectory_configs[i].size();
             ++j)
        {
            std::cout << p_controlled_robot.trajectory_configs[i][j];
            if (j < p_controlled_robot.trajectory_configs[i].size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::setControlJoint(
    RobotManager::ControlledRobot& p_controlled_robot,
    std::string const& p_element_name) const
{
    p_controlled_robot.control_joint = nullptr;

    // No element name provided, use the first end effector if any.
    if (p_element_name.empty())
    {
        if (auto const& end_effectors =
                p_controlled_robot.blueprint().endEffectors();
            !end_effectors.empty())
        {
            p_controlled_robot.control_joint = &end_effectors[0].get();
        }
    }
    // Otherwise, search for the element by name. If not found, use the root.
    else
    {
        p_controlled_robot.control_joint =
            Node::find(p_controlled_robot.blueprint().root(), p_element_name);
        if (p_controlled_robot.control_joint == nullptr)
        {
            p_controlled_robot.control_joint =
                &p_controlled_robot.blueprint().root();
        }
    }

    return p_controlled_robot.control_joint != nullptr;
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::setCameraTarget(
    RobotManager::ControlledRobot& p_controlled_robot,
    std::string const& p_element_name,
    bool p_use_root_if_not_found) const
{
    p_controlled_robot.camera_target = nullptr;

    // No element name provided, use the root if requested or the first end
    // effector if any.
    if (p_element_name.empty())
    {
        if (p_use_root_if_not_found)
        {
            p_controlled_robot.camera_target =
                &p_controlled_robot.blueprint().root();
        }
        else if (auto const& end_effectors =
                     p_controlled_robot.blueprint().endEffectors();
                 !end_effectors.empty())
        {
            p_controlled_robot.camera_target = &end_effectors[0].get();
        }
    }
    // Otherwise, search for the element by name. If not found, use the root.
    else
    {
        p_controlled_robot.camera_target =
            Node::find(p_controlled_robot.blueprint().root(), p_element_name);
        if (p_controlled_robot.camera_target == nullptr)
        {
            p_controlled_robot.camera_target =
                &p_controlled_robot.blueprint().root();
        }
    }

    return p_controlled_robot.camera_target != nullptr;
}

// Note: setupShaderProgram removed - shaders now created inline in onSetup()
// Note: setupImGuiCallbacks removed - functionality moved to DearRobotHMI
// Note: updateCameraTarget removed - camera target is now updated in
// DearRobotHMI::cameraTargetPanel()

// ----------------------------------------------------------------------------
void RobotViewerApplication::onTeardown()
{
    m_robot_manager.clear();
}

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void RobotViewerApplication::onDrawScene()
{
    // Clear screen
    glClearColor(s_clear_color.x(), s_clear_color.y(), s_clear_color.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use shader
    m_shader_manager.useProgram("basic");

    // Update camera matrices
    m_shader_manager.setMatrix4f(m_view_uniform,
                                 m_perspective_camera->viewMatrix().data());
    m_shader_manager.setMatrix4f(
        m_projection_uniform, m_perspective_camera->projectionMatrix().data());

    // Render the world ground
    m_renderer->renderGrid(Eigen::Vector3f(0.5f, 0.5f, 0.5f));

    // Render all robots using RenderVisitor
    for (auto& [_, it] : m_robot_manager.robots())
    {
        if (it.is_visible && it.blueprint().hasRoot())
        {
            RenderVisitor visitor(
                m_mesh_manager, *m_renderer, m_shader_manager);
            it.blueprint().root().traverse(visitor);
        }

        // Render IK target if in IK mode
        if (it.control_mode == RobotManager::ControlMode::INVERSE_KINEMATICS &&
            it.ik_target_poses.size() == 3)
        {
            // Get current target pose
            Pose const& target_pose = it.ik_target_poses[m_target_pose_index];

            // Convert Pose to Transform
            Transform target_transform = robotik::poseToTransform(target_pose);

            // Create a simple cylinder mesh for axes if not already created
            if (!m_mesh_manager.hasMesh("axis_cylinder"))
            {
                m_mesh_manager.createCylinder("axis_cylinder", 0.01f, 0.1f);
            }

            const MeshManager::GPUMesh* axis_mesh =
                m_mesh_manager.getMesh("axis_cylinder");
            if (axis_mesh)
            {
                m_renderer->renderAxes(
                    target_transform.cast<float>(), 0.2f, axis_mesh);
            }
        }

        // Render trajectory path if in TRAJECTORY mode
        if (it.control_mode == RobotManager::ControlMode::TRAJECTORY &&
            it.trajectory && it.control_joint != nullptr)
        {
            renderTrajectoryPath(it);
        }
    }
}

// Note: renderGeometry removed - functionality moved to RenderVisitor

// ----------------------------------------------------------------------------
void RobotViewerApplication::renderTrajectoryPath(
    RobotManager::ControlledRobot& p_robot)
{
    if (!p_robot.trajectory || !p_robot.control_joint)
        return;

    // Number of samples to display along the trajectory
    constexpr size_t num_samples = 20;

    // Get trajectory duration
    double duration = p_robot.trajectory->duration();

    // Save current joint positions to restore them later
    std::vector<double> saved_positions;
    saved_positions.reserve(p_robot.blueprint().numJoints());
    p_robot.blueprint().forEachJoint(
        [&saved_positions](Joint const& joint, size_t /*index*/)
        { saved_positions.push_back(joint.position()); });

    // Create axis mesh if not already created
    if (!m_mesh_manager.hasMesh("axis_cylinder"))
    {
        m_mesh_manager.createCylinder("axis_cylinder", 0.01f, 0.1f);
    }

    const MeshManager::GPUMesh* axis_mesh =
        m_mesh_manager.getMesh("axis_cylinder");

    // Sample the trajectory and render axes at each sample
    for (size_t i = 0; i < num_samples; ++i)
    {
        // Compute time for this sample
        double t = (double(i) / double(num_samples - 1)) * duration;

        // Evaluate trajectory at this time
        Trajectory::States states = p_robot.trajectory->evaluate(t);

        // Apply joint configuration
        p_robot.blueprint().forEachJoint(
            [&states](Joint& joint, size_t index)
            { joint.position(states.position[index]); });

        // Get end-effector transform
        Transform end_effector_transform =
            p_robot.control_joint->worldTransform();

        // Render axes at this position (smaller scale for trajectory)
        if (axis_mesh)
        {
            m_renderer->renderAxes(
                end_effector_transform.cast<float>(), 0.15f, axis_mesh);
        }
    }

    // Restore original joint positions
    p_robot.blueprint().forEachJoint(
        [&saved_positions](Joint& joint, size_t index)
        { joint.position(saved_positions[index]); });
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onUpdate(float const dt)
{
    // Update camera controller
    m_orbit_controller->update(dt);

    auto current_time = std::chrono::steady_clock::now();
    double elapsed_seconds =
        std::chrono::duration<double>(current_time - m_start_time).count();

    for (auto& [_, it] : m_robot_manager.robots())
    {
        switch (it.control_mode)
        {
            case RobotManager::ControlMode::ANIMATION:
                handleAnimation(it, elapsed_seconds);
                break;
            case RobotManager::ControlMode::INVERSE_KINEMATICS:
                handleInverseKinematics(it);
                break;
            case RobotManager::ControlMode::TRAJECTORY:
                handleTrajectory(it, elapsed_seconds, static_cast<double>(dt));
                break;
            case RobotManager::ControlMode::NO_CONTROL:
            case RobotManager::ControlMode::DIRECT_KINEMATICS:
            default:
                break;
        }
    }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onPhysicUpdate(float const /* dt */)
{
    // auto* controlled_robot = m_robot_manager.currentRobot();
    // if (controlled_robot && controlled_robot->robot)
    //{
    //     m_physics_simulator.step(*controlled_robot->robot);
    // }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::handleAnimation(
    RobotManager::ControlledRobot& p_robot,
    double p_time) const
{
    // Update animation time (slower for more visible animation)
    float animation_time = static_cast<float>(p_time) * 0.5f;

    // Get current joint values
    auto& joint_values = p_robot.state().joint_positions;

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

    // Update joint positions
    p_robot.blueprint().forEachJoint([&joint_values](Joint& joint, size_t index)
                                     { joint.position(joint_values[index]); });
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::handleInverseKinematics(
    RobotManager::ControlledRobot& p_robot)
{
    if (p_robot.control_joint == nullptr || p_robot.ik_solver == nullptr)
    {
        std::cout << "🤖 Error: control joint or solver not set" << std::endl;
        return;
    }

    // Get current target pose based on state);
    Pose const& target_pose = p_robot.ik_target_poses[m_target_pose_index];

    // Solve IK for current target
    bool solved =
        p_robot.ik_solver->solve(p_robot, *p_robot.control_joint, target_pose);

    // If converged, transition to next state
    if (solved)
    {
        std::cout << "🎯 Reached target " << (m_target_pose_index + 1)
                  << " - Error: " << p_robot.ik_solver->poseError()
                  << " - Iterations: " << p_robot.ik_solver->numIterations()
                  << std::endl;

        // Transition to next state
        m_target_pose_index++;
        if (m_target_pose_index >= p_robot.ik_target_poses.size())
        {
            m_target_pose_index = 0;
        }
    }
    else
    {
        // Only log failures occasionally to avoid spam
        static size_t failure_count = 0;
        if (++failure_count % 100 == 0)
        {
            std::cout << "⚠️ IK solving in progress - Error: "
                      << p_robot.ik_solver->poseError() << std::endl;
        }
    }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::handleTrajectory(
    RobotManager::ControlledRobot& p_robot,
    double p_time,
    double p_dt)
{
    if (p_robot.trajectory_configs.empty())
    {
        std::cout << "🤖 Error: no trajectory configurations" << std::endl;
        return;
    }

    // If no trajectory is active, create a new one
    if (!p_robot.trajectory)
    {
        size_t start_idx =
            p_robot.trajectory_segment % p_robot.trajectory_configs.size();
        size_t goal_idx = (p_robot.trajectory_segment + 1) %
                          p_robot.trajectory_configs.size();

        std::cout << "🎯 Creating trajectory from config " << (start_idx + 1)
                  << " to config " << (goal_idx + 1) << std::endl;

        // Create trajectory generator
        JointSpaceGenerator generator;

        // Generate trajectory with 3 second duration
        double trajectory_duration = 3.0;
        p_robot.trajectory =
            generator.generate(p_robot.trajectory_configs[start_idx],
                               p_robot.trajectory_configs[goal_idx],
                               trajectory_duration);

        p_robot.trajectory_start_time = p_time;
    }

    // Evaluate trajectory at current time
    double t = p_time - p_robot.trajectory_start_time;

    if (t >= p_robot.trajectory->duration())
    {
        // Trajectory complete, move to next segment
        std::cout << "✅ Trajectory segment "
                  << (p_robot.trajectory_segment + 1) << " complete"
                  << std::endl;

        p_robot.trajectory_segment++;
        p_robot.trajectory.reset(); // Will create new trajectory on next update
        return;
    }

    // Get target position from trajectory
    auto trajectory_point = p_robot.trajectory->evaluate(t);

    // Apply with velocity limits
    p_robot.applyJointTargetsWithSpeedLimit(trajectory_point.position, p_dt);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onFPSUpdated(size_t const p_fps)
{
    m_fps = p_fps;
    OpenGLApplication::setTitle(m_title + " - FPS: " + std::to_string(p_fps));
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onWindowResize(int p_width, int p_height)
{
    // Update main window viewport
    glViewport(0, 0, p_width, p_height);

    // Update camera aspect ratio
    float aspect_ratio =
        static_cast<float>(p_width) / static_cast<float>(p_height);
    m_perspective_camera->setAspectRatio(aspect_ratio);
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
                halt();
                break;
            // Reset robot to neutral position
            case GLFW_KEY_SPACE:
                if (auto* controlled_robot = m_robot_manager.currentRobot();
                    controlled_robot)
                {
                    controlled_robot->setNeutralPosition();
                }
                break;
            // Toggle robot visibility
            case GLFW_KEY_V:
                if (auto* robot = m_robot_manager.currentRobot(); robot)
                {
                    robot->is_visible = !robot->is_visible;
                }
                break;
            // Robot mode switching (animation, inverse kinematics, trajectory)
            case GLFW_KEY_M:
                if (auto* robot = m_robot_manager.currentRobot();
                    robot != nullptr && robot->control_joint != nullptr)
                {
                    switch (robot->control_mode)
                    {
                        case RobotManager::ControlMode::ANIMATION:
                            std::cout << "🤖 Mode: INVERSE_KINEMATICS"
                                      << std::endl;
                            robot->control_mode =
                                RobotManager::ControlMode::INVERSE_KINEMATICS;
                            break;
                        case RobotManager::ControlMode::INVERSE_KINEMATICS:
                            std::cout << "🤖 Mode: TRAJECTORY" << std::endl;
                            robot->control_mode =
                                RobotManager::ControlMode::TRAJECTORY;
                            robot->trajectory_segment = 0;
                            robot->trajectory.reset();
                            break;
                        case RobotManager::ControlMode::TRAJECTORY:
                            std::cout << "🤖 Mode: NO_CONTROL" << std::endl;
                            robot->control_mode =
                                RobotManager::ControlMode::NO_CONTROL;
                            break;
                        case RobotManager::ControlMode::NO_CONTROL:
                            std::cout << "🤖 Mode: ANIMATION" << std::endl;
                            robot->control_mode =
                                RobotManager::ControlMode::ANIMATION;
                            break;
                        default:
                            break;
                    }
                }
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
    m_orbit_controller->handleMouseButton(p_button, p_action, p_mods);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onCursorPos(double p_xpos, double p_ypos)
{
    m_orbit_controller->handleMouseMove(p_xpos, p_ypos);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onScroll(double xoffset, double yoffset)
{
    m_orbit_controller->handleScroll(xoffset, yoffset);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onDrawMenuBar()
{
    if (m_hmi)
    {
        m_hmi->onDrawMenuBar();
    }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onDrawMainPanel()
{
    if (m_hmi)
    {
        m_hmi->onDrawMainPanel();
    }
}

} // namespace robotik::renderer
