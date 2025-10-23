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
#include "Robotik/Core/Debug.hpp"
#include "Robotik/Core/IKSolver.hpp"
#include "Robotik/Core/Trajectory.hpp"
#include "Robotik/Viewer/DearImGuiApp.hpp"

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
    : path(p_config.search_paths),
      m_config(p_config),
      m_window(p_config.window_width, p_config.window_height),
      m_camera(p_config.window_width, p_config.window_height),
      m_geometry_renderer(m_shader_manager),
      m_physics_simulator(1.0 / double(p_config.target_physics_hz),
                          p_config.physics_gravity),
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

    // Initialize OpenGL states
    glEnable(GL_DEPTH_TEST);

    // Search for meshes in the specified paths.
    path.add(m_config.search_paths);

    // Create OpenGL shaders
    if (!setupShaderProgram("default"))
    {
        m_error =
            "Failed to create shader program: " + m_shader_manager.error();
        return false;
    }

    // Use shader program and initialize shader uniform locations. Since
    // we have only one shader program, we can initialize the uniform
    // locations once.
    m_shader_manager.useProgram("default");
    m_model_uniform = m_shader_manager.getUniformLocation("model");
    m_color_uniform = m_shader_manager.getUniformLocation("color");
    m_projection_uniform = m_shader_manager.getUniformLocation("projection");
    m_view_uniform = m_shader_manager.getUniformLocation("view");

    // Initialize geometry renderer
    if (!m_geometry_renderer.initialize())
    {
        m_error = "Failed to initialize geometry renderer: " +
                  m_geometry_renderer.error();
        return false;
    }

    // Initialize ImGui application
    m_imgui_app = std::make_unique<viewer::ImGuiApp>(m_config.window_width,
                                                     m_config.window_height);
    if (!m_imgui_app->setup())
    {
        m_error = "Failed to initialize ImGui";
        return false;
    }

    // Set the render callback for the 3D scene
    m_imgui_app->setRenderCallback([this]() { this->render3DScene(); });

    std::cout << "✨ ImGui initialized with docking support" << std::endl;

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
        std::cout << robotik::debug::printRobot(*controlled_robot->robot, true)
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

    // Compute IK target poses if control joint is set.
    // To be placed before setting the initial joint values.
    computeIKTargetPoses(*controlled_robot);

    // Compute trajectory configurations
    computeTrajectoryConfigs(*controlled_robot);

    // Set initial joint values to the specified position
    if (auto const& joint_positions = m_config.joint_positions;
        !joint_positions.empty())
    {
        controlled_robot->robot->hierarchy().forEachJoint(
            [&joint_positions](Joint& joint, size_t index)
            { joint.position(joint_positions[index]); });
        std::cout << "🤖 Set initial joint values from configuration"
                  << std::endl;
    }
    else
    {
        controlled_robot->robot->setNeutralPosition();
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
        it.resize(p_controlled_robot.robot->hierarchy().numJoints());
    }

    // Setup 3 joint configurations: 1/3 joint limits, 2/3 joint limits and
    // neutral position
    p_controlled_robot.robot->hierarchy().forEachJoint(
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
        p_controlled_robot.robot->hierarchy().forEachJoint(
            [&joint_configs, pose_id](Joint& joint, size_t index)
            { joint.position(joint_configs[pose_id][index]); });

        // Get end-effector transform
        Transform end_effector_transform =
            p_controlled_robot.control_joint->worldTransform();

        // Convert to pose and store
        Pose target_pose = utils::transformToPose(end_effector_transform);
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
        config.resize(p_controlled_robot.robot->hierarchy().numJoints());
    }

    // Setup 3 joint configurations: 1/4, 1/2, 3/4 of joint range
    p_controlled_robot.robot->hierarchy().forEachJoint(
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
                p_controlled_robot.robot->hierarchy().endEffectors();
            !end_effectors.empty())
        {
            p_controlled_robot.control_joint = &end_effectors[0].get();
        }
    }
    // Otherwise, search for the element by name. If not found, use the root.
    else
    {
        p_controlled_robot.control_joint = Node::find(
            p_controlled_robot.robot->hierarchy().root(), p_element_name);
        if (p_controlled_robot.control_joint == nullptr)
        {
            p_controlled_robot.control_joint =
                &p_controlled_robot.robot->hierarchy().root();
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
                &p_controlled_robot.robot->hierarchy().root();
        }
        else if (auto const& end_effectors =
                     p_controlled_robot.robot->hierarchy().endEffectors();
                 !end_effectors.empty())
        {
            p_controlled_robot.camera_target = &end_effectors[0].get();
        }
    }
    // Otherwise, search for the element by name. If not found, use the root.
    else
    {
        p_controlled_robot.camera_target = Node::find(
            p_controlled_robot.robot->hierarchy().root(), p_element_name);
        if (p_controlled_robot.camera_target == nullptr)
        {
            p_controlled_robot.camera_target =
                &p_controlled_robot.robot->hierarchy().root();
        }
    }

    return p_controlled_robot.camera_target != nullptr;
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::setupShaderProgram(
    std::string const& p_program_name)
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

    return true;
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onCleanup()
{
    if (m_imgui_app)
    {
        m_imgui_app->teardown();
        m_imgui_app.reset();
    }
    m_robot_manager.clear();
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::updateCameraTarget()
{
    // Update camera target position, tracking one element of the robot.
    // If no robot is selected, set camera target to zero position.
    if (auto const* controlled_robot = m_robot_manager.currentRobot();
        (controlled_robot != nullptr) &&
        (controlled_robot->camera_target != nullptr))
    {
        Eigen::Vector3f target_position =
            utils::getTranslation(
                controlled_robot->camera_target->worldTransform())
                .cast<float>();
        m_camera.setView(target_position);
    }
    else
    {
        Eigen::Vector3f zero_position = Eigen::Vector3f::Zero();
        m_camera.setView(zero_position);
    }

    // Set camera matrices to OpenGL shader
    m_shader_manager.setMatrix4f(m_projection_uniform,
                                 m_camera.getProjectionMatrix().data());
    m_shader_manager.setMatrix4f(m_view_uniform,
                                 m_camera.getViewMatrix().data());
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onDraw()
{
    // Poll events first
    m_window.pollEvents();

    // Render with ImGui
    if (m_imgui_app)
    {
        m_imgui_app->draw();
    }

    // Swap buffers
    m_window.swapBuffers();
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::render3DScene()
{
    // Clear screen
    glClearColor(s_clear_color.x(), s_clear_color.y(), s_clear_color.z(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Update camera target position, tracking one element of the robot
    updateCameraTarget();

    // Render the world ground
    m_geometry_renderer.renderGrid();

    // Render all robots
    for (auto& [_, it] : m_robot_manager.robots())
    {
        if (it.is_visible && it.robot->hierarchy().hasRoot())
        {
            it.robot->hierarchy().root().traverse(
                [this](Node const& node, size_t /*p_depth*/)
                {
                    if (auto geometry = dynamic_cast<Geometry const*>(&node))
                    {
                        Eigen::Matrix4f transform =
                            node.worldTransform().cast<float>();
                        renderGeometry(*geometry, transform);
                    }
                });
        }

        // Render IK target if in IK mode
        if (it.control_mode == RobotManager::ControlMode::INVERSE_KINEMATICS &&
            it.ik_target_poses.size() == 3)
        {
            // Get current target pose
            Pose const& target_pose = it.ik_target_poses[m_target_pose_index];

            // Convert Pose to Transform
            Transform target_transform = utils::poseToTransform(target_pose);

            // Render axes at target position
            m_geometry_renderer.renderAxes(target_transform.cast<float>(),
                                           0.2f);
        }

        // Render trajectory path if in TRAJECTORY mode
        if (it.control_mode == RobotManager::ControlMode::TRAJECTORY &&
            it.trajectory && it.control_joint != nullptr)
        {
            renderTrajectoryPath(it);
        }
    }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::renderGeometry(Geometry const& p_geometry,
                                            Eigen::Matrix4f const& p_transform)
{
    // URDF to OpenGL transformation: rotate -90 degrees around X-axis
    // URDF: Z up, X forward, Y left
    // OpenGL: Y up, -Z forward, X right
    Eigen::Matrix4f urdf_to_opengl = Eigen::Matrix4f::Identity();
    urdf_to_opengl.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(-static_cast<float>(M_PI) / 2.0f,
                          Eigen::Vector3f::UnitX())
            .toRotationMatrix();

    // Apply transformation to model matrix (order: transform then
    // convert)
    Eigen::Matrix4f transformed = p_transform * urdf_to_opengl;

    // Set model matrix to OpenGL shader
    m_shader_manager.setMatrix4f(m_model_uniform, transformed.data());

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
            // Load and render mesh using MeshManager.
            // Note: use robot name as prefix for the mesh path to avoid
            // conflicts with other mesh names.
            std::string const& mesh_path = p_geometry.meshPath();

            // Load mesh if not already loaded
            if (!m_mesh_manager.isMeshLoaded(mesh_path))
            {
                if (!m_mesh_manager.loadMesh(path.expand(mesh_path)))
                {
                    std::cout << "Failed to load mesh: " << mesh_path << ": "
                              << m_mesh_manager.error() << std::endl;
                    return;
                }
            }

            // Render the mesh
            m_mesh_manager.renderMesh(mesh_path);
            break;
        }
        default:
            break;
    }
}

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
    saved_positions.reserve(p_robot.robot->hierarchy().numJoints());
    p_robot.robot->hierarchy().forEachJoint(
        [&saved_positions](Joint const& joint, size_t /*index*/)
        { saved_positions.push_back(joint.position()); });

    // Sample the trajectory and render axes at each sample
    for (size_t i = 0; i < num_samples; ++i)
    {
        // Compute time for this sample
        double t = (double(i) / double(num_samples - 1)) * duration;

        // Evaluate trajectory at this time
        Trajectory::States states = p_robot.trajectory->evaluate(t);

        // Apply joint configuration
        p_robot.robot->hierarchy().forEachJoint(
            [&states](Joint& joint, size_t index)
            { joint.position(states.position[index]); });

        // Get end-effector transform
        Transform end_effector_transform =
            p_robot.control_joint->worldTransform();

        // Render axes at this position (smaller scale for trajectory)
        m_geometry_renderer.renderAxes(end_effector_transform.cast<float>(),
                                       0.15f);
    }

    // Restore original joint positions
    p_robot.robot->hierarchy().forEachJoint(
        [&saved_positions](Joint& joint, size_t index)
        { joint.position(saved_positions[index]); });
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onUpdate(float const dt)
{
    auto current_time = std::chrono::steady_clock::now();
    double elapsed_seconds =
        std::chrono::duration<double>(current_time - m_start_time).count();

    for (auto& [_, it] : m_robot_manager.robots())
    {
        switch (it.control_mode)
        {
            case RobotManager::ControlMode::ANIMATION:
                handleAnimation(*it.robot, elapsed_seconds);
                break;
            case RobotManager::ControlMode::INVERSE_KINEMATICS:
                handleInverseKinematics(it);
                break;
            case RobotManager::ControlMode::TRAJECTORY:
                handleTrajectory(it, elapsed_seconds, static_cast<double>(dt));
                break;
            case RobotManager::ControlMode::NO_CONTROL:
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
void RobotViewerApplication::handleAnimation(Robot& p_robot,
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
    p_robot.hierarchy().forEachJoint([&joint_values](Joint& joint, size_t index)
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
    bool solved = p_robot.ik_solver->solve(
        *p_robot.robot, *p_robot.control_joint, target_pose);

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
    p_robot.robot->applyJointTargetsWithSpeedLimit(trajectory_point.position,
                                                   p_dt);
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
            // Camera view controls - only change view type, keep
            // current target
            case GLFW_KEY_1:
                m_camera.setView(Camera::ViewType::PERSPECTIVE);
                break;
            case GLFW_KEY_2:
                m_camera.setView(Camera::ViewType::TOP);
                break;
            case GLFW_KEY_3:
                m_camera.setView(Camera::ViewType::FRONT);
                break;
            case GLFW_KEY_4:
                m_camera.setView(Camera::ViewType::SIDE);
                break;
            case GLFW_KEY_5:
                m_camera.setView(Camera::ViewType::ISOMETRIC);
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
void RobotViewerApplication::onScroll(double /* xoffset */, double yoffset)
{
    // Handle scroll event for camera zoom
    // yoffset > 0 = scroll up = zoom in (move camera closer)
    // yoffset < 0 = scroll down = zoom out (move camera further)
    float zoom_speed = 0.5f;
    m_camera.zoom(static_cast<float>(-yoffset) * zoom_speed);
}

} // namespace robotik::viewer