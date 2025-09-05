#include "RobotViewerApplication.hpp"

#include "Robotik/Debug.hpp"
#include "Robotik/Parser.hpp"
#include "Robotik/private/Conversions.hpp"
#include "Robotik/private/Exception.hpp"

#include <iostream>
#include <map>

namespace robotik
{

// ----------------------------------------------------------------------------
RobotViewerApplication::RobotViewerApplication(Configuration const& p_config)
    : Application(p_config.window_width,
                  p_config.window_height,
                  p_config.window_title),
      m_config(p_config),
      m_start_time(std::chrono::steady_clock::now())
{
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::onSetup()
{
    if (!loadRobot(m_config.urdf_file))
        return false;

    if (!setTarget(m_config.control_joint))
        return false;

    if (!initCameraView(m_config.camera_target))
        return false;

    return true;
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::loadRobot(const std::string& p_urdf_file)
{
    URDFParser parser;

    m_robot = parser.load(p_urdf_file);
    if (!m_robot)
    {
        m_error = "Failed to load robot from '" + p_urdf_file +
                  "': " + parser.getError();
        return false;
    }
    else
    {
        std::cout << "Loaded robot from: " << p_urdf_file << std::endl;
        std::cout << debug::printRobot(*m_robot, true) << std::endl;
    }
    return true;
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::setTarget(std::string const& p_target_joint_name)
{
    m_target = nullptr;

    // Search for the joint to control, given by the user from the application
    // command line. If not provided, find the robot end effector.
    try
    {
        if (!p_target_joint_name.empty())
        {
            m_target = &m_robot->joint(p_target_joint_name);
        }
        else
        {
            m_target = &m_robot->findEndEffector();
        }
    }
    catch (RobotikException const& e)
    {
        m_error = e.what();
        return false;
    }

    // Set the target pose to the target joint.
    if (m_target)
    {
        std::cout << "Target: " << m_target->name() << std::endl;
        m_target_pose = utils::transformToPose(m_target->worldTransform());
    }
    else
    {
        m_error =
            "Warning: End effector not found, inverse kinematics disabled";
        return false;
    }

    return m_target != nullptr;
}

// ----------------------------------------------------------------------------
bool RobotViewerApplication::initCameraView(
    std::string const& p_look_at_joint_name)
{
    m_camera_target = nullptr;

    // Search for the joint to look at, given by the user from the
    // application command line. If not provided, use the robot root.
    try
    {
        if (!p_look_at_joint_name.empty())
        {
            m_camera_target = &m_robot->joint(p_look_at_joint_name);
        }
        else
        {
            m_camera_target = &m_robot->root();
        }
    }
    catch (RobotikException const& e)
    {
        m_error = e.what();
        return false;
    }

    // Set the camera view to the target joint.
    if (m_camera_target)
    {
        std::cout << "Camera target: " << m_camera_target->name() << std::endl;
        m_viewer.cameraView(
            OpenGLViewer::CameraViewType::SIDE,
            utils::getTranslation(m_camera_target->worldTransform()));
    }
    else
    {
        m_error = "Warning: Camera target not found";
    }

    return m_camera_target != nullptr;
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onDraw()
{
    std::lock_guard<std::mutex> lock(m_robot_mutex);
    m_viewer.render(*m_robot);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onUpdate(float const /* dt */)
{
    // Track the camera view to the target joint.
    m_viewer.cameraView(
        m_config.camera_view,
        utils::getTranslation(m_camera_target->worldTransform()));
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::onPhysicUpdate(float const /* dt */)
{
    auto current_time = std::chrono::steady_clock::now();
    double elapsed_seconds =
        std::chrono::duration<double>(current_time - m_start_time).count();

    // Lock robot for updates
    std::lock_guard<std::mutex> lock(m_robot_mutex);

    ControlMode mode = m_control_mode.load();
    if (mode == ControlMode::ANIMATION)
    {
        handleAnimation(elapsed_seconds);
    }
    else if (mode == ControlMode::INVERSE_KINEMATICS)
    {
        handleInverseKinematics();
    }
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::handleAnimation(double p_time)
{
    // Update animation time (slower for more visible animation)
    float animation_time = static_cast<float>(p_time) * 0.5f;

    // Get current joint values
    std::vector<double> joint_values = m_robot->jointValues();

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

    // Apply the new joint values to the robot
    m_robot->setJointValues(joint_values);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::handleInverseKinematics()
{
    std::cout << "Target pose: [" << m_target_pose(0) << ", "
              << m_target_pose(1) << ", " << m_target_pose(2) << ", "
              << m_target_pose(3) << ", " << m_target_pose(4) << ", "
              << m_target_pose(5) << "]" << std::endl;

    // Calculate inverse kinematics
    auto solution = m_robot->inverseKinematics(m_target_pose, *m_target);

    if (!solution.empty())
    {
        m_robot->setJointValues(solution);
        std::cout << "IK solution found: ";
        for (double joint_val : solution)
        {
            std::cout << joint_val << " ";
        }
        std::cout << std::endl;
    }
    else
    {
        std::cout << "No IK solution found for target pose!" << std::endl;
    }

    m_target_updated.store(false);
}

// ----------------------------------------------------------------------------
void RobotViewerApplication::handleInput(int key, int /* action */)
{
    // Lookup table for camera view keys => camera view type.
    static const std::map<int, OpenGLViewer::CameraViewType> camera_keys = {
        { GLFW_KEY_1, OpenGLViewer::CameraViewType::PERSPECTIVE },
        { GLFW_KEY_2, OpenGLViewer::CameraViewType::TOP },
        { GLFW_KEY_3, OpenGLViewer::CameraViewType::FRONT },
        { GLFW_KEY_4, OpenGLViewer::CameraViewType::SIDE },
        { GLFW_KEY_5, OpenGLViewer::CameraViewType::ISOMETRIC }
    };

    if (!m_viewer.isKeyPressed(key))
        return;

    // Robot mode switching (animation or inverse kinematics)
    if (key == GLFW_KEY_M)
    {
        auto new_mode = (m_control_mode.load() == ControlMode::ANIMATION)
                            ? ControlMode::INVERSE_KINEMATICS
                            : ControlMode::ANIMATION;
        m_control_mode.store(new_mode);
        std::cout << "Mode: "
                  << (new_mode == ControlMode::ANIMATION ? "Animation" : "IK")
                  << std::endl;

        if (new_mode == ControlMode::INVERSE_KINEMATICS)
        {
            m_target_pose = utils::transformToPose(m_target->worldTransform());
            std::cout << "Target pose initialized" << std::endl;
        }
    }

    // Switch camera views
    auto it = camera_keys.find(key);
    if (it != camera_keys.end())
    {
        m_config.camera_view = it->second;
    }

    // IK target position controls: continuous movement while pressed
    if (m_control_mode.load() == ControlMode::INVERSE_KINEMATICS)
    {
        if (m_viewer.isKeyPressed(GLFW_KEY_Z)) // +X
        {
            m_target_pose(0) += 0.05;
            m_target_updated.store(true);
            std::cout << "Z pressed - Target X: " << m_target_pose(0)
                      << std::endl;
        }
        if (m_viewer.isKeyPressed(GLFW_KEY_S)) // -X
        {
            m_target_pose(0) -= 0.05;
            m_target_updated.store(true);
            std::cout << "S pressed - Target X: " << m_target_pose(0)
                      << std::endl;
        }
        if (m_viewer.isKeyPressed(GLFW_KEY_Q)) // -Y
        {
            m_target_pose(1) -= 0.05;
            m_target_updated.store(true);
            std::cout << "Q pressed - Target Y: " << m_target_pose(1)
                      << std::endl;
        }
        if (m_viewer.isKeyPressed(GLFW_KEY_D)) // +Y
        {
            m_target_pose(1) += 0.05;
            m_target_updated.store(true);
            std::cout << "D pressed - Target Y: " << m_target_pose(1)
                      << std::endl;
        }
        if (m_viewer.isKeyPressed(GLFW_KEY_A)) // +Z
        {
            m_target_pose(2) += 0.05;
            m_target_updated.store(true);
            std::cout << "A pressed - Target Z: " << m_target_pose(2)
                      << std::endl;
        }
        if (m_viewer.isKeyPressed(GLFW_KEY_E)) // -Z
        {
            m_target_pose(2) -= 0.05;
            m_target_updated.store(true);
            std::cout << "E pressed - Target Z: " << m_target_pose(2)
                      << std::endl;
        }
    }
}

} // namespace robotik