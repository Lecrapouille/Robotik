#include "RobotViewer.hpp"
#include "Robotik/Debug.hpp"
#include "Robotik/Parser.hpp"

#include <iostream>
#include <string>
#include <thread>
#include <unordered_map>

namespace robotik_viewer
{

static RobotState g_robot_state;

// ----------------------------------------------------------------------------
void animate(robotik::Robot& p_robot, double p_time)
{
    // Update animation time (slower for more visible animation)
    float animation_time = static_cast<float>(p_time) * 0.5f;

    // Get current joint values
    std::vector<double> joint_values = p_robot.jointValues();

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
    p_robot.setJointValues(joint_values);
}

// ----------------------------------------------------------------------------
void handle_inverse_kinematics(robotik::Robot& p_robot,
                               const robotik::scene::Node* p_end_effector,
                               RobotState& p_state)
{
    if (!p_state.target_updated.load() || !p_end_effector)
    {
        return;
    }

    std::cout << "Target pose: [" << p_state.target_pose(0) << ", "
              << p_state.target_pose(1) << ", " << p_state.target_pose(2)
              << ", " << p_state.target_pose(3) << ", "
              << p_state.target_pose(4) << ", " << p_state.target_pose(5) << "]"
              << std::endl;

    // Calculate inverse kinematics
    auto solution =
        p_robot.inverseKinematics(p_state.target_pose, *p_end_effector);

    if (!solution.empty())
    {
        p_robot.setJointValues(solution);
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

    p_state.target_updated.store(false);
}

// ----------------------------------------------------------------------------
void physics_loop(robotik::Robot& p_robot,
                  const robotik::scene::Node* p_end_effector,
                  RobotState& p_state)
{
    const auto physics_dt = std::chrono::microseconds(1000); // 1kHz = 1ms
    auto next_update = std::chrono::steady_clock::now();

    std::cout << "Physics thread started at 1kHz" << std::endl;

    while (p_state.physics_running.load())
    {
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_seconds =
            std::chrono::duration<double>(current_time - p_state.start_time)
                .count();

        // Lock robot for updates
        {
            std::lock_guard<std::mutex> lock(p_state.robot_mutex);

            ControlMode mode = p_state.control_mode.load();
            if (mode == ControlMode::ANIMATION)
            {
                animate(p_robot, elapsed_seconds);
            }
            else if (mode == ControlMode::INVERSE_KINEMATICS)
            {
                handle_inverse_kinematics(p_robot, p_end_effector, p_state);
            }
        }

        // Sleep until next update
        next_update += physics_dt;
        std::this_thread::sleep_until(next_update);
    }

    std::cout << "Physics thread stopped" << std::endl;
}

// ----------------------------------------------------------------------------
void display_usage(const std::string& p_program_name)
{
    std::cout << "Usage: " << p_program_name << " [OPTIONS] [urdf_file]"
              << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help     Display this help message and exit"
              << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  urdf_file      Path to the URDF file to load" << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  1-5            Change camera view" << std::endl;
    std::cout << "  M              Toggle between Animation and Inverse "
                 "Kinematics mode"
              << std::endl;
    std::cout << "  Z/S            Move target +/-X (IK mode, AZERTY)"
              << std::endl;
    std::cout << "  Q/D            Move target +/-Y (IK mode, AZERTY)"
              << std::endl;
    std::cout << "  A/E            Move target +/-Z (IK mode, AZERTY)"
              << std::endl;
}

// ----------------------------------------------------------------------------
//! \brief Parse the command line arguments.
//! \param argc The number of command line arguments.
//! \param argv The command line arguments.
//! \return A unique pointer to the robot arm.
// ----------------------------------------------------------------------------
static std::unique_ptr<robotik::Robot> parse_command_line(int argc,
                                                          char* argv[])
{
    // Check for help option
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            display_usage(argv[0]);
            return nullptr;
        }
    }

    // Check if a URDF file was provided as command line argument
    if (argc > 1)
    {
        std::string urdf_file = argv[1];
        robotik::URDFParser parser;
        auto robot = parser.load(urdf_file);
        if (!robot)
        {
            std::cerr << "Failed to load robot from '" << urdf_file
                      << "': " << parser.getError() << std::endl;
            return nullptr;
        }
        std::cout << "Loaded robot from: " << urdf_file << std::endl;
        return robot;
    }
    else
    {
        // No URDF file provided, create a simple robot
        // auto robot = create_simple_robot();
        // return robot;
        return nullptr;
    }
}

// ----------------------------------------------------------------------------
std::optional<robotik::OpenGLViewer::CameraViewType>
handle_input(int p_key,
             const robotik::scene::Node* p_end_effector,
             RobotState& p_state,
             const robotik::OpenGLViewer& p_viewer)
{
    if (!p_viewer.isKeyPressed(p_key))
        return std::nullopt;

    // Camera views
    const std::unordered_map<int, robotik::OpenGLViewer::CameraViewType>
        camera_keys = {
            { GLFW_KEY_1, robotik::OpenGLViewer::CameraViewType::PERSPECTIVE },
            { GLFW_KEY_2, robotik::OpenGLViewer::CameraViewType::TOP },
            { GLFW_KEY_3, robotik::OpenGLViewer::CameraViewType::FRONT },
            { GLFW_KEY_4, robotik::OpenGLViewer::CameraViewType::SIDE },
            { GLFW_KEY_5, robotik::OpenGLViewer::CameraViewType::ISOMETRIC }
        };

    auto it = camera_keys.find(p_key);
    if (it != camera_keys.end())
        return it->second;

    // Mode switching
    if (p_key == GLFW_KEY_M)
    {
        auto new_mode = (p_state.control_mode.load() == ControlMode::ANIMATION)
                            ? ControlMode::INVERSE_KINEMATICS
                            : ControlMode::ANIMATION;
        p_state.control_mode.store(new_mode);
        std::cout << "Mode: "
                  << (new_mode == ControlMode::ANIMATION ? "Animation" : "IK")
                  << std::endl;

        if (new_mode == ControlMode::INVERSE_KINEMATICS && p_end_effector)
        {
            p_state.target_pose = robotik::utils::transformToPose(
                p_end_effector->worldTransform());
            std::cout << "Target pose initialized" << std::endl;
        }
    }

    // Help toggle
    if (p_key == GLFW_KEY_H)
    {
        p_state.display_info.show_help = !p_state.display_info.show_help;
        std::cout << "Help: " << (p_state.display_info.show_help ? "ON" : "OFF")
                  << std::endl;
    }

    return std::nullopt;
}

// ----------------------------------------------------------------------------
void handle_continuous_input(RobotState& p_state,
                             const robotik::OpenGLViewer& p_viewer)
{
    static int debug_counter = 0;
    debug_counter++;

    if (p_state.control_mode.load() != ControlMode::INVERSE_KINEMATICS)
    {
        if (debug_counter % 60 == 0) // Log every second (60 FPS)
            std::cout << "Not in IK mode, current mode: "
                      << (p_state.control_mode.load() == ControlMode::ANIMATION
                              ? "Animation"
                              : "IK")
                      << std::endl;
        return;
    }

    // IK target position controls (continuous movement while pressed)
    if (p_viewer.isKeyPressed(GLFW_KEY_Z)) // +X (Z en AZERTY = W en QWERTY)
    {
        p_state.target_pose(0) += 0.05;
        p_state.target_updated.store(true);
        std::cout << "Z pressed - Target X: " << p_state.target_pose(0)
                  << std::endl;
    }
    if (p_viewer.isKeyPressed(GLFW_KEY_S)) // -X
    {
        p_state.target_pose(0) -= 0.05;
        p_state.target_updated.store(true);
        std::cout << "S pressed - Target X: " << p_state.target_pose(0)
                  << std::endl;
    }
    if (p_viewer.isKeyPressed(GLFW_KEY_Q)) // -Y (Q en AZERTY = A en QWERTY)
    {
        p_state.target_pose(1) -= 0.05;
        p_state.target_updated.store(true);
        std::cout << "Q pressed - Target Y: " << p_state.target_pose(1)
                  << std::endl;
    }
    if (p_viewer.isKeyPressed(GLFW_KEY_D)) // +Y
    {
        p_state.target_pose(1) += 0.05;
        p_state.target_updated.store(true);
        std::cout << "D pressed - Target Y: " << p_state.target_pose(1)
                  << std::endl;
    }
    if (p_viewer.isKeyPressed(GLFW_KEY_A)) // +Z (A en AZERTY)
    {
        p_state.target_pose(2) += 0.05;
        p_state.target_updated.store(true);
        std::cout << "A pressed - Target Z: " << p_state.target_pose(2)
                  << std::endl;
    }
    if (p_viewer.isKeyPressed(GLFW_KEY_E)) // -Z
    {
        p_state.target_pose(2) -= 0.05;
        p_state.target_updated.store(true);
        std::cout << "E pressed - Target Z: " << p_state.target_pose(2)
                  << std::endl;
    }
}

} // namespace robotik_viewer

// ----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    using namespace robotik_viewer;

    // Parse the command line arguments and create the robot given by the
    // user.
    auto robot = parse_command_line(argc, argv);
    if (!robot)
    {
        return EXIT_FAILURE;
    }

    // Initialize the viewer.
    robotik::OpenGLViewer viewer(1024, 768, "Robotik Viewer Demo");
    if (!viewer.initialize())
    {
        std::cerr << "Failed to initialize viewer: " << viewer.error()
                  << std::endl;
        return EXIT_FAILURE;
    }

    robotik::debug::printRobot(*robot, true);

    // Find the end effector for inverse kinematics
    auto end_effector =
        robotik::scene::Node::find(robot->root(), "link_end_effector");
    if (!end_effector)
    {
        std::cerr << "Warning: End effector not found, inverse kinematics "
                     "disabled"
                  << std::endl;
    }

    // Initialize global state
    g_robot_state.start_time = std::chrono::steady_clock::now();
    if (end_effector)
    {
        g_robot_state.target_pose =
            robotik::utils::transformToPose(end_effector->worldTransform());
    }

    // Get base position for camera target
    auto base_position =
        robotik::utils::getTranslation(robot->root().worldTransform());

    // Set initial camera view
    viewer.setCameraView(robotik::OpenGLViewer::CameraViewType::SIDE,
                         base_position);

    // Start physics thread
    std::thread physics_thread(
        physics_loop, std::ref(*robot), end_effector, std::ref(g_robot_state));

    std::cout << "Controls:" << std::endl;
    std::cout << "  1-5: Camera views" << std::endl;
    std::cout << "  M: Toggle Animation/IK mode" << std::endl;
    std::cout << "  ZQSD+AE: Move target in IK mode" << std::endl;

    // Main render loop (runs at ~60 FPS)
    while (!viewer.shouldClose())
    {
        // Process input with both camera and IK controls
        viewer.processInput(
            [&viewer, &base_position, &end_effector](int key, int action)
            {
                auto camera_view =
                    handle_input(key, end_effector, g_robot_state, viewer);
                if (camera_view.has_value())
                {
                    viewer.setCameraView(camera_view.value(), base_position);
                }
            });

        // Handle continuous input for IK mode
        handle_continuous_input(g_robot_state, viewer);

        // Render the scene (robot state is updated by physics thread)
        {
            std::lock_guard<std::mutex> lock(g_robot_state.robot_mutex);
            viewer.render(*robot);
        }

        // Small delay to control frame rate (~60 FPS)
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    // Stop physics thread
    g_robot_state.physics_running.store(false);
    if (physics_thread.joinable())
    {
        physics_thread.join();
    }

    std::cout << "Viewer closed successfully" << std::endl;
    return EXIT_SUCCESS;
}