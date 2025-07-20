#include "Robotik/Debug.hpp"
#include "Robotik/Parser.hpp"
#include "Robotik/Robot.hpp"
#include "Robotik/Viewer.hpp"

#include <iostream>
#include <optional>
#include <string>
#include <thread>

// ----------------------------------------------------------------------------
//! \brief Create a simple robot arm. Called if no URDF file is provided in the
//! command line.
//! \return A unique pointer to the robot arm.
//! \fixme missing links
// ----------------------------------------------------------------------------
static std::unique_ptr<robotik::Robot> create_simple_robot()
{
    // Create a simple robot arm
    auto robot = std::make_unique<robotik::Robot>("SimpleArm");

    // Create root node (base)
    auto base = robotik::Node::create<robotik::Node>("base");

    // Create joints and links
    auto& joint1 = base->createChild<robotik::Joint>(
        "joint1", robotik::Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1));
    joint1.limits(-M_PI, M_PI);

    auto& link1 = base->createChild<robotik::Node>("link1");
    link1.localTransform(
        robotik::utils::createTransform(Eigen::Vector3d(0, 0, 0.5), 0, 0, 0));

    auto& joint2 = link1.createChild<robotik::Joint>(
        "joint2", robotik::Joint::Type::REVOLUTE, Eigen::Vector3d(0, 1, 0));
    joint2.limits(-M_PI, M_PI);

    auto& link2 = link1.createChild<robotik::Node>("link2");
    link2.localTransform(
        robotik::utils::createTransform(Eigen::Vector3d(0, 0, 0.5), 0, 0, 0));

    auto& joint3 = link2.createChild<robotik::Joint>(
        "joint3", robotik::Joint::Type::REVOLUTE, Eigen::Vector3d(0, 1, 0));
    joint3.limits(-M_PI, M_PI);

    auto& endEffector = link2.createChild<robotik::Node>("end_effector");
    endEffector.localTransform(
        robotik::utils::createTransform(Eigen::Vector3d(0, 0, 0.3), 0, 0, 0));

    // Setup the robot
    // FIXME robot->setupRobot(std::move(base), endEffector);

    return robot;
}

// ----------------------------------------------------------------------------
//! \brief Animate the robot.
//! \param p_robot The robot to animate.
// ----------------------------------------------------------------------------
static void animate(robotik::Robot& p_robot)
{
    // Update animation time (slower for more visible animation)
    float animation_time = static_cast<float>(glfwGetTime()) * 0.5f;

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

    // Apply the new joint values to the robot (this updates transforms
    // automatically)
    p_robot.setJointValues(joint_values);
}

// ----------------------------------------------------------------------------
//! \brief Display the usage of the program.
//! \param p_program_name The name of the program.
// ----------------------------------------------------------------------------
static void display_usage(const std::string& p_program_name)
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
    std::cout
        << "                 If not provided, a simple robot will be created"
        << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << p_program_name
              << "                    # Create simple robot" << std::endl;
    std::cout << "  " << p_program_name
              << " robot.urdf         # Load robot from URDF file" << std::endl;
    std::cout << "  " << p_program_name
              << " --help             # Show this help message" << std::endl;
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
        auto robot = create_simple_robot();
        std::cout << "Created simple robot arm (no URDF file specified)"
                  << std::endl;
        return robot;
    }
}

// ----------------------------------------------------------------------------
//! \brief Handle keyboard input for camera controls.
//! \param p_key The key that was pressed.
//! \param p_action The action (press/release).
//! \return The selected camera view type, or std::nullopt if no view change.
// ----------------------------------------------------------------------------
static std::optional<robotik::Viewer::CameraViewType>
handle_camera_input(int p_key, int p_action)
{
    if (p_action == GLFW_PRESS)
    {
        switch (p_key)
        {
            case GLFW_KEY_1:
                return robotik::Viewer::CameraViewType::PERSPECTIVE;
            case GLFW_KEY_2:
                return robotik::Viewer::CameraViewType::TOP;
            case GLFW_KEY_3:
                return robotik::Viewer::CameraViewType::FRONT;
            case GLFW_KEY_4:
                return robotik::Viewer::CameraViewType::SIDE;
            case GLFW_KEY_5:
                return robotik::Viewer::CameraViewType::ISOMETRIC;
            default:
                return std::nullopt;
        }
    }
    return std::nullopt;
}

// ----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    // Parse the command line arguments and create the robot given by the user.
    auto robot = parse_command_line(argc, argv);
    if (!robot)
    {
        return EXIT_FAILURE;
    }

    // Initialize the viewer.
    robotik::Viewer viewer(1024, 768, "Robotik Viewer Demo");
    if (!viewer.initialize())
    {
        std::cerr << "Failed to initialize viewer" << std::endl;
        return EXIT_FAILURE;
    }

    // Set initial joint values
    // std::vector<double> joint_values = { 0.0, 0.5, -0.3 };
    // robot->setJointValues(joint_values);

    robotik::debug::printRobot(*robot);

    // Get base position for camera target
    auto base_position =
        robotik::utils::getTranslation(robot->getRootNode()->worldTransform());

    // Set initial camera view
    viewer.setCameraView(robotik::Viewer::CameraViewType::SIDE, base_position);

    // Main render loop
    while (!viewer.shouldClose())
    {
        // Process input with camera controls
        viewer.processInput(
            [&viewer, &base_position](int key, int action)
            {
                auto camera_view = handle_camera_input(key, action);
                if (camera_view.has_value())
                {
                    viewer.setCameraView(camera_view.value(), base_position);
                }
            });

        // Animate the robot
        animate(*robot);

        // Render the scene
        viewer.render(*robot);

        // Small delay to control frame rate (~60 FPS)
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "Viewer closed successfully" << std::endl;
    return EXIT_SUCCESS;
}