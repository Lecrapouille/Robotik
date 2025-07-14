#include "Robotik/Robotik.hpp"
#include "Robotik/Viewer.hpp"

#include <chrono>
#include <iostream>
#include <thread>

// ----------------------------------------------------------------------------
static std::unique_ptr<robotik::RobotArm> createRobot()
{
    // Create a simple robot arm
    auto robot = std::make_unique<robotik::RobotArm>("SimpleArm");

    // Create root node (base)
    auto base = robotik::Node::create<robotik::Node>("base");

    // Create joints and links
    auto& joint1 = base->createChild<robotik::Joint>(
        "joint1", robotik::Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1));
    joint1.setLimits(-M_PI, M_PI);

    auto& link1 = base->createChild<robotik::Node>("link1");
    link1.setLocalTransform(
        robotik::utils::createTransform(Eigen::Vector3d(0, 0, 0.5), 0, 0, 0));

    auto& joint2 = link1.createChild<robotik::Joint>(
        "joint2", robotik::Joint::Type::REVOLUTE, Eigen::Vector3d(0, 1, 0));
    joint2.setLimits(-M_PI, M_PI);

    auto& link2 = link1.createChild<robotik::Node>("link2");
    link2.setLocalTransform(
        robotik::utils::createTransform(Eigen::Vector3d(0, 0, 0.5), 0, 0, 0));

    auto& joint3 = link2.createChild<robotik::Joint>(
        "joint3", robotik::Joint::Type::REVOLUTE, Eigen::Vector3d(0, 1, 0));
    joint3.setLimits(-M_PI, M_PI);

    auto& endEffector = link2.createChild<robotik::Node>("end_effector");
    endEffector.setLocalTransform(
        robotik::utils::createTransform(Eigen::Vector3d(0, 0, 0.3), 0, 0, 0));

    // Setup the robot
    robot->setupRobot(std::move(base), endEffector);

    return robot;
}

// ----------------------------------------------------------------------------
static void animate(robotik::RobotArm& p_robot)
{
    // Update animation time (slower for more visible animation)
    float animation_time = static_cast<float>(glfwGetTime()) * 0.5f;

    // Get current joint values
    std::vector<double> joint_values = p_robot.getJointValues();

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
int main()
{
    auto robot = createRobot();

    robotik::Viewer viewer(1024, 768, "Robotik Viewer Demo");
    if (!viewer.initialize())
    {
        std::cerr << "Failed to initialize viewer" << std::endl;
        return EXIT_FAILURE;
    }

    // Set the robot to visualize
    viewer.setRobot(*robot);

    // Set initial joint values
    std::vector<double> joint_values = { 0.0, 0.5, -0.3 };
    robot->setJointValues(joint_values);

    // Get base position for camera target
    auto base_position = robotik::utils::getTranslation(
        robot->getRootNode()->getWorldTransform());

    // Set initial camera view
    viewer.setCameraView(robotik::Viewer::CameraViewType::SIDE, base_position);

    // Main render loop
    while (!viewer.shouldClose())
    {
        // Process input with camera controls
        viewer.processInput(
            [&viewer, &base_position](int key, int action)
            {
                if (action == GLFW_PRESS)
                {
                    switch (key)
                    {
                        case GLFW_KEY_1:
                            viewer.setCameraView(
                                robotik::Viewer::CameraViewType::PERSPECTIVE,
                                base_position);
                            break;
                        case GLFW_KEY_2:
                            viewer.setCameraView(
                                robotik::Viewer::CameraViewType::TOP,
                                base_position);
                            break;
                        case GLFW_KEY_3:
                            viewer.setCameraView(
                                robotik::Viewer::CameraViewType::FRONT,
                                base_position);
                            break;
                        case GLFW_KEY_4:
                            viewer.setCameraView(
                                robotik::Viewer::CameraViewType::SIDE,
                                base_position);
                            break;
                        case GLFW_KEY_5:
                            viewer.setCameraView(
                                robotik::Viewer::CameraViewType::ISOMETRIC,
                                base_position);
                            break;
                        default:
                            break;
                    }
                }
            });

        // Animate the robot
        animate(*robot);

        // Render the scene
        viewer.render();

        // Small delay to control frame rate (~60 FPS)
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    std::cout << "Viewer closed successfully" << std::endl;

    return EXIT_SUCCESS;
}