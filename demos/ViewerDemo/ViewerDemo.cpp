#include "Robotik/Robotik.hpp"
#include "Robotik/Viewer.hpp"

#include <iostream>

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

    // Set camera view
    auto base_position = robotik::utils::getTranslation(
        robot->getRootNode()->getWorldTransform());
    viewer.setCameraView(robotik::Viewer::CameraViewType::ISOMETRIC,
                         base_position);

    // Main render loop
    while (!viewer.shouldClose())
    {
        // Process input
        viewer.processInput();

        // Render the scene
        viewer.render();

        // Simple animation
        static double time = 0.0;
        time += 0.016; // ~60 FPS

        joint_values[0] = std::sin(time) * 0.5;
        joint_values[1] = std::cos(time * 0.7) * 0.3;
        joint_values[2] = std::sin(time * 1.2) * 0.4;
        robot->setJointValues(joint_values);
    }

    std::cout << "Viewer closed successfully" << std::endl;

    return EXIT_SUCCESS;
}