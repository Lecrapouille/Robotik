#include "Robotik/Robotik.hpp"
#include "Robotik/Viewer.hpp"
#include <iostream>

int main()
{
    try
    {
        std::cout << "Robotik Viewer Demo" << std::endl;
        std::cout << "===================" << std::endl;

        // Create a simple 2-DOF robot arm
        robotik::RobotArm robot("Simple2DOF");

        // Create the robot structure
        auto base = robotik::Node::create<robotik::Node>("base");

        // First joint (revolute around Z axis)
        auto joint1 = base->createChild<robotik::Joint>(
            "joint1", robotik::Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1));
        joint1.setLimits(-M_PI, M_PI);

        // First link
        auto link1 = base->createChild<robotik::Node>("link1");
        link1.setLocalTransform(robotik::utils::createTransform(
            Eigen::Vector3d(0, 0, 1.0), 0, 0, 0));

        // Second joint (revolute around Y axis)
        auto joint2 = link1.createChild<robotik::Joint>(
            "joint2", robotik::Joint::Type::REVOLUTE, Eigen::Vector3d(0, 1, 0));
        joint2.setLimits(-M_PI, M_PI);

        // Second link
        auto link2 = link1.createChild<robotik::Node>("link2");
        link2.setLocalTransform(robotik::utils::createTransform(
            Eigen::Vector3d(0, 0, 1.0), 0, 0, 0));

        // End effector
        auto endEffector = link2.createChild<robotik::Node>("end_effector");
        endEffector.setLocalTransform(robotik::utils::createTransform(
            Eigen::Vector3d(0, 0, 0.5), 0, 0, 0));

        // Setup the robot
        robot.setupRobot(std::move(base), endEffector);

        std::cout << "Robot created with " << robot.getJointNames().size()
                  << " joints" << std::endl;

        // Create and initialize viewer
        robotik::Viewer viewer(1200, 800, "Robotik Simple Viewer");
        if (!viewer.initialize())
        {
            std::cerr << "Failed to initialize viewer" << std::endl;
            return -1;
        }

        std::cout << "Viewer initialized successfully" << std::endl;

        // Set the robot to visualize
        viewer.setRobot(&robot);

        // Set initial joint values
        std::vector<double> jointValues = { 0.0, 0.0 };
        robot.setJointValues(jointValues);

        std::cout << "Starting render loop... (Press Ctrl+C to exit)"
                  << std::endl;

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

            jointValues[0] = std::sin(time) * 0.5;
            jointValues[1] = std::cos(time * 0.7) * 0.3;

            robot.setJointValues(jointValues);
        }

        std::cout << "Viewer closed successfully" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}