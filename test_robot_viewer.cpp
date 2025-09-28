/**
 * @file test_robot_viewer.cpp
 * @brief Test program for robot loading and rendering.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Viewer/RobotViewerApplication.hpp"
#include <iostream>
#include <vector>

int main()
{
    std::cout << "Testing robot loading and rendering..." << std::endl;

    // Create application configuration
    robotik::viewer::RobotViewerApplication::Configuration config;
    config.window_width = 1024;
    config.window_height = 768;
    config.window_title = "Robot Viewer Test";
    config.target_fps = 60;
    config.target_physics_hz = 15;

    // Create application
    robotik::viewer::RobotViewerApplication app(config);

    // Initialize the application
    if (!app.initialize())
    {
        std::cerr << "Failed to initialize application: " << app.error()
                  << std::endl;
        return -1;
    }

    std::cout << "Application initialized successfully!" << std::endl;

    // Get the robot viewer
    auto& robot_viewer = app.robotViewer();

    // Set mesh base path
    robot_viewer.setMeshBasePath("data/");

    // Try to load some robots from URDF files
    std::vector<std::string> urdf_files = {
        "cartesian_robot.urdf",         "scara_robot.urdf",
        "simple_diff_drive_robot.urdf", "simple_prismatic_robot.urdf",
        "simple_revolute_robot.urdf",   "T12.urdf"
    };

    for (const auto& urdf_file : urdf_files)
    {
        std::string robot_name =
            "robot_" + urdf_file.substr(0, urdf_file.find('.'));

        if (robot_viewer.loadRobot(robot_name, urdf_file))
        {
            std::cout << "Successfully loaded robot: " << robot_name << " from "
                      << urdf_file << std::endl;

            // Set some joint values for demonstration
            std::vector<double> joint_values = {
                0.0, 0.5, -0.3, 0.0, 0.0, 0.0
            };
            if (robot_viewer.setRobotJointValues(robot_name, joint_values))
            {
                std::cout << "Set joint values for " << robot_name << std::endl;
            }

            // Set robot position
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            transform(0, 3) = 1.0f; // Move along X axis
            transform(1, 3) = 0.5f; // Move along Y axis
            robot_viewer.setRobotPose(robot_name, transform);

            // Set robot scale
            robot_viewer.setRobotScale(robot_name, 0.5f);
        }
        else
        {
            std::cout << "Failed to load robot: " << robot_name << " from "
                      << urdf_file << " - " << robot_viewer.error()
                      << std::endl;
        }
    }

    std::cout << "Loaded " << robot_viewer.robotRenderer().getRobotCount()
              << " robots" << std::endl;

    // Run the application
    if (!app.run())
    {
        std::cerr << "Application failed to run: " << app.error() << std::endl;
        return -1;
    }

    std::cout << "Robot viewer test completed successfully!" << std::endl;
    return 0;
}
