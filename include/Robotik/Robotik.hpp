/**
 * @file Robotik.hpp
 * @brief Main entry point for the Robotik library.
 *
 * This is the recommended header to include for most use cases.
 * It provides access to the complete public API of the Robotik library.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 *
 * @mainpage Robotik - Robot Kinematics Library
 *
 * @section intro_sec Introduction
 *
 * Robotik is a C++ library for robot kinematics and URDF parsing.
 *
 * @section quick_start Quick Start
 *
 * The simplest way to use Robotik is through the URDF parser:
 *
 * @code{.cpp}
 * #include <Robotik/Robotik.hpp>
 *
 * int main()
 * {
 *     // 1. Create a parser
 *     robotik::URDFParser parser;
 *
 *     // 2. Load a robot from a URDF file
 *     auto robot = parser.load("path/to/robot.urdf");
 *
 *     // 3. Check for errors
 *     if (!robot) {
 *         std::cerr << "Error: " << parser.error() << std::endl;
 *         return 1;
 *     }
 *
 *     // 4. Use the robot
 *     std::cout << "Robot name: " << robot->name() << std::endl;
 *     std::cout << "Joint names: ";
 *     for (const auto& name : robot->jointNames()) {
 *         std::cout << name << " ";
 *     }
 *     std::cout << std::endl;
 *
 *     // 5. Control the robot
 *     robot->setNeutralPosition();
 *     auto positions = robot->jointPositions();
 *
 *     // 6. Perform inverse kinematics
 *     robotik::Pose target_pose;
 *     target_pose << 0.5, 0.0, 0.5, 0.0, 0.0, 0.0;
 *     auto joint_values = robot->inverseKinematics(
 *         target_pose,
 *         robot->root()
 *     );
 *
 *     return 0;
 * }
 * @endcode
 *
 * @section advanced_usage Advanced Usage
 *
 * For advanced users who need direct access to the scene graph:
 *
 * @code{.cpp}
 * #include <Robotik/Robotik.hpp>
 *
 * // Access joints and links directly
 * const auto& joint = robot->joint("joint_name");
 * const auto& link = robot->link("link_name");
 *
 * // Get transformations
 * const auto& root_node = robot->root();
 * auto world_transform = root_node.worldTransform();
 * @endcode
 *
 * @section api_organization API Organization
 *
 * The Robotik API is organized as follows:
 *
 * - **Main Entry Point**: URDFParser - Load robots from URDF files
 * - **Robot Control**: Robot - High-level robot manipulation
 * - **Types**: Common types (Transform, Pose, Jacobian)
 *
 * Most users only need to include <Robotik/Robotik.hpp> which provides
 * everything needed for typical use cases.
 */

#pragma once

// ============================================================================
// Forward Declarations
// ============================================================================

#include "Robotik/Core/ClassesFwd.hpp"

// ============================================================================
// Common Types
// ============================================================================

#include "Robotik/Core/Types.hpp"

// ============================================================================
// Include full API after forward declarations
// ============================================================================

#include "Robotik/Core/Robot.hpp"
#include "Robotik/Core/URDFParser.hpp"

// ============================================================================
// Extended API: Additional modules for advanced functionality
// ============================================================================

#include "Robotik/Core/IKSolver.hpp"
#include "Robotik/Core/PhysicsSimulator.hpp"

namespace robotik
{

/**
 * @brief Convenience function to quickly load a robot from a URDF file.
 *
 * This is a shorthand for creating a URDFParser and calling load().
 *
 * @param p_filename Path to the URDF file
 * @param p_error_message Optional output parameter for error message
 * @return Unique pointer to Robot if successful, nullptr otherwise
 *
 * @code{.cpp}
 * std::string error;
 * auto robot = robotik::loadRobot("robot.urdf", &error);
 * if (!robot) {
 *     std::cerr << "Error: " << error << std::endl;
 * }
 * @endcode
 */
inline std::unique_ptr<Robot> loadRobot(const std::string& p_filename,
                                        std::string* p_error_message = nullptr)
{
    URDFParser parser;
    auto robot = parser.load(p_filename);

    if (!robot && p_error_message)
    {
        *p_error_message = parser.error();
    }

    return robot;
}

} // namespace robotik
