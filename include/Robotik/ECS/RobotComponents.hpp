/**
 * @file RobotComponents.hpp
 * @brief Tags and metadata for the robot root, links, tool, and parallel jaw gripper.
 */

#pragma once

#include <string>

namespace robotik::ecs
{

/**
 * @brief Marks the URDF root entity as the robot instance.
 */
struct RobotTag
{
    /** @brief Reserved lifecycle flag (always true for loaded robots). */
    bool alive = true;
};

/**
 * @brief Human-readable robot id and source model path.
 */
struct RobotIdentity
{
    /** @brief Name from the URDF @c robot element. */
    std::string name;

    /** @brief Path passed to @ref RobotLoader::instantiate. */
    std::string model_path;
};

/**
 * @brief URDF link name on a link entity (may also carry @ref Joint).
 */
struct Link
{
    /** @brief Link name in URDF and Pinocchio frames. */
    std::string name;
};

/**
 * @brief Marks the tool flange link used for IK and grasp geometry.
 */
struct EndEffector
{
    /** @brief Frame name for @ref PinocchioBackend::framePose and IK. */
    std::string name;
};

/**
 * @brief Parallel jaw gripper limits from URDF finger joints.
 *
 * Used by @ref OpenGripperSkill and @ref CloseGripperSkill when present
 * instead of @ref VacuumGripper.
 */
struct Gripper
{
    /** @brief Fully closed finger joint position. */
    double min_opening = 0.0;

    /** @brief Fully open finger joint position. */
    double max_opening = 0.08;
};

} // namespace robotik::ecs
