//=============================================================================
// Source: https://github.com/Robotik-Core/Robotik
// Copyright (c) 2020-2026 Quentin Quadrat.
// Distributed under MIT License.
//=============================================================================
// @file RobotComponents.hpp
// @brief ECS markers and descriptive fields for the loaded URDF robot.
//
// In Compages ECS, components are plain structs on entities. This file splits
// two roles:
// @li @b Tags — almost empty types whose *presence* classifies an entity for
//     queries (@ref RobotTag on the root, @ref EndEffector on the tool link).
//     Systems iterate @ref EndEffector entities instead of searching by name.
// @li @b Metadata — components that *carry data* copied or inferred at load time
//     (@ref RobotIdentity, @ref Link, @ref Gripper opening limits).
//
// They do not run physics or control; @ref RobotLoader attaches them when
// building the hierarchy from URDF. Parallel-jaw @ref Gripper lives here;
// vacuum grasp uses @ref VacuumGripper in @ref ObjectComponents.hpp.
//
// @par Examples (typical 6-DOF arm + tool0)
// @li Root entity @c robot — @ref RobotTag (tag only) +
//     @ref RobotIdentity{ "panda", "/path/panda.urdf" } (metadata).
// @li Entity @c link3 — @ref Link{ "link3" } plus @ref Joint and control
//     components from other headers; no tag struct, just named link data.
// @li Entity @c tool0 — @ref EndEffector{ "tool0" } (tag + frame name for IK).
//     @ref ApproachSkill uses @ref findTool to reach this entity, then reads
//     @c EndEffector::name for @ref PinocchioBackend::framePose.
// @li Finger link @c gripper_finger — optional @ref Gripper metadata
//     @c { min_opening, max_opening } for @ref CloseGripperSkill; a pick-and-place
//     scenario instead adds @ref VacuumGripper on @c tool0 in @ref Simulation::spawn.
//
// @par Query vs lookup
// @code
// // Tag: "give me the robot root" (see Simulation.cpp spawn parent)
// m_world.each<ecs::RobotTag>([&](compages::world::Entity root, ecs::RobotTag&) {
//     parent = root;
// });
// // Tag: "give me the tool link" (see Queries.hpp findTool)
// compages::world::Entity tool = robotik::findTool(m_world);
// auto const& frame = tool.get<ecs::EndEffector>().name;
// @endcode
//=============================================================================

#pragma once

#include "Compages/Core/Units.hpp"

#include <filesystem>
#include <string>

namespace robotik::ecs
{

// ****************************************************************************
//! @brief Marks the URDF root entity as the robot instance.
// ****************************************************************************
struct RobotTag
{
    //!< Reserved lifecycle flag (always true for loaded robots).
    bool alive = true;
};

// ****************************************************************************
//! @brief Human-readable robot id and source model path.
// ****************************************************************************
struct RobotIdentity
{
    //!< Name from the URDF @c robot element.
    std::string name;
    //!< Path passed to @ref RobotLoader::instantiate.
    std::filesystem::path model_path;
};

// ****************************************************************************
//! @brief URDF link name on a link entity (may also carry @ref Joint).
// ****************************************************************************
struct Link
{
    //!< Link name in URDF and Pinocchio frames.
    std::string name;
};

// ****************************************************************************
//! @brief Marks the tool flange link used for IK and grasp geometry.
// ****************************************************************************
struct EndEffector
{
    //!< Frame name for @ref PinocchioBackend::framePose and IK.
    std::string name;
};

// ****************************************************************************
//! @brief Parallel jaw gripper limits from URDF finger joints.
//!
//! Used by @ref OpenGripperSkill and @ref CloseGripperSkill when present
//! instead of @ref VacuumGripper.
// ****************************************************************************
struct Gripper
{
    //!< Fully closed finger opening (SI: m).
    Length min_opening{};
    //!< Fully open finger opening (SI: m).
    Length max_opening{ 0.08 };
};

} // namespace robotik::ecs
