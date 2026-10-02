//=============================================================================
// Source: https://github.com/Robotik-Core/Robotik
// Copyright (c) 2020-2026 Quentin Quadrat.
// Distributed under MIT License.
//=============================================================================
// @file PerceptionComponents.hpp
// @brief Camera parameters and 2D detection output on a camera entity.
//
// Robotik does not render or run ML here: the simulator (or your app) draws
// from @ref CameraSensor settings, fills @ref DetectedObjects after each frame
// (e.g. via @ref ColorDetector). Skills read ECS only—@ref DetectSkill waits
// until @ref Detection::label matches a @ref SceneObject name.
//
// @par Typical setup (@c pick_and_place.yml)
// @li Scenario YAML sets @c world.robot.camera (link, resolution, FOV).
// @li @ref Simulation::spawn parents a camera entity on @c link6 with
//     @ref CameraSensor{ 320, 240, 70° } and empty @ref DetectedObjects.
// @li Each frame, the app renders that view, runs detection, writes
//     @c detected.items and bumps @c detected.frame.
//
// @par Example detection row
// @code
// ecs::Detection{
//     .label = "red_cube",   // same string as SceneObject::name
//     .confidence = 0.85f,
//     .x0 = 120, .y0 = 80, .x1 = 200, .y1 = 160   // pixels, top-left origin
// };
// @endcode
//
// @par Skill consumption
// @code
// p_world.each<ecs::DetectedObjects>([&](Entity, ecs::DetectedObjects& d) {
//     for (ecs::Detection const& box : d.items)
//         if (box.label == "red_cube") { /* DetectSkill succeeds */ }
// });
// @endcode
//=============================================================================

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace robotik::ecs
{

// ****************************************************************************
// @brief Resolution and field of view for a link-mounted camera entity.
//
// Rendering and readback are done by the application; this library only
// stores parameters and detection output.
// ****************************************************************************
struct CameraSensor
{
    //!< Image width in pixels.
    std::uint32_t width = 320;
    //!< Image height in pixels.
    std::uint32_t height = 240;
    //!< Vertical field of view in degrees.
    float fov_degrees = 70.0f;
};

// ****************************************************************************
// @brief Axis-aligned bounding box in image space.
//
// Origin is the top-left corner of the image; @c y increases downward.
// ****************************************************************************
struct Detection
{
    //!< Class or object name (e.g. scenario object name).
    std::string label;
    //!< Heuristic confidence in @c [0, 1].
    float confidence = 0.0f;
    //!< Left edge in pixels.
    int x0 = 0;
    //!< Top edge in pixels.
    int y0 = 0;
    //!< Right edge in pixels (inclusive).
    int x1 = 0;
    //!< Bottom edge in pixels (inclusive).
    int y1 = 0;
};

// ****************************************************************************
// @brief Latest detections written by the perception pipeline each frame.
// ****************************************************************************
struct DetectedObjects
{
    //!< Detections from the most recent update.
    std::vector<Detection> items;
    //!< Monotonic frame counter incremented by the application.
    std::uint64_t frame = 0;
};

} // namespace robotik::ecs
