// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file WorldModel.hpp
/// What the robot believes about the objects around it.
///
/// Skills read this belief, never the simulator ground truth: on a real robot
/// there is none. Priors come from the mission (scenario layout), perception
/// refines them, a simulator without rendering may feed it with an oracle.
#pragma once

#include "Robotik/Perception/Detection.hpp"

#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// Belief about one object, in the robot base frame (in the world frame when
/// several robots share the beliefs: Simulation::beliefFrame).
///
/// @code
/// robotik::WorldObject const* cube = p_context.world_model.find("red_cube");
/// if (cube != nullptr && cube->observed())
///     aimAt(cube->position.x, cube->position.y, cube->top());
/// @endcode
// ****************************************************************************
struct WorldObject
{
    /// Object name, matched to the detection labels.
    std::string name;
    /// Estimated center (m).
    Vector3 position;
    /// Known extents (m); zero when unknown.
    Vector3 size;
    /// Confidence of the last observation; 0 for a prior never observed.
    float confidence = 0.0f;
    /// Time of the last observation.
    Seconds seen{ -1.0 };
    /// Number of observations since the last reset.
    std::uint32_t observations = 0;

    // -------------------------------------------------------------------------
    /// @return True when the object was seen at least once since the reset;
    ///   false for a prior only.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool observed() const
    {
        return observations > 0u;
    }

    // -------------------------------------------------------------------------
    /// @return Height of the top face (center plus half the extent).
    // -------------------------------------------------------------------------
    [[nodiscard]] double top() const
    {
        return position.z + 0.5 * size.z;
    }
};

// ****************************************************************************
/// Dense list of beliefs, matched to detections by label.
///
/// Objects of known size are assumed to rest on a support: an observation
/// refines their horizontal position and keeps their height. Without a 3D
/// point, a detection center is lifted by intersecting its pixel ray with the
/// top plane of the object.
///
/// @code
/// robotik::WorldModel beliefs;
/// beliefs.add("red_cube", robotik::Vector3(0.4, 0.0, 0.02),
///             robotik::Vector3(0.04, 0.04, 0.04));      // prior
/// beliefs.gate(0.10);                                   // ignore jumps > 10
/// cm beliefs.update(pipeline.process(camera.frame()));     // perception
/// @endcode
// ****************************************************************************
class WorldModel
{
public:

    // -------------------------------------------------------------------------
    /// Declares an object with a prior position, or resets the prior of a
    /// known one.
    /// @param p_name Object name, matched to the detection labels.
    /// @param p_position Prior center (m).
    /// @param p_size Known extents (m); zero when unknown.
    /// @return The belief.
    // -------------------------------------------------------------------------
    WorldObject&
    add(std::string p_name, Vector3 p_position, Vector3 p_size = {});

    // -------------------------------------------------------------------------
    /// Finds a belief by name.
    /// @param p_name Object name.
    /// @return The belief, or null.
    // -------------------------------------------------------------------------
    [[nodiscard]] WorldObject* find(std::string_view p_name);

    /// @copydoc find(std::string_view)
    [[nodiscard]] WorldObject const* find(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    /// Rejects observations too far from the current belief: a detector
    /// matching the wrong thing (a robot link of the same color) must not
    /// teleport an object. Unbounded by default.
    /// @param p_distance Largest accepted jump (m).
    // -------------------------------------------------------------------------
    void gate(double p_distance)
    {
        m_gate = p_distance;
    }

    // -------------------------------------------------------------------------
    /// @return The largest accepted jump (m).
    // -------------------------------------------------------------------------
    [[nodiscard]] double gate() const
    {
        return m_gate;
    }

    // -------------------------------------------------------------------------
    /// Records that an object was seen. Unknown names and observations
    /// outside the @ref gate are ignored.
    /// @param p_name Object name.
    /// @param p_point Seen position (robot base frame).
    /// @param p_confidence Confidence of the detector in [0, 1].
    /// @param p_stamp Time of the observation.
    /// @return True when the belief was updated.
    // -------------------------------------------------------------------------
    bool observe(std::string_view p_name,
                 Vector3 const& p_point,
                 float p_confidence,
                 Seconds p_stamp);

    // -------------------------------------------------------------------------
    /// Overwrites the belief (oracle / RL). No gate. Stamps @p_stamp so
    /// Detect sees a fresh observation.
    /// @param p_name Object name.
    /// @param p_position New center (m).
    /// @param p_stamp Time of the observation.
    // -------------------------------------------------------------------------
    void
    place(std::string_view p_name, Vector3 const& p_position, Seconds p_stamp);

    // -------------------------------------------------------------------------
    /// Folds the detections of one frame into the beliefs.
    /// @param p_detections Detections with the camera pose of the frame.
    // -------------------------------------------------------------------------
    void update(Detections const& p_detections);

    // -------------------------------------------------------------------------
    /// @return Every belief, in declaration order.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::span<WorldObject const> objects() const
    {
        return m_objects;
    }

    // -------------------------------------------------------------------------
    /// Forgets every belief.
    // -------------------------------------------------------------------------
    void clear()
    {
        m_objects.clear();
    }

private:

    /// Beliefs, in declaration order.
    std::vector<WorldObject> m_objects;
    /// Largest accepted jump (m).
    double m_gate = std::numeric_limits<double>::infinity();
};

} // namespace robotik
