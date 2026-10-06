// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file WorldModel.hpp
//! @brief What the robot believes about the objects around it.
//!
//! Skills read this belief, never the simulator ground truth: on a real robot
//! there is none. Priors come from the mission (scenario layout), perception
//! refines them, a simulator without rendering may feed it with an oracle.
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
//! @brief Belief about one object, in the robot base frame.
// ****************************************************************************
struct WorldObject
{
    std::string name;
    //!< Estimated center (m).
    Vector3 position;
    //!< Known extents (m); zero when unknown.
    Vector3 size;
    //!< Confidence of the last observation; 0 for a prior never observed.
    float confidence = 0.0f;
    //!< Time of the last observation.
    Seconds seen{ -1.0 };
    //!< Number of observations since the last reset.
    std::uint32_t observations = 0;

    [[nodiscard]] bool observed() const
    {
        return observations > 0u;
    }

    //! @brief Height of the top face (center plus half the extent).
    [[nodiscard]] double top() const
    {
        return position.z + 0.5 * size.z;
    }
};

// ****************************************************************************
//! @brief Dense list of beliefs, matched to detections by label.
//!
//! Objects of known size are assumed to rest on a support: an observation
//! refines their horizontal position and keeps their height. Without a 3D
//! point, a detection center is lifted by intersecting its pixel ray with the
//! top plane of the object.
// ****************************************************************************
class WorldModel
{
public:

    // -------------------------------------------------------------------------
    //! @brief Declares an object with a prior position (or resets the prior).
    // -------------------------------------------------------------------------
    WorldObject&
    add(std::string p_name, Vector3 p_position, Vector3 p_size = {});

    [[nodiscard]] WorldObject* find(std::string_view p_name);
    [[nodiscard]] WorldObject const* find(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    //! @brief Rejects observations farther than @p_distance (m) from the
    //! current belief: a detector matching the wrong thing (a robot link of
    //! the same color) must not teleport an object. Unbounded by default.
    // -------------------------------------------------------------------------
    void gate(double p_distance)
    {
        m_gate = p_distance;
    }

    [[nodiscard]] double gate() const
    {
        return m_gate;
    }

    // -------------------------------------------------------------------------
    //! @brief Records that @p_name was seen at @p_point (robot base frame).
    //! Unknown names and observations outside the @ref gate are ignored.
    //! @return True when the belief was updated.
    // -------------------------------------------------------------------------
    bool observe(std::string_view p_name,
                 Vector3 const& p_point,
                 float p_confidence,
                 Seconds p_stamp);

    //! @brief Overwrites the belief (oracle / RL). No gate. Stamps
    //! @p_stamp so Detect sees a fresh observation.
    void place(std::string_view p_name,
               Vector3 const& p_position,
               Seconds p_stamp);

    // -------------------------------------------------------------------------
    //! @brief Folds the detections of one frame into the beliefs.
    // -------------------------------------------------------------------------
    void update(Detections const& p_detections);

    [[nodiscard]] std::span<WorldObject const> objects() const
    {
        return m_objects;
    }

    void clear()
    {
        m_objects.clear();
    }

private:

    std::vector<WorldObject> m_objects;
    double m_gate = std::numeric_limits<double>::infinity();
};

} // namespace robotik
