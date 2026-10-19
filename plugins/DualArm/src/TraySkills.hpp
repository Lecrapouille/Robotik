// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file TraySkills.hpp
/// @brief Skills of the tray scenario: a smooth straight move of the tool
/// (glasses on a tray do not like jerks), and a path drilled on the tray
/// that follows the arm carrying it.
#pragma once

#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Skills/Skill.hpp"

#include <optional>
#include <string>
#include <vector>

namespace dual_arm
{

// ****************************************************************************
/// @brief Moves the tool in a straight line, orientation kept, with a
/// minimum-jerk profile over a fixed time: by an offset, under or above an
/// object of the beliefs at the same height, or under its bottom face. The
/// speed limits of the joints may make it slower, never faster.
// ****************************************************************************
class PathSkill final: public robotik::Skill
{
public:

    /// @brief By @p_offset (m, in the robot base frame).
    PathSkill(robotik::Vector3 const& p_offset, Seconds p_duration);

    /// @brief Above the believed centre of @p_object, at the same height.
    PathSkill(std::string p_object, Seconds p_duration);

    /// @brief @p_below under the believed bottom face of @p_object.
    PathSkill(std::string p_object, Length p_below, Seconds p_duration);

    void reset() override
    {
        m_started = false;
        m_why.clear();
    }

    robotik::Status tick(robotik::RobotContext& p_context,
                         Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

    [[nodiscard]] std::string why() const override
    {
        return m_why;
    }

private:

    robotik::Vector3 m_offset = robotik::zero3();
    std::string m_object;
    /// Under the bottom face of the object by this much, if set.
    std::optional<double> m_below;
    Seconds m_duration;
    bool m_started = false;
    Seconds m_elapsed{};
    robotik::Pose m_from{ robotik::zero3(), robotik::Quaternion{} };
    robotik::Vector3 m_to = robotik::zero3();
    std::string m_why;
};

// ****************************************************************************
/// @brief Touches a tray that another arm carries with the tool, presses on
/// it with @p_force and follows @p_path on its top at @p_speed, then lifts.
///
/// The contact is felt, not assumed: the tool goes down until the force
/// sensor of the robot feels the tray, then it moves up or down to keep the
/// force (an admittance). The believed tray gives only the path in x and y.
///
/// The two arms are synchronized: the path is drawn in the frame of the
/// tool of the carrying arm, followed through what that arm measures of
/// itself (@ref robotik::RobotContext::team), not through the camera. The
/// spindle of a drill stays still: a turning bit would drag the tray (and
/// its glasses) by friction.
// ****************************************************************************
class TraceSkill final: public robotik::Skill
{
public:

    /// @param p_path Points in the frame of the tray: its centre, axes of
    /// the world (the carrier keeps it level and does not turn it).
    TraceSkill(std::string p_tray,
               std::vector<robotik::Vector3> p_path,
               LinearVelocity p_speed,
               Force p_force,
               Seconds p_timeout = Seconds(40.0));

    void reset() override
    {
        m_phase = Phase::Approach;
        m_offsets.clear();
        m_height = 0.0;
        m_travel = 0.0;
        m_tare = robotik::zero3();
        m_elapsed = Seconds{};
        m_why.clear();
    }

    robotik::Status tick(robotik::RobotContext& p_context,
                         Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

    [[nodiscard]] std::string why() const override
    {
        return m_why;
    }

private:

    /// From above the first point, down onto the tray, along the path, up.
    enum class Phase
    {
        Approach,
        Descend,
        Follow,
        Lift,
    };

    /// @brief Point of the path at @p_travel metres from its start, in the
    /// tool frame of the carrier.
    [[nodiscard]] robotik::Vector3 along(double p_travel) const;

    std::string m_tray;
    std::vector<robotik::Vector3> m_path;
    LinearVelocity m_speed;
    Force m_force;
    Seconds m_timeout;
    Phase m_phase = Phase::Approach;
    /// Points of the path in the tool frame of the carrying arm.
    std::vector<robotik::Vector3> m_offsets;
    /// Height of the tool over the path (m), negative when it presses.
    double m_height = 0.0;
    /// Force read before the contact (the weight of the tool), subtracted.
    robotik::Vector3 m_tare = robotik::zero3();
    /// Length of the path done (m).
    double m_travel = 0.0;
    Seconds m_elapsed{};
    std::string m_why;
};

} // namespace dual_arm
