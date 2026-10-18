// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file TraySkills.hpp
/// @brief Skills of the tray scenario: a smooth straight move of the tool
/// (glasses on a tray do not like jerks), and a visit of a glass that
/// follows the arm carrying the tray.
#pragma once

#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Skills/Skill.hpp"

#include <string>

namespace dual_arm
{

// ****************************************************************************
/// @brief Moves the tool in a straight line, orientation kept, with a
/// minimum-jerk profile over a fixed time: by an offset, or above an object
/// of the beliefs at the same height. The speed limits of the joints may
/// make it slower, never faster.
// ****************************************************************************
class PathSkill final: public robotik::Skill
{
public:

    /// @brief By @p_offset (m, in the robot base frame).
    PathSkill(robotik::Vector3 const& p_offset, Seconds p_duration);

    /// @brief Above the believed centre of @p_object, at the same height.
    PathSkill(std::string p_object, Seconds p_duration);

    void reset() override
    {
        m_started = false;
        m_why.clear();
    }

    robotik::Status tick(robotik::RobotContext& p_context, Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

    [[nodiscard]] std::string why() const override
    {
        return m_why;
    }

private:

    robotik::Vector3 m_offset = robotik::zero3();
    std::string m_object;
    Seconds m_duration;
    bool m_started = false;
    Seconds m_elapsed{};
    robotik::Pose m_from{ robotik::zero3(), robotik::Quaternion{} };
    robotik::Vector3 m_to = robotik::zero3();
    std::string m_why;
};

// ****************************************************************************
/// @brief Brings the tool @p_clearance above a glass on a tray that another
/// arm carries, and stays there @p_dwell while the tray moves.
///
/// The two arms are synchronized: the target is the glass in the frame of
/// the tool of the carrying arm, followed through what that arm measures of
/// itself (@ref robotik::RobotContext::team), not through the camera.
// ****************************************************************************
class VisitSkill final: public robotik::Skill
{
public:

    VisitSkill(std::string p_glass,
               std::string p_tray,
               Length p_clearance,
               Seconds p_dwell = Seconds(1.0),
               Seconds p_timeout = Seconds(8.0));

    void reset() override
    {
        m_started = false;
        m_descending = false;
        m_elapsed = Seconds{};
        m_still = Seconds{};
        m_why.clear();
    }

    robotik::Status tick(robotik::RobotContext& p_context, Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

    [[nodiscard]] std::string why() const override
    {
        return m_why;
    }

private:

    /// Comes from above the glass: from far, the joints would cut through
    /// the tray or the other arm.
    bool m_descending = false;
    std::string m_glass;
    std::string m_tray;
    Length m_clearance;
    Seconds m_dwell;
    Seconds m_timeout;
    bool m_started = false;
    std::string m_holder;
    /// Visit point in the tool frame of the carrying arm.
    robotik::Vector3 m_offset = robotik::zero3();
    Seconds m_elapsed{};
    Seconds m_still{};
    std::string m_why;
};

} // namespace dual_arm
