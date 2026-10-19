// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PainterSkills.hpp
/// @brief Dip, stroke and rinse. The color is chosen when the skill starts.
#pragma once

#include "PainterScene.hpp"

#include "Robotik/Runtime/Scheduler.hpp"
#include "Robotik/Skills/MotionSkills.hpp"

#include "BlackThorn/BlackThorn.hpp"

#include <map>
#include <string>
#include <vector>

namespace painter
{

class DipSkill final: public robotik::Skill
{
public:

    explicit DipSkill(Stroke& p_stroke);

    void use(std::string p_color);

    void reset() override;
    robotik::Status tick(robotik::RobotContext& p_context,
                         robotik::Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

private:

    Stroke* m_stroke = nullptr;
    std::string m_color = "red";
    std::vector<robotik::Pose> m_path;
    std::size_t m_index = 0;
    robotik::MoveTCPSkill m_move;
    bool m_ready = false;
};

/// @brief Follows the stroke of the chosen colour: the scenario trajectory of
/// that name, already in the robot base frame.
class MarkSkill final: public robotik::Skill
{
public:

    using Strokes =
        std::map<std::string, std::vector<robotik::Pose>, std::less<>>;

    /// @brief How the brush follows a stroke.
    struct Settings
    {
        /// Longest Cartesian step (m) between two IK targets. Each target
        /// is reached in joint space: without small steps the brush draws
        /// arcs, not lines. Zero: the waypoints only.
        double step = 0.002;
        /// Joint tolerance (rad) to go on to the next target.
        double tolerance = 2e-2;
    };

    MarkSkill(Stroke& p_stroke, Strokes p_strokes, Settings p_settings);

    void use(std::string p_color);

    void reset() override;
    robotik::Status tick(robotik::RobotContext& p_context,
                         robotik::Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

private:

    Stroke* m_stroke = nullptr;
    Strokes m_strokes;
    std::string m_color = "red";
    Settings m_settings;
    std::vector<robotik::Pose> m_path;
    /// Targets of the path that lay paint: from the second waypoint to the
    /// one before last.
    std::size_t m_paint_begin = 0;
    std::size_t m_paint_end = 0;
    std::size_t m_index = 0;
    robotik::MoveTCPSkill m_move;
    bool m_ready = false;
};

class RinseSkill final: public robotik::Skill
{
public:

    explicit RinseSkill(Stroke& p_stroke);

    void reset() override;
    robotik::Status tick(robotik::RobotContext& p_context,
                         robotik::Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

private:

    Stroke* m_stroke = nullptr;
    std::vector<robotik::Pose> m_path;
    std::size_t m_index = 0;
    robotik::MoveTCPSkill m_move;
    bool m_ready = false;
};

/// @brief Tree actions Dip, Mark and Rinse. Each reads @c color on its
/// blackboard.
void registerNodes(bt::NodeFactory& p_factory,
                   robotik::SkillScheduler& p_scheduler);

} // namespace painter
