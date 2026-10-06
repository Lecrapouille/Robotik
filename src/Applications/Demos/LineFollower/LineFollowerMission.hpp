// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "DiffDrive.hpp"
#include "Floor.hpp"
#include "Skills.hpp"
#include "Track.hpp"

#include "Robotik/Runtime/Mission.hpp"
#include "Robotik/Runtime/Scheduler.hpp"

#include <cstdint>
#include <memory>
#include <vector>

class LineFollowerMission final: public robotik::Mission
{
public:

    explicit LineFollowerMission(double p_laps = 1.0);

    void setup(robotik::Simulation& p_simulation,
               robotik::SceneView* p_view) override;
    void reset(robotik::Simulation& p_simulation, robotik::Seed p_seed) override;
    void step(robotik::Simulation& p_simulation, Seconds p_dt) override;
    void measure(robotik::Simulation const& p_simulation,
                 robotik::Metrics& p_metrics) const override;

    [[nodiscard]] Track const& track() const
    {
        return m_track;
    }

    [[nodiscard]] FloorMap const& floor() const
    {
        return *m_floor;
    }

    [[nodiscard]] DiffDriveBackend const* drive() const
    {
        return m_drive;
    }

    [[nodiscard]] Navigation const& navigation() const
    {
        return *m_navigation;
    }

    [[nodiscard]] double followProgress() const;

    [[nodiscard]] std::vector<Pose2> const& truthPath() const
    {
        return m_truth_path;
    }

    [[nodiscard]] std::vector<Pose2> const& estimatePath() const
    {
        return m_estimate_path;
    }

    [[nodiscard]] double fixErrorMean() const;
    [[nodiscard]] double fixErrorMax() const
    {
        return m_fix_error_max;
    }

    [[nodiscard]] double crossTrackMax() const
    {
        return m_cross_max;
    }

private:

    void placeTags();
    void bindCamera(robotik::Simulation& p_simulation);

private:

    double m_laps;
    Track m_track;
    std::vector<FloorTag> m_tags;
    std::unique_ptr<FloorMap> m_floor;
    std::unique_ptr<FloorCamera> m_source;
    DiffDriveBackend* m_drive = nullptr;
    std::unique_ptr<Navigation> m_navigation;
    robotik::Motor* m_left = nullptr;
    robotik::Motor* m_right = nullptr;
    robotik::SkillId m_follow = robotik::NO_SKILL;
    FollowLineSkill const* m_follow_skill = nullptr;
    std::uint32_t m_steps = 0;
    double m_following = 0.0;
    bool m_was_following = false;
    double m_cross_sum = 0.0;
    double m_cross_max = 0.0;
    std::size_t m_cross_count = 0;
    double m_fix_error_sum = 0.0;
    double m_fix_error_max = 0.0;
    std::size_t m_fix_count = 0;
    std::vector<Pose2> m_truth_path;
    std::vector<Pose2> m_estimate_path;
};
