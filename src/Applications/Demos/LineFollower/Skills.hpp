// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "DiffDrive.hpp"
#include "Track.hpp"

#include "Robotik/Actuators/Actuator.hpp"
#include "Robotik/Perception/Detector.hpp"
#include "Robotik/Perception/Localization.hpp"
#include "Robotik/Skills/Skill.hpp"

#include <cstdint>

// The robot's own belief of its pose: wheel odometry with a calibration
// error, reset by AprilTag fixes. Once localized, a fix too far from the
// belief is taken for a misdetection and dropped, unless several in a row
// disagree: then the belief was wrong and the robot relocalizes.
struct Navigation
{
    Navigation(Track const& p_track, DriveGeometry const& p_model) : track(p_track), model(p_model)
    {
    }

    Track const& track;
    // Geometry the odometry believes in (not the true one).
    DriveGeometry model;
    Pose2 estimate;
    std::uint32_t fixes = 0;
    std::uint32_t rejected = 0;
    std::uint32_t streak = 0;
    Seconds last_fix{ -1.0 };

    // @return False when the fix is rejected.
    bool fix(robotik::Localization const& p_fix, Seconds p_stamp);
    void odometry(double p_left, double p_right, double p_dt);
};

// Both wheel motors, commanded as a unicycle.
struct Wheels
{
    robotik::Motor& left;
    robotik::Motor& right;
    DriveGeometry const& model;

    void drive(double p_speed, double p_yaw_rate);
    void stop();
};

// Turns in place until an AprilTag gives a pose fix; after a full turn
// without one, moves ahead a little and tries again.
class LocalizeSkill final: public robotik::Skill
{
public:

    LocalizeSkill(Wheels p_wheels, Navigation& p_navigation)
        : m_wheels(p_wheels), m_navigation(p_navigation)
    {
    }

    void reset() override;
    robotik::Status tick(robotik::RobotContext& p_context, Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

private:

    Wheels m_wheels;
    Navigation& m_navigation;
    std::uint32_t m_fixes = 0;
    double m_elapsed = 0.0;
    double m_phase = 0.0;
};

// Drives on the estimated pose to a point of the track a little ahead of the
// closest one, so that the line is in view and roughly aligned.
class ReachLineSkill final: public robotik::Skill
{
public:

    ReachLineSkill(Wheels p_wheels, Navigation& p_navigation)
        : m_wheels(p_wheels), m_navigation(p_navigation)
    {
    }

    void reset() override;
    robotik::Status tick(robotik::RobotContext& p_context, Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

private:

    Wheels m_wheels;
    Navigation& m_navigation;
    Track::Point m_goal;
    bool m_started = false;
    double m_elapsed = 0.0;
};

// Pure pursuit on the line seen by the camera; succeeds after @p_laps laps of
// the track measured on the estimated pose, fails when the line stays out of
// sight (the behavior tree then reaches it again).
class FollowLineSkill final: public robotik::Skill
{
public:

    FollowLineSkill(Wheels p_wheels,
                    Navigation& p_navigation,
                    robotik::PerceptionPipeline const& p_perception,
                    double p_laps,
                    double p_speed)
        : m_wheels(p_wheels),
          m_navigation(p_navigation),
          m_perception(p_perception),
          m_laps(p_laps),
          m_speed(p_speed)
    {
    }

    void reset() override;
    robotik::Status tick(robotik::RobotContext& p_context, Seconds p_dt) override;
    void cancel(robotik::RobotContext& p_context) override;

    [[nodiscard]] double progress() const
    {
        return m_progress;
    }

private:

    Wheels m_wheels;
    Navigation& m_navigation;
    robotik::PerceptionPipeline const& m_perception;
    double m_laps;
    double m_speed;
    double m_progress = 0.0;
    double m_last_s = 0.0;
    double m_elapsed = 0.0;
    double m_turn = 0.0;
    Seconds m_seen{ -1.0 };
};
