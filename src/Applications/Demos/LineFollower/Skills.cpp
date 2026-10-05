// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Skills.hpp"

#include "Robotik/Runtime/RobotContext.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#define LOCALIZE_YAW_RATE 0.6
#define LOCALIZE_TIMEOUT_S 40.0
#define LOCALIZE_SPEED 0.2
#define LOCALIZE_ADVANCE_S 2.5
#define NAVIGATION_GATE_M 0.30
#define NAVIGATION_RELOCALIZE 5u
#define REACH_LOOKAHEAD_M 0.45
#define REACH_SPEED 0.25
#define REACH_GAIN 2.0
#define REACH_TOLERANCE_M 0.08
#define REACH_TIMEOUT_S 20.0
#define FOLLOW_MAX_YAW_RATE 2.5
#define FOLLOW_LOST_S 0.3
#define FOLLOW_SEARCH_YAW_RATE 0.8
#define FOLLOW_GIVE_UP_S 4.0

bool Navigation::fix(robotik::Localization const& p_fix, Seconds p_stamp)
{
    if (fixes > 0u &&
        std::hypot(p_fix.pose.position.x - estimate.x, p_fix.pose.position.y - estimate.y) > NAVIGATION_GATE_M &&
        ++streak < NAVIGATION_RELOCALIZE)
    {
        ++rejected;
        return false;
    }
    streak = 0;
    estimate = { p_fix.pose.position.x, p_fix.pose.position.y, p_fix.pose.rotation.yaw() };
    ++fixes;
    last_fix = p_stamp;
    return true;
}

void Navigation::odometry(double p_left, double p_right, double p_dt)
{
    double speed = 0.0;
    double yaw_rate = 0.0;
    model.twist(p_left, p_right, speed, yaw_rate);
    estimate = model.integrate(estimate, speed, yaw_rate, p_dt);
}

void Wheels::drive(double p_speed, double p_yaw_rate)
{
    double left_speed = 0.0;
    double right_speed = 0.0;
    model.wheels(p_speed, p_yaw_rate, left_speed, right_speed);
    left.spin(left_speed);
    right.spin(right_speed);
}

void Wheels::stop()
{
    left.stop();
    right.stop();
}

void LocalizeSkill::reset()
{
    m_fixes = m_navigation.fixes;
    m_elapsed = 0.0;
    m_phase = 0.0;
}

robotik::Status LocalizeSkill::tick(robotik::RobotContext& /*p_context*/, Seconds p_dt)
{
    if (m_navigation.fixes > m_fixes)
    {
        m_wheels.stop();
        return robotik::Status::SUCCESS;
    }
    m_elapsed += p_dt.value();
    if (m_elapsed > LOCALIZE_TIMEOUT_S)
    {
        m_wheels.stop();
        return robotik::Status::FAILURE;
    }
    // A full turn, then a short straight run.
    double const turn = 2.0 * std::numbers::pi / LOCALIZE_YAW_RATE;
    m_phase += p_dt.value();
    if (m_phase > turn + LOCALIZE_ADVANCE_S)
    {
        m_phase = 0.0;
    }
    if (m_phase < turn)
    {
        m_wheels.drive(0.0, LOCALIZE_YAW_RATE);
    }
    else
    {
        m_wheels.drive(LOCALIZE_SPEED, 0.0);
    }
    return robotik::Status::RUNNING;
}

void LocalizeSkill::cancel(robotik::RobotContext& /*p_context*/)
{
    m_wheels.stop();
}

void ReachLineSkill::reset()
{
    m_started = false;
    m_elapsed = 0.0;
}

robotik::Status ReachLineSkill::tick(robotik::RobotContext& /*p_context*/, Seconds p_dt)
{
    Pose2 const& pose = m_navigation.estimate;
    if (!m_started)
    {
        m_started = true;
        Track const& track = m_navigation.track;
        m_goal = track.at(track.closest(pose.x, pose.y).s + REACH_LOOKAHEAD_M);
    }
    m_elapsed += p_dt.value();
    if (m_elapsed > REACH_TIMEOUT_S)
    {
        m_wheels.stop();
        return robotik::Status::FAILURE;
    }
    double const dx = m_goal.x - pose.x;
    double const dy = m_goal.y - pose.y;
    double const distance = std::hypot(dx, dy);
    if (distance < REACH_TOLERANCE_M)
    {
        m_wheels.stop();
        return robotik::Status::SUCCESS;
    }
    double const bearing = std::remainder(std::atan2(dy, dx) - pose.yaw, 2.0 * std::numbers::pi);
    m_wheels.drive(REACH_SPEED * std::max(std::cos(bearing), 0.0), REACH_GAIN * bearing);
    return robotik::Status::RUNNING;
}

void ReachLineSkill::cancel(robotik::RobotContext& /*p_context*/)
{
    m_wheels.stop();
}

void FollowLineSkill::reset()
{
    Pose2 const& pose = m_navigation.estimate;
    m_last_s = m_navigation.track.closest(pose.x, pose.y).s;
    m_progress = 0.0;
    m_elapsed = 0.0;
    m_turn = 0.0;
    m_seen = Seconds(-1.0);
}

robotik::Status FollowLineSkill::tick(robotik::RobotContext& p_context, Seconds p_dt)
{
    Track const& track = m_navigation.track;
    Pose2 const& pose = m_navigation.estimate;
    double const s = track.closest(pose.x, pose.y).s;
    m_progress += track.advance(m_last_s, s);
    m_last_s = s;
    if (m_progress >= m_laps * track.perimeter())
    {
        m_wheels.stop();
        return robotik::Status::SUCCESS;
    }
    m_elapsed += p_dt.value();
    if (m_elapsed > 3.0 * m_laps * track.perimeter() / m_speed)
    {
        m_wheels.stop();
        return robotik::Status::FAILURE;
    }

    // Lift the line centroid onto the floor, in the base frame.
    robotik::Detections const& detections = m_perception.detections();
    robotik::Detection const* line = detections.find("line");
    if (line != nullptr && detections.stamp > m_seen)
    {
        m_seen = detections.stamp;
        robotik::Vector3 const ray = detections.camera.rotation.rotate(
            detections.intrinsics.ray(line->center[0], line->center[1]));
        double const floor = -m_wheels.model.height;
        if (ray.z < -1e-6)
        {
            double const t = (floor - detections.camera.position.z) / ray.z;
            double const x = detections.camera.position.x + t * ray.x - m_wheels.model.axle;
            double const y = detections.camera.position.y + t * ray.y;
            m_turn = 2.0 * y / (x * x + y * y);
        }
    }
    double const age = m_seen.value() < 0.0 ? m_elapsed : (p_context.time - m_seen).value();
    if (age > FOLLOW_GIVE_UP_S)
    {
        m_wheels.stop();
        return robotik::Status::FAILURE;
    }
    if (m_seen.value() < 0.0 || age > FOLLOW_LOST_S)
    {
        m_wheels.drive(0.0, m_turn >= 0.0 ? FOLLOW_SEARCH_YAW_RATE : -FOLLOW_SEARCH_YAW_RATE);
        return robotik::Status::RUNNING;
    }
    m_wheels.drive(m_speed, std::clamp(m_speed * m_turn, -FOLLOW_MAX_YAW_RATE, FOLLOW_MAX_YAW_RATE));
    return robotik::Status::RUNNING;
}

void FollowLineSkill::cancel(robotik::RobotContext& /*p_context*/)
{
    m_wheels.stop();
}
