// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "LineFollowerMission.hpp"

#include "Vision.hpp"

#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Perception/Localization.hpp"
#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Runtime/Resources.hpp"
#include "Robotik/Runtime/Simulation.hpp"
#include "Robotik/Sensors/Camera.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

#define TAG_SIZE_M 0.16
#define TAG_OFFSET_M 0.30
#define FOLLOW_SPEED 0.30
#define CROSS_TRACK_SETTLE_S 2.0
#define CAMERA_PITCH_DEG 40.0
#define SKILL_PRIORITY 100

LineFollowerMission::LineFollowerMission(double p_laps) : m_laps(p_laps)
{
    placeTags();
    m_floor = std::make_unique<FloorMap>(m_track, m_tags, TAG_SIZE_M);
}

void LineFollowerMission::placeTags()
{
    int const count = 12;
    for (int id = 0; id < count; ++id)
    {
        Track::Point const p =
            m_track.at((id + 0.5) * m_track.perimeter() / count);
        double const side = (id % 2 == 0) ? TAG_OFFSET_M : -TAG_OFFSET_M;
        m_tags.push_back({ id,
                           p.x + side * std::sin(p.heading),
                           p.y - side * std::cos(p.heading) });
    }
}

void LineFollowerMission::bindCamera(robotik::Simulation& p_simulation)
{
    robotik::Camera* camera = p_simulation.camera();
    if (camera == nullptr)
    {
        throw std::runtime_error("Line follower needs a camera");
    }
    double const pitch = CAMERA_PITCH_DEG * std::numbers::pi / 180.0;
    double const c = std::cos(pitch);
    double const s = std::sin(pitch);
    camera->mount(robotik::Pose{
        { 0.2, 0.0, 0.2 },
        robotik::basis({ 0.0, -1.0, 0.0 },
                       { -s, 0.0, -c },
                       { c, 0.0, -s }) });
    m_source = std::make_unique<FloorCamera>(*m_floor, *m_drive);
    camera->source(m_source.get());
    camera->onFrame(
        [this, &p_simulation](robotik::CameraFrame const& p_frame)
        {
            if (auto fix = robotik::localize(
                    p_simulation.perception().detections(),
                    m_floor->landmarks());
                fix && m_navigation->fix(*fix, p_frame.stamp))
            {
                Pose2 const& truth = m_drive->truth();
                double const error =
                    std::hypot(fix->pose.position.x - truth.x,
                               fix->pose.position.y - truth.y);
                m_fix_error_sum += error;
                m_fix_error_max = std::max(m_fix_error_max, error);
                ++m_fix_count;
            }
        });
}

void LineFollowerMission::setup(robotik::Simulation& p_simulation,
                                robotik::SceneView* p_view)
{
    auto backend = std::make_unique<DiffDriveBackend>("left_wheel_joint",
                                                      "right_wheel_joint");
    m_drive = backend.get();
    p_simulation.robot().connect(std::move(backend));

    m_left = p_simulation.robot().actuators().find<robotik::Motor>("left_wheel");
    m_right =
        p_simulation.robot().actuators().find<robotik::Motor>("right_wheel");
    if (m_left == nullptr || m_right == nullptr)
    {
        throw std::runtime_error(
            "Line follower needs left_wheel and right_wheel motors");
    }

    p_simulation.perception().add<AprilTagDetector>(TAG_SIZE_M);
    p_simulation.perception().add<LineDetector>();
    bindCamera(p_simulation);

    m_navigation =
        std::make_unique<Navigation>(m_track, m_drive->geometry());

    robotik::ResourceManager& resources = p_simulation.robot().resources();
    robotik::SkillDescription describe;
    describe.priority = SKILL_PRIORITY;
    describe.resources = { resources.require("left_wheel"),
                           resources.require("right_wheel"),
                           resources.require("front_camera",
                                             robotik::Access::Shared) };

    Wheels const wheels{ *m_left, *m_right, m_navigation->model };
    describe.name = "Localize";
    p_simulation.skills().add<LocalizeSkill>(describe, wheels, *m_navigation);
    describe.name = "ReachLine";
    p_simulation.skills().add<ReachLineSkill>(describe, wheels, *m_navigation);
    describe.name = "FollowLine";
    m_follow = p_simulation.skills().add<FollowLineSkill>(
        describe,
        wheels,
        *m_navigation,
        p_simulation.perception(),
        m_laps,
        FOLLOW_SPEED);
    m_follow_skill = &static_cast<FollowLineSkill const&>(
        p_simulation.skills().skill(m_follow));

    if (p_view != nullptr)
    {
        robotik::Image const& image = m_floor->image();
        double const width = image.width() * 0.004;
        double const height = image.height() * 0.004;
        p_view->ground(image, width, height);
    }
}

void LineFollowerMission::reset(robotik::Simulation& p_simulation,
                                robotik::Seed p_seed)
{
    robotik::Random start(p_seed.derive("start"));
    Track::Point const near = m_track.at(start.uniform(0.0, m_track.perimeter()));
    double const offset = start.uniform(-0.3, 0.3);
    m_drive->start({ near.x - offset * std::sin(near.heading),
                     near.y + offset * std::cos(near.heading),
                     near.heading + start.uniform(-0.8, 0.8) });
    p_simulation.robot().startPose(m_drive->pose());

    robotik::Random calibration(p_seed.derive("odometry"));
    DriveGeometry model = m_drive->geometry();
    model.radius *= 1.0 + calibration.uniform(-0.03, 0.03);
    model.track *= 1.0 + calibration.uniform(-0.05, 0.05);
    m_navigation->model = model;
    m_navigation->estimate = {};
    m_navigation->fixes = 0;
    m_navigation->rejected = 0;
    m_navigation->streak = 0;
    m_navigation->last_fix = Seconds(-1.0);

    m_following = 0.0;
    m_was_following = false;
    m_cross_sum = 0.0;
    m_cross_max = 0.0;
    m_cross_count = 0;
    m_fix_error_sum = 0.0;
    m_fix_error_max = 0.0;
    m_fix_count = 0;
    m_steps = 0;
    m_truth_path.clear();
    m_estimate_path.clear();
}

void LineFollowerMission::step(robotik::Simulation& p_simulation, Seconds p_dt)
{
    if (m_left == nullptr || m_navigation == nullptr)
    {
        return;
    }
    m_navigation->odometry(
        m_left->velocity().value(), m_right->velocity().value(), p_dt.value());

    robotik::SkillScheduler const& skills = p_simulation.skills();
    bool const following =
        m_follow != robotik::NO_SKILL &&
        skills.state(m_follow) == robotik::SkillState::Running;
    if (following)
    {
        if (!m_was_following)
        {
            m_cross_sum = 0.0;
            m_cross_max = 0.0;
            m_cross_count = 0;
        }
        m_was_following = true;
        m_following += p_dt.value();
    }
    else
    {
        m_following = 0.0;
        m_was_following = false;
    }
    if (m_following > CROSS_TRACK_SETTLE_S)
    {
        Pose2 const& truth = m_drive->truth();
        DriveGeometry const& model = m_navigation->model;
        double const ax = truth.x + model.axle * std::cos(truth.yaw);
        double const ay = truth.y + model.axle * std::sin(truth.yaw);
        Track::Point const on = m_track.closest(ax, ay);
        double const error = std::hypot(ax - on.x, ay - on.y);
        m_cross_sum += error;
        m_cross_max = std::max(m_cross_max, error);
        ++m_cross_count;
    }
    if (++m_steps % 5u == 0u)
    {
        m_truth_path.push_back(m_drive->truth());
        m_estimate_path.push_back(m_navigation->estimate);
    }
}

void LineFollowerMission::measure(robotik::Simulation const& /*p_simulation*/,
                                  robotik::Metrics& p_metrics) const
{
    p_metrics.set("laps", followProgress() / m_track.perimeter());
    p_metrics.set("cross_track.max", m_cross_max);
    p_metrics.set("cross_track.mean",
                  m_cross_count > 0u
                      ? m_cross_sum / static_cast<double>(m_cross_count)
                      : 0.0);
    p_metrics.set("fix.error.mean", fixErrorMean());
    p_metrics.set("fix.error.max", m_fix_error_max);
    if (m_navigation)
    {
        p_metrics.set("fixes", static_cast<double>(m_navigation->fixes));
        p_metrics.set("fixes.rejected",
                      static_cast<double>(m_navigation->rejected));
    }
}

double LineFollowerMission::followProgress() const
{
    return m_follow_skill != nullptr ? m_follow_skill->progress() : 0.0;
}

double LineFollowerMission::fixErrorMean() const
{
    return m_fix_count > 0u
               ? m_fix_error_sum / static_cast<double>(m_fix_count)
               : 0.0;
}
