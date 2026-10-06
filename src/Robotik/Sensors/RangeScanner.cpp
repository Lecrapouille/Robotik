// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Sensors/RangeScanner.hpp"

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Robot/Robot.hpp"

#include "Compages/World/World.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robotik
{

namespace
{

struct Box
{
    Vector3 low;
    Vector3 high;
};

//! Slab test: distance to an axis-aligned box, or infinity.
double hit(Box const& p_box, Vector3 const& p_origin, Vector3 const& p_dir)
{
    double near = 0.0;
    double far = std::numeric_limits<double>::infinity();
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        double const o = p_origin[axis];
        double const d = p_dir[axis];
        if (std::abs(d) < 1e-12)
        {
            if (o < p_box.low[axis] || o > p_box.high[axis])
            {
                return std::numeric_limits<double>::infinity();
            }
            continue;
        }
        double t0 = (p_box.low[axis] - o) / d;
        double t1 = (p_box.high[axis] - o) / d;
        if (t0 > t1)
        {
            std::swap(t0, t1);
        }
        near = std::max(near, t0);
        far = std::min(far, t1);
        if (near > far)
        {
            return std::numeric_limits<double>::infinity();
        }
    }
    return near;
}

} // namespace

RangeScanner::RangeScanner(std::string p_name, RangeScannerConfig p_config)
    : Sensor(std::move(p_name), p_config.frequency),
      m_config(std::move(p_config))
{
    m_config.beams = std::max(m_config.beams, 1u);
}

bool RangeScanner::sample(Robot const& p_robot, Seconds p_now)
{
    Pose const pose = p_robot.worldPose(m_config.parent) * m_config.mount;
    double const max_range = m_config.max_range.value();
    double const fov = m_config.field_of_view.value();
    bool const full_turn = fov >= 2.0 * std::numbers::pi - 1e-9;
    double const increment =
        m_config.beams > 1
            ? fov / (full_turn ? m_config.beams : m_config.beams - 1)
            : 0.0;
    double const angle_min = m_config.beams > 1 ? -0.5 * fov : 0.0;

    // Scene objects are axis-aligned boxes at their world position.
    std::vector<Box> boxes;
    p_robot.world().each<ecs::SceneObject>(
        [&boxes](compages::world::Entity p_entity,
                 ecs::SceneObject const& p_object)
        {
            auto const center = p_entity.worldPosition();
            Vector3 const half(0.5 * p_object.size[0].value(),
                               0.5 * p_object.size[1].value(),
                               0.5 * p_object.size[2].value());
            Vector3 const c(center.x, center.y, center.z);
            boxes.push_back({ c - half, c + half });
        });

    RobotBackend const* backend = p_robot.backend();
    m_scan.ranges.resize(m_config.beams);
    for (std::uint32_t i = 0; i < m_config.beams; ++i)
    {
        double const angle = angle_min + i * increment;
        Vector3 const direction =
            pose.rotation * Vector3(std::cos(angle), std::sin(angle), 0.0);
        double range = max_range;
        if (backend != nullptr)
        {
            range = std::min(range, backend->raycast(pose.position, direction,
                                                     max_range)
                                        .value_or(max_range));
        }
        for (Box const& box : boxes)
        {
            range = std::min(range, hit(box, pose.position, direction));
        }
        if (m_config.noise > 0.0 && range < max_range)
        {
            range = std::clamp(m_random.normal(range, m_config.noise), 0.0,
                               max_range);
        }
        m_scan.ranges[i] = static_cast<float>(range);
    }
    m_scan.angle_min = Radians(angle_min);
    m_scan.increment = Radians(increment);
    m_scan.max_range = m_config.max_range;
    m_scan.stamp = p_now;

    compages::world::Entity holder =
        m_config.parent.empty() ? p_robot.root() : p_robot.link(m_config.parent);
    if (holder)
    {
        holder.set(m_scan);
    }
    return true;
}

} // namespace robotik
