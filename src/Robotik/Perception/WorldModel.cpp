// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Perception/WorldModel.hpp"

#include <cmath>

namespace robotik
{

WorldObject&
WorldModel::add(std::string p_name, Vector3 p_position, Vector3 p_size)
{
    WorldObject* object = find(p_name);
    if (object == nullptr)
    {
        object = &m_objects.emplace_back();
        object->name = std::move(p_name);
    }
    object->position = p_position;
    object->size = p_size;
    object->confidence = 0.0f;
    object->seen = Seconds(-1.0);
    object->observations = 0;
    return *object;
}

WorldObject* WorldModel::find(std::string_view p_name)
{
    for (WorldObject& object : m_objects)
    {
        if (object.name == p_name)
        {
            return &object;
        }
    }
    return nullptr;
}

WorldObject const* WorldModel::find(std::string_view p_name) const
{
    return const_cast<WorldModel*>(this)->find(p_name);
}

bool WorldModel::observe(std::string_view p_name,
                         Vector3 const& p_point,
                         float p_confidence,
                         Seconds p_stamp)
{
    WorldObject* object = find(p_name);
    if (object == nullptr)
    {
        return false;
    }
    Vector3 next = p_point;
    if (object->size.z > 0.0)
    {
        next.z = object->position.z;
    }
    if ((next - object->position).norm() > m_gate)
    {
        return false;
    }
    object->position = next;
    object->confidence = p_confidence;
    object->seen = p_stamp;
    ++object->observations;
    return true;
}

void WorldModel::update(Detections const& p_detections)
{
    Pose const& camera = p_detections.camera;
    for (Detection const& detection : p_detections.items)
    {
        WorldObject const* object = find(detection.label);
        if (object == nullptr)
        {
            continue;
        }

        Vector3 point;
        if (detection.pose)
        {
            point = camera * detection.pose->position;
        }
        else if (detection.position)
        {
            point = camera * *detection.position;
        }
        else
        {
            // Monocular: the center lies on the top plane of the object.
            Vector3 const ray = camera.rotation.rotate(p_detections.intrinsics.ray(
                detection.center[0], detection.center[1]));
            double const height = object->top() - camera.position.z;
            if (std::abs(ray.z) < 1e-9 || height / ray.z <= 0.0)
            {
                continue;
            }
            point = camera.position + ray * (height / ray.z);
        }
        (void)observe(detection.label, point, detection.confidence, p_detections.stamp);
    }
}

} // namespace robotik
