// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Grasp/GraspModel.hpp"

#include "Robotik/Robot/Backends/RobotBackend.hpp"
#include "Robotik/Robot/Components/Objects.hpp"
#include "Robotik/Robot/Components/Queries.hpp"
#include "Robotik/Robot/Robot.hpp"

#include "Compages/World/World.hpp"

#include <algorithm>

namespace robotik
{

/// Largest gap between the suction tip and the top face of an object that
/// still grasps it (same as the abstract level).
static constexpr double CONTACT_DISTANCE_M = 0.02;

std::string RigidGrasp::weldName(std::string const& p_gripper,
                                 std::string const& p_object)
{
    return "robotik_grasp_" + p_gripper + "_" + p_object;
}

void RigidGrasp::update(Robot& p_robot)
{
    RobotBackend* backend = p_robot.backend();
    if (backend == nullptr || m_objects.empty())
    {
        return;
    }
    compages::world::World& world = p_robot.world();
    ActuatorSet const& actuators = p_robot.actuators();
    for (std::size_t i = 0; i < actuators.size(); ++i)
    {
        auto* gripper = dynamic_cast<VacuumGripper*>(&actuators[i]);
        if (gripper == nullptr)
        {
            continue;
        }
        // The object held, if it is one of ours.
        Graspable const* held = nullptr;
        if (gripper->holding() && world.alive(gripper->held()))
        {
            ecs::Shape const* shape =
                world.entity(gripper->held()).find<ecs::Shape>();
            for (Graspable const& object : m_objects)
            {
                held = shape != nullptr && object.id == shape->name ? &object
                                                                    : held;
            }
            if (held == nullptr)
            {
                continue; // Held by another model.
            }
        }

        if (!gripper->suction() && held != nullptr)
        {
            // Release: the weld goes, the object falls.
            (void)backend->weld(weldName(gripper->name(), held->id), false);
            gripper->held({});
        }
        else if (gripper->suction() && !gripper->holding())
        {
            Vector3 const tip = p_robot.base().pose * gripper->tip(p_robot);
            Graspable const* best = nullptr;
            double best_gap = CONTACT_DISTANCE_M;
            compages::world::EntityId best_id{};
            for (Graspable const& object : m_objects)
            {
                compages::world::Entity const entity =
                    findObject(world, object.id);
                if (!entity)
                {
                    continue;
                }
                // The top or the bottom face: a cup takes it from above or
                // below.
                auto const at = entity.position();
                Vector3 const center(static_cast<double>(at.x),
                                     static_cast<double>(at.y),
                                     static_cast<double>(at.z));
                Vector3 const half(0.0, 0.0, object.half_height);
                double const gap = std::min((center + half - tip).norm(),
                                            (center - half - tip).norm());
                if (gap <= best_gap)
                {
                    best = &object;
                    best_gap = gap;
                    best_id = entity.id();
                }
            }
            if (best != nullptr &&
                backend->weld(weldName(gripper->name(), best->id), true))
            {
                gripper->held(best_id);
            }
        }
    }
}

} // namespace robotik
