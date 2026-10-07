// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Skills/PickPlaceSkills.hpp"

#include "Robotik/Scene/ContainerBounds.hpp"
#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/ECS/Queries.hpp"
#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include "Compages/Core/Units.hpp"
#include "Compages/World/World.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string_view>

namespace robotik
{

static constexpr Length kDefaultCubeHalfHeight{ 0.02 };
static constexpr Length kTransitClearanceThreshold{ 0.08 };
static constexpr Length kMaxFinalApproachClearance{ 0.03 };

//! @brief If the gripper is not holding an object, return the default cube half
//! height. Otherwise, return the half height of the object.
static Length carriedCubeHalfHeight(compages::world::World& p_world,
                                    VacuumGripper const& p_gripper)
{
    if (!p_gripper.holding() || !p_world.alive(p_gripper.held()))
    {
        return kDefaultCubeHalfHeight;
    }
    ecs::SceneObject const* shape =
        p_world.entity(p_gripper.held()).find<ecs::SceneObject>();
    if (shape == nullptr)
    {
        return kDefaultCubeHalfHeight;
    }
    return shape->size[2] * 0.5;
}

//! @brief If the clearance is greater than the transit clearance threshold,
//! return the clearance. Otherwise, return the minimum of the clearance and the
//! final approach clearance.
static Length clearanceAboveCavity(Length p_clearance)
{
    // Transit keeps full clearance; Reach (≈ 0) caps at 3 cm above the cavity.
    if (p_clearance >= kTransitClearanceThreshold)
    {
        return p_clearance;
    }
    return std::min(p_clearance, kMaxFinalApproachClearance);
}

//! TCP height over an open box. @p_at is the believed centre (perception or
//! the scenario prior). The wall layout comes from the named entity's shape.
static std::optional<Length>
tcpHeightOverOpenBox(compages::world::World& p_world,
                     std::string_view p_object_name,
                     VacuumGripper const& p_gripper,
                     Length p_clearance,
                     Vector3 const& p_at)
{
    compages::world::Entity const box = findObject(p_world, p_object_name);
    if (!box)
    {
        return std::nullopt;
    }

    ecs::SceneObject const* scene = box.find<ecs::SceneObject>();
    if (scene == nullptr || scene->type != ecs::SceneObject::Type::BOX)
    {
        return std::nullopt;
    }

    std::optional<scene::ContainerInner> const inner =
        scene::innerBounds(*scene, p_at);
    if (!inner)
    {
        return std::nullopt;
    }

    Length const cube_half = carriedCubeHalfHeight(p_world, p_gripper);
    Length const above = clearanceAboveCavity(p_clearance);
    return Length(inner->floor_z) + cube_half * 2.0 + above +
           p_gripper.length();
}

VacuumGripper* findGripper(Robot const& p_robot, std::string const& p_name)
{
    return p_name.empty() ? p_robot.actuators().first<VacuumGripper>()
                          : p_robot.actuators().find<VacuumGripper>(p_name);
}

ApproachSkill::ApproachSkill(std::string p_object,
                             Length p_clearance,
                             std::string p_gripper)
    : m_object(std::move(p_object)),
      m_clearance(p_clearance),
      m_gripper(std::move(p_gripper))
{
}

void ApproachSkill::reset()
{
    m_planned = false;
    m_move.reset();
}

Status ApproachSkill::tick(RobotContext& p_context, Seconds p_dt)
{
    if (!m_planned)
    {
        // Believed pose from perception (or oracle), not ECS ground truth.
        WorldObject const* object = p_context.world_model.find(m_object);
        VacuumGripper const* gripper = findGripper(p_context.robot, m_gripper);
        if (object == nullptr || gripper == nullptr)
        {
            return Status::FAILURE;
        }

        // Default target: hover above the object's reported top + clearance.
        Vector3 at = object->position;
        double const yaw = std::atan2(at.y, at.x);
        Length height = Length(object->top()) + m_clearance + gripper->length();

        // If the object is a box, calculate the TCP height above the inner
        // floor.
        compages::world::World& world = p_context.robot.world();
        if (std::optional<Length> over_box = tcpHeightOverOpenBox(
                world, m_object, *gripper, m_clearance, at))
        {
            height = *over_box;
        }

        // Create the target pose.
        Pose const target{
            { at.x, at.y, height.value() },
            { 0.0, std::cos(yaw * 0.5), std::sin(yaw * 0.5), 0.0 }
        };

        // Create the move skill.
        m_move = MoveTCPSkill(gripper->link(), target);
        m_planned = true;
    }

    // Tick the move skill.
    return m_move.tick(p_context, p_dt);
}

void ApproachSkill::cancel(RobotContext& p_context)
{
    m_move.cancel(p_context);
}

GraspSkill::GraspSkill(std::string p_object,
                       std::string p_gripper,
                       Seconds p_timeout)
    : m_object(std::move(p_object)),
      m_gripper(std::move(p_gripper)),
      m_timeout(p_timeout)
{
}

Status GraspSkill::tick(RobotContext& p_context, Seconds p_dt)
{
    // Find the gripper by name.
    VacuumGripper* gripper = findGripper(p_context.robot, m_gripper);
    if (gripper == nullptr)
    {
        return Status::FAILURE;
    }

    // Turn on the vacuum.
    gripper->suction(true);

    // Success when the held entity matches the requested object name.
    if (gripper->holding())
    {
        compages::world::World& world = p_context.robot.world();

        // Check if the held entity is alive and matches the requested object
        // name.
        auto const* held =
            world.alive(gripper->held())
                ? world.entity(gripper->held()).find<ecs::SceneObject>()
                : nullptr;

        // If the held entity is alive and matches the requested object name,
        // return success.
        if (held != nullptr && held->name == m_object)
        {
            return Status::SUCCESS;
        }
    }

    // If the timeout has been reached, turn off the vacuum and return failure.
    m_waited += p_dt;
    if (m_waited > m_timeout)
    {
        gripper->suction(false);
        return Status::FAILURE;
    }

    // Otherwise, return running.
    return Status::RUNNING;
}

void GraspSkill::cancel(RobotContext& p_context)
{
    if (VacuumGripper* gripper = findGripper(p_context.robot, m_gripper);
        gripper != nullptr && !gripper->holding())
    {
        gripper->suction(false);
    }
}

ReleaseSkill::ReleaseSkill(std::string p_gripper, Seconds p_timeout)
    : m_gripper(std::move(p_gripper)), m_timeout(p_timeout)
{
}

Status ReleaseSkill::tick(RobotContext& p_context, Seconds p_dt)
{
    // Find the gripper by name.
    VacuumGripper* gripper = findGripper(p_context.robot, m_gripper);
    if (gripper == nullptr)
    {
        return Status::FAILURE;
    }

    // Turn off the vacuum.
    gripper->suction(false);
    if (!gripper->holding())
    {
        return Status::SUCCESS;
    }

    // Timeout has been reached ?
    m_waited += p_dt;
    return m_waited > m_timeout ? Status::FAILURE : Status::RUNNING;
}

DetectSkill::DetectSkill(std::string p_object, Seconds p_timeout)
    : m_object(std::move(p_object)), m_timeout(p_timeout)
{
}

Status DetectSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    // If the skill has not started, start it and record the observations at the
    // start.
    if (!m_started)
    {
        m_start = p_context.time;
        m_started = true;

        // Ignore detections from before this attempt (e.g. stale camera frame).
        if (WorldObject const* prior = p_context.world_model.find(m_object))
        {
            m_observations_at_start = prior->observations;
        }
    }

    // Find the object by name.
    WorldObject const* object = p_context.world_model.find(m_object);
    if (object == nullptr)
    {
        return Status::FAILURE;
    }

    // Check if the object is fresh and has been seen since the start.
    bool const fresh = object->observations > m_observations_at_start;
    if (fresh && object->seen >= m_start)
    {
        return Status::SUCCESS;
    }

    // Timeout has been reached ?
    return p_context.time - m_start > m_timeout ? Status::FAILURE
                                                : Status::RUNNING;
}

} // namespace robotik
